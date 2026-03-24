#define GDB_SERVER_SUPPORT_
#define GDI_LAYER_DEBUG_

#include "main.h"
#include "hookRam.c"

#include "myui.c"
#include "msdc.c"
#include "uart.c"
#include "tdma.c"
#include "lcd.c"
#include "sfi.c"
#include "sej.c"
#include "gpt.c"
#include "rtc.c"
#include "sim.c"
#include "keypad.c"
#include "touchscreen.c"
#include "vmEvent.c"

void dumpCpuInfo(void);
static void moral_perf_tick(void);

/*
 * main.c：主程序入口与 Unicorn 生命周期（initMtkSimalator）、SDL 主循环。
 * 默认（MORAL_EMU_DEDICATED_THREAD）：模拟在独立 pthread 内 uc_emu_start；UI/外设 tick 在主线程 loop()。
 * hookCodeCallBack：在特定 PC 上于「指令执行前」打桩（RTC、DMA、异常路径等）。
 * hookRam.c 经 #include 并入本翻译单元，负责 UC_HOOK_MEM_* 的 MMIO（DE/FCIE/定时器/触摸等）。
 */

#ifdef GDB_SERVER_SUPPORT
#include "gdb_client.c"
pthread_t gdb_server_mutex;

void readMemoryToGdb(unsigned int addr, unsigned int length, void *buffer)
{
    uc_mem_read(MTK, addr, buffer, length);
}
void writeMemoryToGdb(unsigned int addr, char value)
{
    uc_mem_write(MTK, addr, &value, 1);
}
void writeRegToGdb(u32 reg, u32 value)
{
    if (reg == 0)
        uc_reg_write(MTK, UC_ARM_REG_R0, &value);
    else if (reg == 1)
        uc_reg_write(MTK, UC_ARM_REG_R1, &value);
    else if (reg == 2)
        uc_reg_write(MTK, UC_ARM_REG_R2, &value);
    else if (reg == 3)
        uc_reg_write(MTK, UC_ARM_REG_R3, &value);
    else if (reg == 4)
        uc_reg_write(MTK, UC_ARM_REG_R4, &value);
    else if (reg == 5)
        uc_reg_write(MTK, UC_ARM_REG_R5, &value);
    else if (reg == 6)
        uc_reg_write(MTK, UC_ARM_REG_R6, &value);
    else if (reg == 7)
        uc_reg_write(MTK, UC_ARM_REG_R7, &value);
    else if (reg == 8)
        uc_reg_write(MTK, UC_ARM_REG_R8, &value);
    else if (reg == 9)
        uc_reg_write(MTK, UC_ARM_REG_R9, &value);
    else if (reg == 10)
        uc_reg_write(MTK, UC_ARM_REG_R10, &value);
    else if (reg == 11)
        uc_reg_write(MTK, UC_ARM_REG_R11, &value);
    else if (reg == 12)
        uc_reg_write(MTK, UC_ARM_REG_R12, &value);
    else if (reg == 13)
        uc_reg_write(MTK, UC_ARM_REG_R13, &value);
    else if (reg == 14)
        uc_reg_write(MTK, UC_ARM_REG_R14, &value);
    else if (reg == 15)
        uc_reg_write(MTK, UC_ARM_REG_R15, &value);
    else if (reg == 16)
        uc_reg_write(MTK, UC_ARM_REG_CPSR, &value);
}

void ReadRegsToGdb(int *regPtr)
{
    uc_reg_read(MTK, UC_ARM_REG_R0, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R1, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R2, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R3, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R4, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R5, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R6, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R7, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R8, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R9, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R10, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R11, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R12, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R13, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R14, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_R15, regPtr++);
    uc_reg_read(MTK, UC_ARM_REG_CPSR, regPtr++);
}
#endif

/*
 * Moral I9：HalRtcGetSecondCount(0x31ad8) 通过 HalAsura + SPI 读外置 RTC，不用 MT6252 的 0x810b。
 * 在即将执行 STR R5,[R0]（0x31b9c）前覆盖 R5，使状态栏时间与主机本地时间一致。
 * 秒计数与 _ConvertBCDToSeconds 一致：自本地 2000-01-01 00:00:00 起的秒数（mktime）。
 */
static u32 moral_fw_rtc_seconds_since_2000(void)
{
    time_t now = time(NULL);
    struct tm ep;
    ep.tm_sec = 0;
    ep.tm_min = 0;
    ep.tm_hour = 0;
    ep.tm_mday = 1;
    ep.tm_mon = 0;
    ep.tm_year = 100;
    ep.tm_wday = 0;
    ep.tm_yday = 0;
    ep.tm_isdst = -1;
    time_t t0 = mktime(&ep);
    if (t0 == (time_t)-1)
        return 0;
    long long d = (long long)now - (long long)t0;
    if (d <= 0)
        return 0;
    d += (long long)MORAL_RTC_SECOND_OFFSET;
    d += (long long)MORAL_RTC_EXTRA_SECONDS;
    if (d > 0xFFFFFFFFLL)
        return 0xFFFFFFFFu;
    if (d < 0)
        return 0;
    return (u32)d;
}

u32 Interrupt_Handler_Entry; // 中断入口地址

u32 IRQ_MASK_LOW_REG = 0x3400183C;
u32 IRQ_MASK_HIGH_REG = 0x34001840;

enum
{
    SD_DMA_Read,
    SD_DMA_Write
};

u32 DMA_Data_Transfer_Ptr;    // DMA搬运数据源地址
u32 DMA_Transfer_Bytes_Count; // DMA搬运数据字节数
u32 SD_Multi_Read_Count;      // 连续读块数
u32 SD_DMA_Type;              // 0表示读取SD数据块 1表示写SD数据块

u8 ucs2Tmp[128] = {0}; // utf16-le转utf-8 缓存空间

FILE *SD_File_Handle;
/** 本次运行打开 SD 镜像时的文件字节数；写入不得超出，否则 Win 上 fwrite 会把文件拉长到数 GB */
static unsigned long long g_sd_img_max_bytes;
pthread_mutex_t mutex; // 线程锁
uint64_t lastFlashTime;
uint64_t lastTaskTime;

u32 lastSaveAdr = 0;
u32 lastSaveAdr2 = 0;

u8 isStepNext = 0;

u32 debugAddress;

SDL_Keycode isKeyDown = SDLK_UNKNOWN;
bool isMouseDown = false;

#if MORAL_EMU_DEDICATED_THREAD
pthread_t emu_thread;
volatile int moral_emu_thread_stop;
#endif

u8 currentProgramDir[256] = {0};

u32 stackCallback[17];
bool isEnterCallback;
int simulatePress = -1;
u32 lastSprintfPtr = 0;

u32 size_128mb = 1024 * 1024 * 128;
u32 size_32mb = 1024 * 1024 * 32;
u32 size_16mb = 1024 * 1024 * 16;
u32 size_8mb = 1024 * 1024 * 8;
u32 size_4mb = 1024 * 1024 * 4;
u32 size_1mb = 1024 * 1024;
u32 size_2kb = 1024 * 2;

/* initMtkSimalator 里 IDA XRAM 后备缓冲首址，供 Find* 在映像溢出段扫 magic */
static u8 *s_ida_xram_host = NULL;

u32 *isrStackPtr;
u32 isrStackList[100][17];

u32 buff1, buff2;
char *pp;

u32 sendCount;

void dumpVirtMemory(u32 addr, u32 len)
{
    uc_mem_read(MTK, addr, globalSprintfBuff, len);
    printf("dumpMemory[%x]\n", addr);
    for (u32 i = 0; i < len; i++)
    {
        printf(" %x ", globalSprintfBuff[i]);
    }
    printf("\n");
}

int utf16_len(char *utf16)
{
    int len = 0;
    while (*utf16++ != 0)
        len++;
    return len;
}

int ucs2_to_utf8(const unsigned char *in, int ilen, unsigned char *out, int olen)
{
    int length = 0;
    if (!out)
        return length;
    char *start = NULL;
    char *pout = out;
    for (start = in; start != NULL && start < in + ilen - 1; start += 2)
    {
        unsigned short ucs2_code = *(unsigned short *)start;
        if (0x0080 > ucs2_code)
        {
            /* 1 byte UTF-8 Character.*/
            if (length + 1 > olen)
                return -1;

            *pout = (char)*start;
            length++;
            pout++;
        }
        else if (0x0800 > ucs2_code)
        {
            /*2 bytes UTF-8 Character.*/
            if (length + 2 > olen)
                return -1;
            *pout = ((char)(ucs2_code >> 6)) | 0xc0;
            *(pout + 1) = ((char)(ucs2_code & 0x003F)) | 0x80;
            length += 2;
            pout += 2;
        }
        else
        {
            /* 3 bytes UTF-8 Character .*/
            if (length + 3 > olen)
                return -1;

            *pout = ((char)(ucs2_code >> 12)) | 0xE0;
            *(pout + 1) = ((char)((ucs2_code & 0x0FC0) >> 6)) | 0x80;
            *(pout + 2) = ((char)(ucs2_code & 0x003F)) | 0x80;
            length += 3;
            pout += 3;
        }
    }

    return length;
}

void keyEvent(int type, int key)
{
    int simulateKey = -1;

    if (key == 0x4000003a && type == 4)
    {
        isStepNext = 1;
    }
    if (key == 0x4000003b && type == 4)
    {
        u8 *p = SDL_malloc(size_8mb);
        uc_mem_read(MTK, 0, p, size_8mb);
        writeFile("0x0000.bin", p, size_8mb);
        printf("成功导出0x0000.bin\n");
    }
    if (key == 0x4000003c && type == 4)
    {
        u8 *p = SDL_malloc(size_8mb);
        uc_mem_read(MTK, 0xf0000000, p, size_8mb);
        writeFile("0xf000.bin", p, size_8mb);
        printf("成功导出0xf000.bin\n");
    }
    if (key == 0x4000003e && type == 4)
    {
        dumpCpuInfo();
    }
    if (key == 0x4000003f && type == 4)
    {
        EnqueueVMEvent(VM_EVENT_EXIT, 0, 0);
    }

    if (key >= 0x30 && key <= 0x39)
    { // 数字键盘1-9
        simulateKey = key - 0x30;
    }
    else if (key == 0x77) // w
    {
        simulateKey = 14; // 上
    }
    else if (key == 0x73) // s
    {
        simulateKey = 15; // 下
    }
    else if (key == 0x61) // a
    {
        simulateKey = 16; // 左
    }
    else if (key == 0x64) // d
    {
        simulateKey = 17; // 右
    }

    else if (key == 0x66) // f
    {
        simulateKey = 18; // OK
    }
    else if (key == 0x71) // q
    {
        simulateKey = 20; // 左软
    }
    else if (key == 0x65) // e
    {
        simulateKey = 21; // 右软
    }
    else if (key == 0x7a) // z
    {
        simulateKey = 22; // 拨号
    }
    else if (key == 0x63 || key == SDLK_ESCAPE) // c 或 Esc → 挂机/电源键
    {
        EnqueueVMEvent(VM_EVENT_POWER_KEY, (type == 4) ? 1 : 0, 0);
        return;
    }

    else if (key == 0x6e) // n
    {
        simulateKey = 10; // *
    }
    else if (key == 0x6d) // m
    {
        simulateKey = 11; // #
    }
    simulatePress = type == 4 ? 1 : 0;
    if (simulateKey != -1)
    {
        EnqueueVMEvent(VM_EVENT_KEYBOARD, simulateKey, simulatePress);
        simulateKey = -1;
    }
}

void mouseEvent(int type, int x, int y)
{
    if (x < 0)
        x = 0;
    else if (x > 239)
        x = 239;
    if (y < 0)
        y = 0;
    else if (y > 399)
        y = 399;
    touchX = (u32)x;
    touchY = (u32)y;
    if (type == MR_MOUSE_DOWN)
        isTouchDown = 1;
    else if (type == MR_MOUSE_UP)
        isTouchDown = 0;

    mtk_touch_regs_sync();

    moral_vm_touch_adc_request((u32)x, (u32)y);

    if (type == MR_MOUSE_MOVE)
    {
        static uint64_t last_move_enqueue = 0;
        uint64_t now = moral_get_ticks_ms();
        uint64_t min_iv = 1000 / 30;
        if (min_iv < 1)
            min_iv = 1;
        if (last_move_enqueue != 0 && (now - last_move_enqueue) < min_iv)
            return;
        last_move_enqueue = now;
    }
    EnqueueVMEvent(VM_EVENT_TOUCH_SCREEN_IRQ, type, (x << 16) | y);
}

static int s_moral_emu_halted;

static void moral_emu_on_stop(uc_err p)
{
    if (p == UC_ERR_READ_UNMAPPED)
        printf("模拟错误：此处内存不可读\n");
    else if (p == UC_ERR_WRITE_UNMAPPED)
        printf("模拟错误：此处内存不可写\n");
    else if (p == UC_ERR_FETCH_UNMAPPED)
        printf("模拟错误：此处内存不可执行\n");
    else if (p != UC_ERR_OK)
        printf("模拟错误：(未处理)%s\n", uc_strerror(p));
    dumpCpuInfo();
#ifdef GDB_SERVER_SUPPORT
    send_gdb_response(&clients[0], "S01");
#endif
    s_moral_emu_halted = 1;
}

#if !MORAL_EMU_DEDICATED_THREAD
static u32 s_moral_emu_pc;
static u32 s_moral_emu_cpsr;

static void moral_emu_prepare(void *startAddr)
{
    u32 pc = (u32)(uintptr_t)startAddr;

    s_moral_emu_halted = 0;
#ifdef GDB_SERVER_SUPPORT
    gdbTarget.running = 1;
    gdbTarget.breakpoints[gdbTarget.num_breakpoints++] = pc;
    readAllCpuRegFunc = ReadRegsToGdb;
    gdb_readMemFunc = readMemoryToGdb;
#endif
    uc_reg_read(MTK, UC_ARM_REG_CPSR, &s_moral_emu_cpsr);
    s_moral_emu_pc = pc;
}

static void moral_emu_run_slice(void)
{
    uc_err p;
    int pending_before;
    uint64_t timeout_us;
    uint64_t instr_count;
    uint64_t start_pc;

    if (MTK == NULL || s_moral_emu_halted)
        return;

    pending_before = moral_vm_has_pending_events();
    timeout_us = pending_before ? 4000u : 100000u;
    instr_count = pending_before ? 50000u : 400000u;
    start_pc = (uint64_t)(s_moral_emu_pc & ~1u);
    if (s_moral_emu_cpsr & 0x20)
        start_pc |= 1;

    p = uc_emu_start(MTK, start_pc, (uint64_t)-1, timeout_us, instr_count);
    uc_reg_read(MTK, UC_ARM_REG_PC, &s_moral_emu_pc);
    uc_reg_read(MTK, UC_ARM_REG_CPSR, &s_moral_emu_cpsr);

    if (p != UC_ERR_OK)
    {
        moral_emu_on_stop(p);
        return;
    }
    if (pending_before || moral_vm_has_pending_events())
    {
        handleVmEvent_EMU((uint64_t)s_moral_emu_pc);
        uc_reg_read(MTK, UC_ARM_REG_PC, &s_moral_emu_pc);
        uc_reg_read(MTK, UC_ARM_REG_CPSR, &s_moral_emu_cpsr);
    }
}
#else
static void *moral_pthread_run_arm(void *arg)
{
    RunArmProgram(arg);
    return NULL;
}

void RunArmProgram(void *startAddr)
{
    u32 pc = (u32)(uintptr_t)startAddr;
    u32 cpsr = 0;
    uc_err p = UC_ERR_OK;

#ifdef GDB_SERVER_SUPPORT
    gdbTarget.running = 1;
    gdbTarget.breakpoints[gdbTarget.num_breakpoints++] = pc;
    readAllCpuRegFunc = ReadRegsToGdb;
    gdb_readMemFunc = readMemoryToGdb;
#endif

    s_moral_emu_halted = 0;
    uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr);

    for (;;)
    {
        if (moral_emu_thread_stop)
            break;
#ifdef GDB_SERVER_SUPPORT
        if (gdbTarget.running == 0)
        {
            usleep(1000);
            continue;
        }
#endif
        if (s_moral_emu_halted)
            break;

        {
            uint64_t start_pc = (uint64_t)(pc & ~1u);
            if (cpsr & 0x20)
                start_pc |= 1;

#if MORAL_EMU_UNLIMITED_SLICE
            p = uc_emu_start(MTK, start_pc, (uint64_t)-1, 0, 0);
#else
            {
                int pending_before = moral_vm_has_pending_events();
                uint64_t timeout_us = pending_before ? 4000u : 100000u;
                uint64_t instr_count = pending_before ? 50000u : 400000u;

                p = uc_emu_start(MTK, start_pc, (uint64_t)-1, timeout_us, (size_t)instr_count);
            }
#endif
        }
        uc_reg_read(MTK, UC_ARM_REG_PC, &pc);
        uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr);

        if (p != UC_ERR_OK)
        {
            moral_emu_on_stop(p);
            break;
        }
        de_emulator_service_guest_fb_pull();
        if (moral_vm_has_pending_events())
        {
            handleVmEvent_EMU((uint64_t)pc);
            uc_reg_read(MTK, UC_ARM_REG_PC, &pc);
            uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr);
        }
    }
}
#endif /* MORAL_EMU_DEDICATED_THREAD */

/* 原 MainUpdateTask 每轮一次（去掉独立线程与 usleep） */
static void moral_mainupdate_tick_once(void)
{
    uint64_t elapsed_ms;

    currentTime = moral_get_ticks_ms();

#ifdef GDB_SERVER_SUPPORT
    if (gdbTarget.running == 0)
        return;
#endif
    lcdTaskMain();
    RtcTaskMain();
    GptTaskMain();
    SimTaskMain();

    if (last_hal_timer_tick_time == 0)
        last_hal_timer_tick_time = currentTime;
    elapsed_ms = currentTime - last_hal_timer_tick_time;
    if (elapsed_ms > 100)
        elapsed_ms = 100;
    if (elapsed_ms > 0)
    {
        last_hal_timer_tick_time = currentTime;
        halTimerCount += (u32)elapsed_ms;

        if (halTimerOutLength > 0)
        {
            uint64_t next_tick = (uint64_t)halTimerCnt + elapsed_ms;
            if (next_tick >= (uint64_t)halTimerOutLength)
            {
                halTimerCnt = (u32)(next_tick % (uint64_t)halTimerOutLength);
                if (halTimerIntStatus == 1)
                    timer_irq_pending = 1;
            }
            else
                halTimerCnt = (u32)next_tick;
        }
        else
            halTimerCnt += (u32)elapsed_ms;
    }
    if (currentTime > lastFlashTime)
    {
        lastFlashTime = currentTime + 300;
        fflush(stdout);
    }

    moral_perf_tick();

#if MORAL_EMU_DEDICATED_THREAD && MORAL_EMU_UNLIMITED_SLICE
    if (timer_irq_pending && MTK != NULL)
        uc_emu_stop(MTK);
#endif
}

void loop()
{
    SDL_Event ev;
    bool isLoop = true;

#if !MORAL_EMU_DEDICATED_THREAD
    moral_emu_prepare((void *)(uintptr_t)ROM_ADDRESS);
#endif

    while (isLoop)
    {
        while (SDL_PollEvent(&ev))
        {
            if (ev.type == SDL_QUIT)
            {
                isLoop = false;
                break;
            }
            switch (ev.type)
            {
            case SDL_KEYDOWN:
                if (isKeyDown == SDLK_UNKNOWN)
                {
                    isKeyDown = ev.key.keysym.sym;
                    keyEvent(MR_KEY_PRESS, ev.key.keysym.sym);
                }
                break;
            case SDL_KEYUP:
                if (isKeyDown == ev.key.keysym.sym)
                {
                    isKeyDown = SDLK_UNKNOWN;
                    keyEvent(MR_KEY_RELEASE, ev.key.keysym.sym);
                }
                break;
            case SDL_MOUSEMOTION:
                if (isMouseDown)
                {
                    mouseEvent(MR_MOUSE_MOVE, ev.motion.x, ev.motion.y);
                }
                break;
            case SDL_MOUSEBUTTONDOWN:
                isMouseDown = true;
                mouseEvent(MR_MOUSE_DOWN, ev.button.x, ev.button.y);
                break;
            case SDL_MOUSEBUTTONUP:
                isMouseDown = false;
                mouseEvent(MR_MOUSE_UP, ev.button.x, ev.button.y);
                break;
            }
        }
        /* 先外设/DE 把像素写入 SDL surface，再 present，减少一帧延迟 */
        moral_mainupdate_tick_once();

        if (Lcd_Need_Update)
            renderGdiBufferToWindow();

#if !MORAL_EMU_DEDICATED_THREAD
        if (!s_moral_emu_halted)
        {
#ifdef GDB_SERVER_SUPPORT
            if (gdbTarget.running != 0)
#endif
            {
                uint64_t budget_end = moral_get_ticks_ms() + (uint64_t)MORAL_EMU_MS_PER_MAIN_ITER;
                while (moral_get_ticks_ms() < budget_end && !s_moral_emu_halted)
                {
#ifdef GDB_SERVER_SUPPORT
                    if (gdbTarget.running == 0)
                        break;
#endif
                    moral_emu_run_slice();
                }
            }
        }
#endif

        SDL_Delay(1);
    }

#if MORAL_EMU_DEDICATED_THREAD
    moral_emu_thread_stop = 1;
    if (MTK != NULL)
        uc_emu_stop(MTK);
    {
        void *thr_ret;
        pthread_join(emu_thread, &thr_ret);
    }
#endif

    if (SD_File_Handle != NULL)
        fclose(SD_File_Handle);
    SD_File_Handle = NULL;
    g_sd_img_max_bytes = 0;
}

/**
 * 读取文件
 * 读取完成后需要释放
 */
u8 *readFile(const char *filename, u32 *size)
{
    FILE *file;
    u8 *tmp;
    long file_size;
    u8 flag;
    // 打开文件 a.txt
    file = fopen(filename, "rb");
    if (file == NULL)
    {
        printf("Failed to open file:%s\n", filename);
        return NULL;
    }

    // 移动文件指针到文件末尾，获取文件大小
    fseek(file, 0, SEEK_END);
    file_size = ftell(file);
    *size = file_size;
    rewind(file);
    // 为 tmp 分配内存
    tmp = (u8 *)SDL_malloc(file_size * sizeof(u8));
    if (tmp == NULL)
    {
        printf("Failed to allocate memory");
        fclose(file);
        return NULL;
    }

    // 读取文件内容到 tmp 中
    size_t result = fread(tmp, 1, file_size, file);
    if (result != file_size)
    {
        printf("Failed to read file");
        SDL_free(tmp);
        fclose(file);
        return NULL;
    }
    fclose(file);
    return tmp;
}
int writeFile(const char *filename, void *buff, u32 size)
{
    FILE *file;
    u8 *tmp;
    u8 flag;
    // 打开文件 a.txt
    file = fopen(filename, "w");
    if (file == NULL)
    {
        fclose(file);
        return 0;
    }
    // 移动文件指针到文件末尾，获取文件大小
    fseek(file, 0, SEEK_SET);
    // 读取文件内容到 tmp 中
    size_t result = fwrite(buff, 1, size, file);
    if (result != size)
    {
        printf("Failed to write file\n");
    }
    fclose(file);
    return result;
}

static int sd_boot_sector_is_sane(const u8 *sec, unsigned long long file_bytes);
static int sd_image_candidate_is_usable(FILE *fp, const char *path);

static FILE *open_sd_image_with_fallback(const char **opened_path)
{
    static char resolved_path[512];
    static const char *relative_candidates[] = {
        "Rom\\fat32.img",
        "Rom\\fat32-bk.img",
        "Rom\\FatImage.fat",
    };
    static const char *cwd_candidates[] = {
        SD_CARD_IMG_PATH,
        "Rom\\fat32-bk.img",
        "Rom\\FatImage.fat",
        "bin\\Rom\\fat32.img",
        "bin\\Rom\\fat32-bk.img",
        "bin\\Rom\\FatImage.fat",
    };
    char *base_path;
    u32 i;

    if (opened_path != NULL)
        *opened_path = NULL;

    base_path = SDL_GetBasePath();
    if (base_path != NULL)
    {
        for (i = 0; i < (sizeof(relative_candidates) / sizeof(relative_candidates[0])); i++)
        {
            FILE *fp;
            snprintf(resolved_path, sizeof(resolved_path), "%s%s", base_path, relative_candidates[i]);
            fp = fopen(resolved_path, "r+b");
            if (fp == NULL)
                fp = fopen(resolved_path, "rb");
            if (fp != NULL)
            {
                if (!sd_image_candidate_is_usable(fp, resolved_path))
                {
                    fclose(fp);
                    continue;
                }
                if (opened_path != NULL)
                    *opened_path = resolved_path;
                SDL_free(base_path);
                return fp;
            }
        }
        SDL_free(base_path);
    }

    for (i = 0; i < (sizeof(cwd_candidates) / sizeof(cwd_candidates[0])); i++)
    {
        FILE *fp = fopen(cwd_candidates[i], "r+b");
        if (fp != NULL)
        {
            if (!sd_image_candidate_is_usable(fp, cwd_candidates[i]))
            {
                fclose(fp);
                continue;
            }
            if (opened_path != NULL)
                *opened_path = cwd_candidates[i];
            return fp;
        }
    }

    for (i = 0; i < (sizeof(cwd_candidates) / sizeof(cwd_candidates[0])); i++)
    {
        FILE *fp = fopen(cwd_candidates[i], "rb");
        if (fp != NULL)
        {
            if (!sd_image_candidate_is_usable(fp, cwd_candidates[i]))
            {
                fclose(fp);
                continue;
            }
            if (opened_path != NULL)
                *opened_path = cwd_candidates[i];
            return fp;
        }
    }

    return NULL;
}
/* SD 镜像可能 >2GB 或扇区偏移*512 超过 32 位；Win 上用 _fseeki64 */
static int sd_img_fseek_set(FILE *fp, unsigned long long pos)
{
#if defined(_WIN32)
    return _fseeki64(fp, (__int64)pos, SEEK_SET);
#else
    if (pos > (unsigned long long)LONG_MAX)
        return -1;
    return fseek(fp, (long)pos, SEEK_SET);
#endif
}

static u16 sd_ld16_img(const u8 *p)
{
    return (u16)p[0] | ((u16)p[1] << 8);
}

static u32 sd_ld32_img(const u8 *p)
{
    return (u32)p[0] | ((u32)p[1] << 8) | ((u32)p[2] << 16) | ((u32)p[3] << 24);
}

/* 对比 MBR 分区范围 / FAT BPB 与镜像文件大小；超出则必然出现写被拒、FAT 异常、间歇无法开机 */
static void sd_diagnose_image_geometry(FILE *fp, unsigned long long file_bytes)
{
#if defined(_WIN32)
    __int64 save = _ftelli64(fp);
#else
    long save = ftell(fp);
#endif
    u8 s0[512];
    unsigned long long nsec = file_bytes / 512ULL;

    if (file_bytes < 512ULL)
        goto restore;
    if (sd_img_fseek_set(fp, 0ULL) != 0)
        goto restore;
    if (fread(s0, 1, 512, fp) != 512)
        goto restore;
    if (s0[0x1FE] != 0x55 || s0[0x1FF] != 0xAA)
        goto restore;

    {
        u32 lba = sd_ld32_img(s0 + 0x1C6);
        u32 cnt = sd_ld32_img(s0 + 0x1CA);
        if (cnt != 0u)
        {
            unsigned long long part_end = (unsigned long long)lba + (unsigned long long)cnt;
            if (part_end > nsec || (unsigned long long)cnt > nsec)
            {
                printf("[SD] !!! MBR 第一分区超出镜像文件: LBA首=%u 扇区数=%u (末LBA=%llu) 镜像总扇区=%llu\n",
                       lba, cnt, part_end - 1ULL, nsec);
                printf("[SD]     → 用 DiskGenius 把分区容量调到不超过镜像大小再格式化，否则必现写被拒/系统不稳\n");
            }
            if (cnt >= 1u && (unsigned long long)lba < nsec &&
                (unsigned long long)lba + (unsigned long long)cnt <= nsec)
            {
                u8 vbr[512];
                unsigned long long off = (unsigned long long)lba * 512ULL;
                if (sd_img_fseek_set(fp, off) == 0 && fread(vbr, 1, 512, fp) == 512 &&
                    vbr[0x1FE] == 0x55 && vbr[0x1FF] == 0xAA)
                {
                    u16 bps = sd_ld16_img(vbr + 0x0B);
                    u16 ts16 = sd_ld16_img(vbr + 0x13);
                    u32 ts32 = sd_ld32_img(vbr + 0x20);
                    u32 vol_sec = (ts16 != 0) ? (u32)ts16 : ts32;
                    if (bps != 0u && vol_sec != 0u)
                    {
                        unsigned long long vb = (unsigned long long)vol_sec * (unsigned long long)bps;
                        if (vb > file_bytes)
                        {
                            printf("[SD] !!! FAT 引导扇区声明卷大小约 %llu 字节 > 镜像 %llu 字节\n", vb,
                                   file_bytes);
                            printf("[SD]     → 请重做分区/格式化使总扇区×512 不超过镜像文件长度\n");
                        }
                    }
                }
            }
        }
    }

restore:
#if defined(_WIN32)
    if (save >= 0)
        (void)_fseeki64(fp, save, SEEK_SET);
#else
    if (save >= 0)
        (void)fseek(fp, save, SEEK_SET);
#endif
}

static int sd_img_get_file_size(FILE *fp, unsigned long long *out_sz)
{
#if defined(_WIN32)
    __int64 save = _ftelli64(fp);
    if (save < 0)
        return -1;
    if (_fseeki64(fp, 0, SEEK_END) != 0)
        return -1;
    __int64 sz = _ftelli64(fp);
    if (sz < 0)
        return -1;
    if (_fseeki64(fp, save, SEEK_SET) != 0)
        return -1;
    *out_sz = (unsigned long long)sz;
    return 0;
#else
    long save = ftell(fp);
    if (save < 0)
        return -1;
    if (fseek(fp, 0, SEEK_END) != 0)
        return -1;
    long sz = ftell(fp);
    if (sz < 0)
        return -1;
    if (fseek(fp, save, SEEK_SET) != 0)
        return -1;
    *out_sz = (unsigned long long)sz;
    return 0;
#endif
}

static int sd_is_valid_sector_size(u32 bps)
{
    return bps == 512u || bps == 1024u || bps == 2048u || bps == 4096u;
}

static int sd_is_valid_cluster_size(u32 spc)
{
    return spc != 0u && (spc & (spc - 1u)) == 0u;
}

static int sd_boot_sector_is_sane(const u8 *sec, unsigned long long file_bytes)
{
    u32 bps;
    u32 spc;
    u32 reserved;
    u32 fats;
    u32 ts16;
    u32 ts32;
    unsigned long long vol_sec;
    unsigned long long vol_bytes;

    if (sec[0x1FE] != 0x55 || sec[0x1FF] != 0xAA)
        return 0;

    bps = sd_ld16_img(sec + 0x0B);
    spc = sec[0x0D];
    reserved = sd_ld16_img(sec + 0x0E);
    fats = sec[0x10];
    ts16 = sd_ld16_img(sec + 0x13);
    ts32 = sd_ld32_img(sec + 0x20);

    if (!sd_is_valid_sector_size(bps))
        return 0;
    if (!sd_is_valid_cluster_size(spc))
        return 0;
    if (reserved == 0u)
        return 0;
    if (fats == 0u || fats > 4u)
        return 0;

    vol_sec = (ts16 != 0u) ? (unsigned long long)ts16 : (unsigned long long)ts32;
    if (vol_sec == 0u)
        return 0;

    vol_bytes = vol_sec * (unsigned long long)bps;
    if (vol_bytes > file_bytes)
        return 0;

    return 1;
}

static int sd_image_candidate_is_usable(FILE *fp, const char *path)
{
#if defined(_WIN32)
    __int64 save = _ftelli64(fp);
#else
    long save = ftell(fp);
#endif
    unsigned long long file_bytes = 0;
    unsigned long long nsec;
    u8 s0[512];
    u32 lba;
    u32 cnt;

    if (save < 0)
        return 1;
    if (sd_img_get_file_size(fp, &file_bytes) != 0 || file_bytes < 512ULL)
        return 1;
    nsec = file_bytes / 512ULL;
    if (sd_img_fseek_set(fp, 0ULL) != 0)
        goto restore_and_accept;
    if (fread(s0, 1, sizeof(s0), fp) != sizeof(s0))
        goto restore_and_accept;

    lba = sd_ld32_img(s0 + 0x1C6);
    cnt = sd_ld32_img(s0 + 0x1CA);

    if (cnt != 0u && (unsigned long long)lba < nsec &&
        (unsigned long long)lba + (unsigned long long)cnt <= nsec)
    {
        u8 vbr[512];
        if (sd_img_fseek_set(fp, (unsigned long long)lba * 512ULL) == 0 &&
            fread(vbr, 1, sizeof(vbr), fp) == sizeof(vbr) &&
            sd_boot_sector_is_sane(vbr, file_bytes))
            goto restore_and_accept;

        printf("[SD] 跳过候选镜像: %s\n", path);
        printf("[SD]     原因: 分区表存在，但引导扇区/BPB 几何非法或超出镜像长度\n");
        goto restore_and_reject;
    }

    if (sd_boot_sector_is_sane(s0, file_bytes))
        goto restore_and_accept;

restore_and_accept:
#if defined(_WIN32)
    (void)_fseeki64(fp, save, SEEK_SET);
#else
    (void)fseek(fp, save, SEEK_SET);
#endif
    return 1;

restore_and_reject:
#if defined(_WIN32)
    (void)_fseeki64(fp, save, SEEK_SET);
#else
    (void)fseek(fp, save, SEEK_SET);
#endif
    return 0;
}

/*
 * SDHC CSD V2.0：用户容量 = (C_SIZE+1) * 512 KiB = (C_SIZE+1) * 1024 个 512B 扇区。
 * 按镜像文件实际长度换算，并向下对齐到 1024 扇区（SDHC 要求）。
 */
unsigned long sd_get_reported_sectors_for_csd(void)
{
    unsigned long long fsize = 0;
    const unsigned long fallback = 1048576UL; /* 默认 512MB */
    if (g_sd_img_max_bytes != 0ULL)
        fsize = g_sd_img_max_bytes;
    else if (SD_File_Handle == NULL)
        return fallback;
    else if (sd_img_get_file_size(SD_File_Handle, &fsize) != 0)
        return fallback;
    if (fsize < 512ULL)
        return 1024UL;
    {
        unsigned long long sec = fsize / 512ULL;
        unsigned long long aligned = (sec / 1024ULL) * 1024ULL;
        if (aligned < 1024ULL)
            aligned = 1024ULL;
        /* C_SIZE 为 22 位：最大 (0x3FFFFF+1)*1024 扇区 */
        {
            unsigned long long max_sec = 0x400000ULL * 1024ULL;
            if (aligned > max_sec)
                aligned = max_sec;
        }
        return (unsigned long)aligned;
    }
}

void sd_fill_csd_v2_regs(u32 csd_regs[9])
{
    unsigned long sec = sd_get_reported_sectors_for_csd();
    unsigned long c_size = sec / 1024UL - 1UL;
    if (c_size > 0x3FFFFFu)
        c_size = 0x3FFFFFu;
    {
        u32 be = (u32)(c_size & 0x3FFFFFu);
        u8 b7 = (u8)((be >> 24) & 0xFF);
        u8 b8 = (u8)((be >> 16) & 0xFF);
        u8 b9 = (u8)((be >> 8) & 0xFF);
        u8 b10 = (u8)(be & 0xFF);
        my_memset(csd_regs, 0, (int)(sizeof(u32) * 9));
        csd_regs[0] = 0x00u | (0x40u << 8);
        csd_regs[1] = 0x0Eu | (0x00u << 8);
        csd_regs[2] = 0x5Au | (0x5Bu << 8);
        csd_regs[3] = 0x59u | ((u32)b7 << 8);
        csd_regs[4] = (u32)b8 | ((u32)b9 << 8);
        csd_regs[5] = (u32)b10 | (0x7Fu << 8);
        csd_regs[6] = 0x80u | (0x0Au << 8);
        csd_regs[7] = 0x40u | (0x00u << 8);
        csd_regs[8] = 0x01u;
    }
}

u8 *readSDFile(unsigned long long startPos, u32 size)
{
    u8 *tmp;
    u8 flag;
    static u32 sd_read_oob_warns;
    if (SD_File_Handle == NULL)
    {
        return NULL;
    }
    tmp = (u8 *)SDL_malloc(size);
    if (tmp == NULL)
    {
        printf("申请文件内存失败");
        return NULL;
    }
    my_memset(tmp, 0, size);
    {
        unsigned long long fsize = 0;
        if (g_sd_img_max_bytes != 0ULL)
            fsize = g_sd_img_max_bytes;
        else if (sd_img_get_file_size(SD_File_Handle, &fsize) != 0)
        {
            printf("读取SD卡：无法取得镜像大小\n");
            SDL_free(tmp);
            return NULL;
        }
        if (startPos >= fsize)
        {
            if (sd_read_oob_warns < 8u)
            {
                sd_read_oob_warns++;
                printf("[SD] 读越界(视为全0): offset=%llu size=%u 镜像=%llu 字节\n",
                       (unsigned long long)startPos, (unsigned)size, fsize);
            }
            return tmp;
        }
        {
            unsigned long long avail = fsize - startPos;
            size_t to_read = (avail >= (unsigned long long)size) ? (size_t)size : (size_t)avail;
            if (to_read == 0)
                return tmp;
            if (sd_img_fseek_set(SD_File_Handle, startPos) != 0)
            {
                printf("移动文件指针失败");
                SDL_free(tmp);
                return NULL;
            }
            {
                size_t result = fread(tmp, 1, to_read, SD_File_Handle);
                if (result != to_read)
                {
                    printf("读取SD卡文件失败 %zu <> %zu\n", result, to_read);
                    SDL_free(tmp);
                    return NULL;
                }
            }
            if (to_read < (size_t)size && sd_read_oob_warns < 8u)
            {
                sd_read_oob_warns++;
                printf("[SD] 读部分越界: offset=%llu 请求=%u 实际可读=%zu 余下填0 (镜像=%llu)\n",
                       (unsigned long long)startPos, (unsigned)size, to_read, fsize);
            }
        }
    }
    Perf_SdReadOps++;
    return tmp;
}

bool writeSDFile(u8 *Buffer, unsigned long long startPos, u32 size)
{
    u8 flag;
    unsigned long long limit_bytes = g_sd_img_max_bytes;
    u32 write_size = size;
    if (SD_File_Handle == NULL)
    {
        return false;
    }
    if (limit_bytes == 0ULL)
        limit_bytes = 1ULL * 1024 * 1024 * 1024;
    {
        if (startPos >= limit_bytes)
        {
            static u32 sd_bigwr_log;
            if (sd_bigwr_log < 8u)
            {
                sd_bigwr_log++;
                printf("[SD] 写入越界(跳过): offset=%llu len=%u limit=%llu\n",
                       (unsigned long long)startPos, (unsigned)size, limit_bytes);
            }
            return false;
        }
        if (startPos + (unsigned long long)write_size > limit_bytes)
        {
            static u32 sd_partial_log;
            write_size = (u32)(limit_bytes - startPos);
            if (sd_partial_log < 8u)
            {
                sd_partial_log++;
                printf("[SD] 写入部分越界: offset=%llu 请求=%u 实际=%u limit=%llu\n",
                       (unsigned long long)startPos, (unsigned)size, (unsigned)write_size, limit_bytes);
            }
        }
    }
    if (write_size == 0)
        return false;
    if (sd_img_fseek_set(SD_File_Handle, startPos) != 0)
    {
        printf("移动文件指针失败\n");
        return false;
    }
    size_t result = fwrite(Buffer, 1, write_size, SD_File_Handle);
    if (result != write_size)
    {
        printf("写入文件失败\n");
        return false;
    }
    {
        static u32 sd_flush_counter;
        sd_flush_counter++;
        if (MORAL_SD_WRITE_FLUSH_EVERY <= 1 || (sd_flush_counter % MORAL_SD_WRITE_FLUSH_EVERY) == 0)
            fflush(SD_File_Handle);
    }
    {
        static u32 sd_wr_ok_log;
        if (MORAL_LOG_SD_IO && (sd_wr_ok_log < 30u || (sd_wr_ok_log % 100) == 0))
            printf("[SD-WR] ok offset=%llu len=%u (#%u)\n",
                   (unsigned long long)startPos, (unsigned)write_size, sd_wr_ok_log);
        sd_wr_ok_log++;
    }
    Perf_SdWriteOps++;
    return true;
}

static void moral_perf_tick(void)
{
#if MORAL_PERF_STATS_INTERVAL_MS > 0
    static uint64_t last_perf_tick = 0;
    static u32 last_enqueue = 0;
    static u32 last_dequeue = 0;
    static u32 last_drop = 0;
    static u32 last_coalesce = 0;
    static u32 last_irq = 0;
    static u32 last_lcd = 0;
    static u32 last_sd_read = 0;
    static u32 last_sd_write = 0;
    uint64_t interval = (uint64_t)MORAL_PERF_STATS_INTERVAL_MS;

    if (last_perf_tick == 0)
    {
        last_perf_tick = currentTime;
        return;
    }
    if ((currentTime - last_perf_tick) < interval)
        return;

    printf("[perf] queue=%u high=%u enq=+%u deq=+%u drop=+%u coal=+%u irq=+%u lcd=+%u sd_r=+%u sd_w=+%u\n",
           VmEventCount,
           Perf_EventQueueHighWater,
           Perf_EventEnqueueCount - last_enqueue,
           Perf_EventDequeueCount - last_dequeue,
           Perf_EventDropCount - last_drop,
           Perf_EventCoalesceCount - last_coalesce,
           Perf_IrqInjectCount - last_irq,
           Perf_LcdRefreshCount - last_lcd,
           Perf_SdReadOps - last_sd_read,
           Perf_SdWriteOps - last_sd_write);

    last_perf_tick = currentTime;
    last_enqueue = Perf_EventEnqueueCount;
    last_dequeue = Perf_EventDequeueCount;
    last_drop = Perf_EventDropCount;
    last_coalesce = Perf_EventCoalesceCount;
    last_irq = Perf_IrqInjectCount;
    last_lcd = Perf_LcdRefreshCount;
    last_sd_read = Perf_SdReadOps;
    last_sd_write = Perf_SdWriteOps;
#endif
}

/* 线性固件：[0, GUEST_LOW_IMAGE_MAP_SIZE)→低映射；超出写入 GUEST_IDA_XRAM_BASE（与原先连续 16MB 布局一致） */
static void moral_uc_write_low_and_xram(const u8 *buf, size_t total)
{
    if (MTK == NULL || buf == NULL || total == 0)
        return;
    const size_t low_max = (size_t)GUEST_LOW_IMAGE_MAP_SIZE;
    size_t n0 = total < low_max ? total : low_max;
    uc_mem_write(MTK, ROM_ADDRESS, buf, n0);
    if (total > low_max)
    {
        size_t rest = total - low_max;
        if (rest > (size_t)GUEST_IDA_XRAM_SIZE)
            rest = (size_t)GUEST_IDA_XRAM_SIZE;
        uc_err er = uc_mem_write(MTK, GUEST_IDA_XRAM_BASE, buf + low_max, rest);
        if (er != UC_ERR_OK)
            printf("[mem] spill -> XRAM @0x%X err %u (len=%u)\n", GUEST_IDA_XRAM_BASE, er, (unsigned)rest);
        else if (total - low_max > rest)
            printf("[mem] WARN: %u bytes beyond low+XRAM not loaded\n", (unsigned)(total - low_max - rest));
    }
}

/* 打开 ARM 引擎、映射 ROM/RAM/IRAM/XRAM/外设窗口，注册 CODE hook 与 MMIO 读写回调 */
void initMtkSimalator()
{
    uc_err err;
    uc_hook trace[20];

    InitVmEvent();
    InitSimCard();
    InitLcd();

    err = uc_open(UC_ARCH_ARM, UC_MODE_ARM, &MTK);
    if (err)
    {
        printf("Failed on uc_open() with error returned: %u (%s)\n", err, uc_strerror(err));
        return NULL;
    }
    uc_ctl_set_cpu_model(MTK, UC_CPU_ARM_CORTEX_A9);

    ROM_MEMPOOL = SDL_malloc(size_16mb);

    ROM2_MEMPOOL = SDL_malloc((size_t)GUEST_IRAM_SECTION0_MAP_SIZE);

    /* 勿映射满 16MB：须留出 0x00D00000 给 IDA XRAM，否则与触摸/MMI 全局量冲突 → err 11 */
    err = uc_mem_map_ptr(MTK, ROM_ADDRESS, GUEST_LOW_IMAGE_MAP_SIZE, UC_PROT_ALL, ROM_MEMPOOL);
    /* IDA：IRAM_SECTION0 / IRAM_SECTION / IRAM_RF_SECTION，约 0x08000000..0x08005BAC */
    err = uc_mem_map_ptr(MTK, GUEST_IRAM_SECTION0_BASE, (size_t)GUEST_IRAM_SECTION0_MAP_SIZE,
                         UC_PROT_ALL, ROM2_MEMPOOL);
    if (err != UC_ERR_OK)
        printf("[mem] map IRAM_SECTION0 @0x%X err %u (%s)\n", GUEST_IRAM_SECTION0_BASE, err, uc_strerror(err));
    else
        printf("[mem] Mapped IDA IRAM_SECTION0..RF 0x%08X size=0x%X\n", GUEST_IRAM_SECTION0_BASE,
               (unsigned)GUEST_IRAM_SECTION0_MAP_SIZE);
    err = uc_mem_map_ptr(MTK, 0x7000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    err = uc_mem_map_ptr(MTK, 0x1000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    if (err)
    {
        printf("Failed mem  Rom map: %u (%s)\n", err, uc_strerror(err));
        return NULL;
    }
    err = uc_mem_map_ptr(MTK, 0x0f000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    err = uc_mem_map_ptr(MTK, 0x0e000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    err = uc_mem_map_ptr(MTK, 0x0d000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    s_ida_xram_host = NULL;
    {
        void *ida_xram = SDL_malloc((size_t)GUEST_IDA_XRAM_SIZE);
        if (!ida_xram)
            printf("[mem] SDL_malloc IDA XRAM failed\n");
        else
        {
            err = uc_mem_map_ptr(MTK, GUEST_IDA_XRAM_BASE, (size_t)GUEST_IDA_XRAM_SIZE, UC_PROT_ALL, ida_xram);
            if (err != UC_ERR_OK)
                printf("[mem] map 0x%X XRAM err %u (%s)\n", GUEST_IDA_XRAM_BASE, err, uc_strerror(err));
            else
            {
                s_ida_xram_host = (u8 *)ida_xram;
                printf("[mem] Mapped IDA XRAM 0x%X size=0x%X (touch ZI / MMI globals)\n",
                       GUEST_IDA_XRAM_BASE, (unsigned)GUEST_IDA_XRAM_SIZE);
            }
        }
    }

    /* MT6252 外部 SRAM/PSRAM 可能在 0x04000000 bank */
    err = uc_mem_map_ptr(MTK, 0x04000000, size_16mb * 4, UC_PROT_ALL, SDL_calloc(1, size_16mb * 4));
    err = uc_mem_map_ptr(MTK, 0x10000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    err = uc_mem_map_ptr(MTK, 0x60000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    err = uc_mem_map_ptr(MTK, 0x50000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    /* IDA：IRAM_SECTION2，约 0x1C000000..0x1C010C9C；不映射会 UC_ERR_MAP */
    {
        void *iram2 = SDL_malloc((size_t)GUEST_IRAM_SECTION2_MAP_SIZE);
        if (!iram2)
            printf("[mem] SDL_malloc IRAM_SECTION2 failed\n");
        else
        {
            err = uc_mem_map_ptr(MTK, GUEST_IRAM_SECTION2_BASE, (size_t)GUEST_IRAM_SECTION2_MAP_SIZE,
                                 UC_PROT_ALL, iram2);
            if (err != UC_ERR_OK)
                printf("[mem] map IRAM_SECTION2 @0x%X err %u (%s)\n", GUEST_IRAM_SECTION2_BASE, err,
                       uc_strerror(err));
            else
                printf("[mem] Mapped IDA IRAM_SECTION2 0x%08X size=0x%X\n", GUEST_IRAM_SECTION2_BASE,
                       (unsigned)GUEST_IRAM_SECTION2_MAP_SIZE);
        }
    }

    /* KER/BACKTRACE 诊断路径会访问 0x2800xxxx，缺失映射会触发 UC_ERR_MAP */
    err = uc_mem_map_ptr(MTK, 0x28000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    err = uc_mem_map_ptr(MTK, 0x74000000, size_4mb, UC_PROT_ALL, SDL_malloc(size_4mb));
    err = uc_mem_map_ptr(MTK, 0x34000000, size_4mb, UC_PROT_ALL, SDL_malloc(size_4mb));
    err = uc_mem_map_ptr(MTK, 0x90000000, size_4mb, UC_PROT_ALL, SDL_malloc(size_4mb));
    if (err)
    {
        printf("Failed mem  Rom map: %u (%s)\n", err, uc_strerror(err));
        return NULL;
    }
    /* 0x81000000 外设池：RTC / GPT / AUX / IRQ / SDC 等 */
    {
        void *mtk_peri = SDL_malloc(0x02000000);
        if (!mtk_peri)
        {
            printf("SDL_malloc MTK peripheral pool failed\n");
            return NULL;
        }
        err = uc_mem_map_ptr(MTK, 0x81000000, 0x02000000, UC_PROT_ALL, mtk_peri);
        if (err != UC_ERR_OK)
        {
            printf("Failed map 0x81000000 peripherals: %u (%s)\n", err, uc_strerror(err));
            return NULL;
        }
        printf("[mem] Mapped 0x81000000..0x83000000 (RTC / AUX / GPT / legacy IRQ regs)\n");
        InitSimCardRegs();
    }
    /*
     * 代码钩子地址表：每项 [起始,结束] 含两端。Thumb 下 PC 可能为偶地址或 +1，
     * 故像 0xEE9DC 这类 16 位指令常登记两字节区间。
     */
    {
        /* 与 hook_ranges 行数一致并留余量；曾用 32 与表项持平导致无法再挂 sysIsCdcFastInitMode */
        static uc_hook code_hooks[48];
        int hi = 0;
        static const u32 hook_ranges[][2] = {
            {0xA144, 0xA145},         /* pin config */
            {0xC166, 0xC167},         /* DrvLcdSetDisplayRange: 捕获脏区 x,y,w,h */
            {0xCD44, 0xCD45},         /* LCD params */
            {0x1E574, 0x1E575},       /* HalDispTransAddr */
            {0x2AE1C, 0x2AE1D},       /* ker_assert */
            {0x2C55E, 0x2C55F},       /* DrvDMA2DIsHWBitBlt */
            {0x2C5DC, 0x2C5DD},       /* DrvDMA2DIsHWFillRect */
            {0x2CB28, 0x2CB29},       /* DrvDMA2DCmdFinish */
            {0x2D5ECA, 0x2D5ECB},     /* uart_print */
            {0x31B9C, 0x31B9D},       /* RTC seconds hook */
            {0x1A605C, 0x1A605D},     /* ker_assert_func */
            {0x1D4B96, 0x1D4B97},     /* nand_translate_DMA */
            {0x1ED480, 0x1ED481},     /* _RtkExceptionRoutine */
            {0x1FE99C, 0x1FE99D},     /* LOG_SD */
            {0x219712, 0x219713},     /* MdlTouchScreenStatusReport MsSend return */
            {0x219848, 0x219849},     /* _MdlTouchscreenGetYCoordination */
            {0x219878, 0x219879},     /* _MdlTouchscreenGetXCoordination */
            {0x219DF0, 0x219DF1},     /* _MdlTouchscreenRepeatADCProcess MsSend return */
            {0x1DEF62, 0x1DEF63},     /* HalUtilPHY2MIUAddr entry */
            {0x1DEF6C, 0x1DEF6D},     /* HalUtilPHY2MIUAddr clamp to 0 branch */
            {0x33370,  0x33371},      /* MDrvFCIEStorageR: capture DMA phys addr */
            {0x336C2,  0x336C3},      /* MDrvFCIEStorageW: capture DMA phys addr */
            {0x40ED0,  0x40ED1},      /* Drv_DoDataCompare: skip write verify */
            {0x30F34A, 0x30F34B},     /* dev_accGetLCDStatus */
            {0x32DFA4, 0x32DFA5},     /* _RtkAssertRoutine */
            {0x34D236, 0x34D237},     /* MsSend POP */
            {0x36ED44, 0x36ED45},     /* fatal error check */
            {0x3B5A00, 0x3B5A01},     /* KER_VTRACE */
            {0x3B5A52, 0x3B5A53},     /* ker_trace */
            {0x3B5BA4, 0x3B5BA5},     /* fatal error check */
            {0x3B5C54, 0x3B5C55},     /* KER error */
            {0x7C322C, 0x7C3238},     /* skip mrc instructions */
            {0xEE9DC, 0xEE9DD},       /* LDRH R1,[R1,#34] @ hwll1_ReadE2pParameters → UC_ERR_EXCEPTION */
        };
        for (u32 ri = 0; ri < sizeof(hook_ranges) / sizeof(hook_ranges[0]); ri++)
        {
            err = uc_hook_add(MTK, &code_hooks[hi++], UC_HOOK_CODE,
                              hookCodeCallBack, 0,
                              hook_ranges[ri][0], hook_ranges[ri][1]);
        }
        if (MORAL_HOOK_SYS_IS_CDC_PC != 0u)
        {
            u32 cdc_lo = MORAL_HOOK_SYS_IS_CDC_PC & ~1u;
            u32 cdc_hi = cdc_lo + MORAL_HOOK_SYS_IS_CDC_BYTES - 1u;
            if (hi >= (int)(sizeof(code_hooks) / sizeof(code_hooks[0])))
                printf("[hook] code_hooks[] full, skip sysIsCdcFastInitMode @0x%X\n", cdc_lo);
            else if (cdc_hi >= cdc_lo)
            {
                err = uc_hook_add(MTK, &code_hooks[hi++], UC_HOOK_CODE,
                                  hookCodeCallBack, 0, cdc_lo, cdc_hi);
                if (err != UC_ERR_OK)
                    printf("[hook] sysIsCdcFastInitMode CODE hook @0x%X err %u\n", cdc_lo, err);
                else
                    printf("[hook] sysIsCdcFastInitMode 已挂接 0x%X..0x%X（命中时打印 [sysIsCdcFastInitMode]）\n",
                           cdc_lo, cdc_hi);
            }
        }
        if (MORAL_HOOK_VSPRINTF_PC != 0u)
        {
            u32 vlo = MORAL_HOOK_VSPRINTF_PC & ~1u;
            u32 vhi = vlo + MORAL_HOOK_VSPRINTF_BYTES - 1u;
            if (hi >= (int)(sizeof(code_hooks) / sizeof(code_hooks[0])))
                printf("[hook] code_hooks[] full, skip vsprintf @0x%X\n", vlo);
            else if (vhi >= vlo)
            {
                err = uc_hook_add(MTK, &code_hooks[hi++], UC_HOOK_CODE,
                                  hookCodeCallBack, 0, vlo, vhi);
                if (err != UC_ERR_OK)
                    printf("[hook] vsprintf CODE hook @0x%X err %u\n", vlo, err);
                else
                    printf("[hook] vsprintf 已挂接 0x%X..0x%X（命中时打印 [vsprintf] 展开行）\n", vlo, vhi);
            }
        }
    }

    {
        static uc_hook mem_hooks[24];
        int mi = 0;
        static const u32 mem_ranges[][2] = {
            {0x34000000, 0x3400FFFF}, /* MTK 外设寄存器 (IRQ/Timer/UART/AUXADC) */
            {0x74000000, 0x74006FFF}, /* DE/DMA/FCIE/SPI */
            {0x81060000, 0x810600FF}, /* GPT */
            {0x810E0000, 0x810E00FF}, /* MSDC 寄存器 → handleMsdcReg */
            {0x81020200, 0x810202FF}, /* DMA MSDC 通道 (base+0x200)，勿含 0x810203xx 以免与 SIM DMA 冲突 */
            {0x82050000, 0x82050FFF}, /* AUX 触摸 */
            {0x90000000, 0x90000FFF}, /* LCD 控制器 */
            {0x00D00000, 0x00D0FFFF}, /* XRAM 全局量 (RF config) */
        };
        for (u32 ri = 0; ri < sizeof(mem_ranges) / sizeof(mem_ranges[0]); ri++)
        {
            uc_hook_add(MTK, &mem_hooks[mi++], UC_HOOK_MEM_READ,
                        hookRamCallBack, 0,
                        mem_ranges[ri][0], mem_ranges[ri][1]);
            uc_hook_add(MTK, &mem_hooks[mi++], UC_HOOK_MEM_WRITE,
                        hookRamCallBack, 1,
                        mem_ranges[ri][0], mem_ranges[ri][1]);
        }
    }

    err = uc_hook_add(MTK, &trace, UC_HOOK_MEM_READ_UNMAPPED, hookRamErrorBack, 2, 0, 0xFFFFFFFF);
    err = uc_hook_add(MTK, &trace, UC_HOOK_MEM_WRITE_UNMAPPED, hookRamErrorBack, 3, 0, 0xFFFFFFFF);
    err = uc_hook_add(MTK, &trace, UC_HOOK_MEM_FETCH_UNMAPPED, hookRamErrorBack, 4, 0, 0xFFFFFFFF);

    err = uc_hook_add(MTK, &trace, UC_HOOK_INSN_INVALID, hookInsnInvalid, 4, 0, 0xFFFFFFFF);

    if (err != UC_ERR_OK)
    {
        printf("add hook err %u (%s)\n", err, uc_strerror(err));
        return NULL;
    }
}

void dumpCpuInfo()
{
    u32 r0 = 0;
    u32 r1 = 0;
    u32 r2 = 0;
    u32 r3 = 0;
    u32 r4 = 0;
    u32 msp = 0;
    u32 pc = 0;
    u32 lr = 0;
    u32 cpsr = 0;
    uc_reg_read(MTK, UC_ARM_REG_PC, &pc);
    uc_reg_read(MTK, UC_ARM_REG_SP, &msp);
    uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr);
    uc_reg_read(MTK, UC_ARM_REG_LR, &lr);
    uc_reg_read(MTK, UC_ARM_REG_R0, &r0);
    uc_reg_read(MTK, UC_ARM_REG_R1, &r1);
    uc_reg_read(MTK, UC_ARM_REG_R2, &r2);
    uc_reg_read(MTK, UC_ARM_REG_R3, &r3);
    uc_reg_read(MTK, UC_ARM_REG_R4, &r4);
    printf("r0:%x r1:%x r2:%x r3:%x r4:%x r5:%x r6:%x r7:%x r8:%x r9:%x\n", r0, r1, r2, r3, r4);
    printf("msp:%x cpsr:%x(thumb:%x)(mode:%x) lr:%x pc:%x lastPc:%x irq_c(%x)\n", msp, cpsr, (cpsr & 0x20) > 0, cpsr & 0x1f, lr, pc, lastAddress, irq_nested_count);
    printf("------------\n");
}
void debugDumpCpu()
{
    u32 r0 = 0;
    u32 r1 = 0;
    u32 r2 = 0;
    u32 r3 = 0;
    u32 r4 = 0;
    u32 msp = 0;
    u32 pc = 0;
    u32 lr = 0;
    u32 cpsr = 0;
    uc_reg_read(MTK, UC_ARM_REG_R0, &r0);
    uc_reg_read(MTK, UC_ARM_REG_R1, &r1);
    uc_reg_read(MTK, UC_ARM_REG_R2, &r2);
    uc_reg_read(MTK, UC_ARM_REG_R3, &r3);
    uc_reg_read(MTK, UC_ARM_REG_R4, &r4);
    printf("r0:%08x r1:%08x r2:%08x r3:%08x r4:%08x\n", r0, r1, r2, r3, r4);
    uc_reg_read(MTK, UC_ARM_REG_R5, &r0);
    uc_reg_read(MTK, UC_ARM_REG_R6, &r1);
    uc_reg_read(MTK, UC_ARM_REG_R7, &r2);
    uc_reg_read(MTK, UC_ARM_REG_R8, &r3);
    uc_reg_read(MTK, UC_ARM_REG_R9, &r4);
    printf("r5:%08x r6:%08x r7:%08x r8:%08x r9:%08x\n", r0, r1, r2, r3, r4);
    uc_reg_read(MTK, UC_ARM_REG_SP, &msp);
    uc_reg_read(MTK, UC_ARM_REG_LR, &lr);
    uc_reg_read(MTK, UC_ARM_REG_PC, &pc);
    printf("msp:%08x lr:%08x pc:%08x\n", msp, lr, pc);
}

u8 *SimpleRamMatch(u8 *start, u8 *end, u8 *matchStart, int matchLen)
{
    u8 ii;
    while (start < end)
    {
        for (ii = 0; ii < matchLen; ii++)
        {
            if (*(start + ii) != *(matchStart + ii))
            {
                break;
            }
        }
        if (ii == matchLen)
            break;
        start++;
    }
    if (ii == matchLen)
        return start;
    else
        return NULL;
}

void FindKeypadData()
{
}

void FindRomMpuSettingAddr()
{
    u8 PMU_Setting_Magic[] = {'M', 'T', 'K', '_', 'C', 'M', 'B', '_', 'M', 'E', 'M', '_', 'L', 'S', 'T', 0};
    u32 matchCount = 0;
    u8 matchLen = sizeof(PMU_Setting_Magic) / sizeof(PMU_Setting_Magic[0]);

    printf("Find Match MPU Setting Addr...\n");
    u8 *dataStart = SimpleRamMatch(ROM_MEMPOOL, ROM_MEMPOOL + (size_t)GUEST_LOW_IMAGE_MAP_SIZE, (u8 *)PMU_Setting_Magic, matchLen);
    u32 guest_base = ROM_ADDRESS;
    u8 *pool_base = ROM_MEMPOOL;
    if (dataStart == NULL && s_ida_xram_host != NULL)
    {
        size_t spill = (size_t)size_16mb - (size_t)GUEST_LOW_IMAGE_MAP_SIZE;
        if (spill > (size_t)GUEST_IDA_XRAM_SIZE)
            spill = (size_t)GUEST_IDA_XRAM_SIZE;
        dataStart = SimpleRamMatch(s_ida_xram_host, s_ida_xram_host + spill, (u8 *)PMU_Setting_Magic, matchLen);
        if (dataStart != NULL)
        {
            guest_base = GUEST_IDA_XRAM_BASE;
            pool_base = s_ida_xram_host;
        }
    }
    if (dataStart != NULL)
    {
        MPU_Setting_ROM_Addr = ((u32)(dataStart - pool_base)) + guest_base + 32;
        printf("Find  MPU Setting Addr Success:%x\n", MPU_Setting_ROM_Addr);
    }
    else
        printf("Find MPU Setting Addr Failed\n");
}

void FindNorFlashId()
{
    u8 NorFlashID_Magic[] = {'C', 'O', 'M', 'B', 'O', 'M', 'E', 'M', '_', 'I', 'D', 0};

    u8 matchLen = sizeof(NorFlashID_Magic) / sizeof(NorFlashID_Magic[0]);

    printf("Find Match NorFlash ID...\n");
    u8 *dataStart = SimpleRamMatch(ROM_MEMPOOL, ROM_MEMPOOL + (size_t)GUEST_LOW_IMAGE_MAP_SIZE, (u8 *)NorFlashID_Magic, matchLen);
    if (dataStart == NULL && s_ida_xram_host != NULL)
    {
        size_t spill = (size_t)size_16mb - (size_t)GUEST_LOW_IMAGE_MAP_SIZE;
        if (spill > (size_t)GUEST_IDA_XRAM_SIZE)
            spill = (size_t)GUEST_IDA_XRAM_SIZE;
        dataStart = SimpleRamMatch(s_ida_xram_host, s_ida_xram_host + spill, (u8 *)NorFlashID_Magic, matchLen);
    }
    if (dataStart != NULL)
    {
        NorFlashID = *((u32 *)(dataStart + 25));
        printf("Find NorFlash Id Success:%x\n", NorFlashID);
    }
    else
        printf("Find NorFlash Id Failed\n");
}

u32 getMapAddr(u32 addr)
{
    if (addr >= 0x871A5F8 && addr <= (0x871A5F8 + 0x1188 - 0x200))
    {
        printf("addr %x map to %x\n", addr, (addr - 0x871A5F8 + 0x200 + 1));
    }
}

uint32_t nand_checksum(uint8_t *buf, uint32_t len)
{
    uint32_t sum = 0;
    for (uint32_t i = 0; i < len; i++)
        sum += buf[i];
    return sum; // 不取反，不模256
}

int main(int argc, char *args[])
{

    SetConsoleOutputCP(CP_UTF8);

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) < 0)
    {
        printf("SDL could not initialize! SDL_Error: %s\n", SDL_GetError());
        return -1;
    }
    /* 勿用 SDL_WINDOW_OPENGL：本工程用 GetWindowSurface 直接写像素，OpenGL 窗口下格式/stride 常异常 → 竖条花屏 */
#ifdef GDI_LAYER_DEBUG
    window = SDL_CreateWindow("moral i9 simulato", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH * 5, SCREEN_HEIGHT, 0);
#else
    window = SDL_CreateWindow("moral i9 simulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 240, 400, 0);
#endif
    if (window == NULL)
    {
        printf("Window could not be created! SDL_Error: %s\n", SDL_GetError());
        return -1;
    }

    initMtkSimalator();

    if (MTK != NULL)
    {
        u32 size = 0;
        u8 *tmp = 0;
        tmp = readFile(ROM_PROGRAM_BIN, &size);
        moral_uc_write_low_and_xram(tmp, size);
        SDL_free(tmp); 

        tmp = readFile("Rom\\moral-i9.bin", &size);
        my_memcpy(NandFlashCard, tmp, size);
        SDL_free(tmp);

        uc_mem_read(MTK, 0x34, &Interrupt_Handler_Entry, 4);
        printf("中断处理函数入口地址:%x\n", Interrupt_Handler_Entry);

        {
            u32 kpd_init = 0xFFFF;
            uc_mem_write(MTK, 0x34000814, &kpd_init, 4);
        }

        {
            time_t now = time(NULL);
            struct tm *lt = localtime(&now);
            u32 rtc_val;
            uc_err rtc_err;
            rtc_val = lt->tm_sec;
            rtc_err = uc_mem_write(MTK, RTC_SECOND_REG, &rtc_val, 4);
            rtc_val = lt->tm_min;
            rtc_err |= uc_mem_write(MTK, RTC_MINUTE_REG, &rtc_val, 4);
            rtc_val = lt->tm_hour;
            rtc_err |= uc_mem_write(MTK, RTC_HOUR_REG, &rtc_val, 4);
            rtc_val = lt->tm_mday;
            rtc_err |= uc_mem_write(MTK, RTC_DAY_REG, &rtc_val, 4);
            rtc_val = lt->tm_wday;
            rtc_err |= uc_mem_write(MTK, RTC_WEEKDAY_REG, &rtc_val, 4);
            rtc_val = lt->tm_mon;
            rtc_err |= uc_mem_write(MTK, RTC_MONTH_REG, &rtc_val, 4);
            rtc_val = lt->tm_year - 100;
            rtc_err |= uc_mem_write(MTK, RTC_YEAR_REG, &rtc_val, 4);
            u32 rb_hour = 0xFFFFFFFFu;
            uc_mem_read(MTK, RTC_HOUR_REG, &rb_hour, 4);
            printf("[RTC] init %02d:%02d:%02d write_err=%u readback_hour=%u (若 readback 异常则外设区未映射)\n",
                   lt->tm_hour, lt->tm_min, lt->tm_sec, (unsigned)rtc_err, rb_hour);
        }

        g_sd_img_max_bytes = 0;
        {
            const char *sd_img_path = NULL;
            SD_File_Handle = open_sd_image_with_fallback(&sd_img_path);
            if (SD_File_Handle != NULL && sd_img_path != NULL)
                printf("[SD] 使用镜像文件: %s\n", sd_img_path);
        }
        if (SD_File_Handle == NULL)
            printf("没有SD卡镜像文件，跳过加载\n");
        else
        {
            if (sd_img_get_file_size(SD_File_Handle, &g_sd_img_max_bytes) != 0)
            {
                printf("[SD] 无法读取镜像大小，写入将不加上限保护\n");
                g_sd_img_max_bytes = 0;
            }
            else
            {
                printf("[SD] 镜像打开大小: %llu 字节（写入不会超出此长度，避免被撑成数 GB）\n",
                       g_sd_img_max_bytes);
                sd_diagnose_image_geometry(SD_File_Handle, g_sd_img_max_bytes);
            }
            unsigned long sec = sd_get_reported_sectors_for_csd();
            unsigned long long mb = (unsigned long long)sec * 512ULL / (1024ULL * 1024ULL);
            printf("[SD] 镜像 CSD 报告容量: %lu 扇区 (~%llu MiB)\n", sec, mb);
        }

#if MORAL_EMU_DEDICATED_THREAD
        pthread_mutex_init(&g_lcd_frame_mutex, NULL);
        moral_emu_thread_stop = 0;
        pthread_create(&emu_thread, NULL, moral_pthread_run_arm, (void *)(uintptr_t)ROM_ADDRESS);
#endif
#ifdef GDB_SERVER_SUPPORT
        pthread_create(&gdb_server_mutex, NULL, gdb_server_main, NULL);
#endif
        printf("Unicorn Engine Initialized\n");

        loop();
    }
    return 0;
}

// 是否禁用IRQ中断
bool isIRQ_Disable(u32 cpsr)
{
    return (cpsr & (1 << 7));
}
bool isIrqMode(u32 cpsr)
{
    return (cpsr & 0xFFFFFFE0 | 0x12) == cpsr;
}
// 获取真机内存地址
char *getRealMemPtr(u32 ptr)
{
    if (ptr > 0xf0000000)
    {
        return ptr - 0xf0000000 + RAMF0_POOL;
    }
    else if (ptr > 0x40000000)
    {
        return ptr - 0x40000000 + RAM40_POOL;
    }
    else
        return RAM_MEMPOOL + ptr;
}
void hookBlockCallBack(uc_engine *uc, uint64_t address, uint32_t size, void *user_data)
{
    (void)uc;
    (void)address;
    (void)size;
    (void)user_data;
}

/* UC_HOOK_CODE 在取指后、执行当前 PC 指令之前调用；此处改寄存器/PC 即替代该条指令效果 */

u8 mssend_pop_log = 0;

/*
 * Thumb 指令 LDRH R1,[R1,#34]（半字约 0x8C49，hwll1_ReadE2pParameters 路径）。
 * 真机正常执行；当前 Unicorn 组合下会触发 UC_ERR_EXCEPTION，故在 0xEE9DC 用代码钩子
 * 手工读半字写回 R1 并将 PC+=2。与 ker_trace 无因果关系，仅调用顺序上可能相邻。
 */
static void moral_emu_thumb_ldrh_r1_r1_plus34(void)
{
    u32 base, cpsr, pc_raw, npc, r1_out;
    u16 v;

    uc_reg_read(MTK, UC_ARM_REG_R1, &base);
    {
        u32 ea = base + 34u;
        if (uc_mem_read(MTK, ea, &v, 2) != UC_ERR_OK)
            v = 0;
    }
    r1_out = (u32)v;
    uc_reg_write(MTK, UC_ARM_REG_R1, &r1_out);

    uc_reg_read(MTK, UC_ARM_REG_PC, &pc_raw);
    uc_reg_read(MTK, UC_ARM_REG_CPSR, &cpsr);
    npc = (pc_raw & ~1u) + 2u;
    if (cpsr & 0x20u)
        npc |= 1u;
    uc_reg_write(MTK, UC_ARM_REG_PC, &npc);
}

typedef enum
{
    UA_INT,
    UA_STR
} moral_uart_spec_t;

/* 扫描 printf 风格格式串中的转换说明（跳过 %%），最多 maxspec 个 */
static int moral_uart_scan_specs(const char *fmt, moral_uart_spec_t *specs, int maxspec)
{
    int n = 0;
    const char *p = fmt;

    while (*p && n < maxspec)
    {
        if (*p != '%')
        {
            p++;
            continue;
        }
        p++;
        if (*p == '%')
        {
            p++;
            continue;
        }
        while (*p == '-' || *p == '+' || *p == ' ' || *p == '#' || *p == '0')
            p++;
        while (*p >= '0' && *p <= '9')
            p++;
        if (*p == '*')
            p++;
        if (*p == '.')
        {
            p++;
            while (*p >= '0' && *p <= '9')
                p++;
            if (*p == '*')
                p++;
        }
        if (*p == 'l' && p[1] == 'l')
            p += 2;
        else if (*p == 'l' || *p == 'h' || *p == 'z' || *p == 'j' || *p == 't')
            p++;

        {
            char c = *p++;
            if (c == 'd' || c == 'i' || c == 'u' || c == 'x' || c == 'X' || c == 'o' || c == 'p' || c == 'c')
                specs[n++] = UA_INT;
            else if (c == 's')
                specs[n++] = UA_STR;
        }
    }
    return n;
}

/*
 * sys_UartPrintf 钩子：R0=格式串 Guest 地址，R1/R2/R3 为前三个实参（与 ARM AAPCS 一致）。
 * 按格式中的 %d/%u/%s 等顺序从寄存器取参并从 Guest 读字符串，snprintf 成一行再打印。
 */
static void moral_uart_hook_sprintf_line(u32 fmt_gva, u32 r1, u32 r2, u32 r3, char *out, size_t out_sz)
{
    char fmt[256];
    char b0[256], b1[256];
    moral_uart_spec_t sp[6];
    int ns;
    u32 rv[3];
    int ri;
    int i;
    int ints[4];
    int nint;
    const char *strs[2];
    int nstr;
    char pat[8];
    int pi;

    if (out_sz == 0)
        return;
    out[0] = '\0';

    if (fmt_gva < 0x1000u ||
        uc_mem_read(MTK, fmt_gva, (u8 *)fmt, sizeof(fmt) - 1) != UC_ERR_OK)
    {
        snprintf(out, out_sz, "<uart bad fmt @0x%x>", fmt_gva);
        return;
    }
    fmt[sizeof(fmt) - 1] = '\0';

    ns = moral_uart_scan_specs(fmt, sp, 6);
    rv[0] = r1;
    rv[1] = r2;
    rv[2] = r3;
    nint = 0;
    nstr = 0;
    ri = 0;

    for (i = 0; i < ns && ri < 3; i++)
    {
        u32 w = rv[ri++];
        if (sp[i] == UA_INT)
        {
            if (nint < 4)
                ints[nint++] = (int)(int32_t)w;
        }
        else
        {
            if (nstr >= 2)
                continue;
            if (w >= 0x1000u && uc_mem_read(MTK, w, (u8 *)(nstr == 0 ? b0 : b1), 255) == UC_ERR_OK)
                (nstr == 0 ? b0 : b1)[255] = '\0';
            else
                (nstr == 0 ? b0 : b1)[0] = '\0';
            strs[nstr] = (nstr == 0 ? b0 : b1);
            nstr++;
        }
    }

    if (i < ns)
    {
        snprintf(out, out_sz, "<uart need %d args, hook has 3 (fmt ok prefix: %s)>", ns, fmt);
        return;
    }

    for (pi = 0; pi < ns && pi < (int)sizeof(pat) - 1; pi++)
        pat[pi] = (sp[pi] == UA_INT) ? 'i' : 's';
    pat[pi] = '\0';

    if (ns == 0)
        snprintf(out, out_sz, "%s", fmt);
    else if (strcmp(pat, "i") == 0)
        snprintf(out, out_sz, fmt, ints[0]);
    else if (strcmp(pat, "ii") == 0)
        snprintf(out, out_sz, fmt, ints[0], ints[1]);
    else if (strcmp(pat, "iii") == 0)
        snprintf(out, out_sz, fmt, ints[0], ints[1], ints[2]);
    else if (strcmp(pat, "s") == 0)
        snprintf(out, out_sz, fmt, strs[0]);
    else if (strcmp(pat, "is") == 0)
        snprintf(out, out_sz, fmt, ints[0], strs[0]);
    else if (strcmp(pat, "si") == 0)
        snprintf(out, out_sz, fmt, strs[0], ints[0]);
    else if (strcmp(pat, "ss") == 0)
        snprintf(out, out_sz, fmt, strs[0], strs[1]);
    else if (strcmp(pat, "iis") == 0)
        snprintf(out, out_sz, fmt, ints[0], ints[1], strs[0]);
    else if (strcmp(pat, "isi") == 0)
        snprintf(out, out_sz, fmt, ints[0], strs[0], ints[1]);
    else if (strcmp(pat, "sii") == 0)
        snprintf(out, out_sz, fmt, strs[0], ints[0], ints[1]);
    else if (strcmp(pat, "sis") == 0)
        snprintf(out, out_sz, fmt, strs[0], ints[0], strs[1]);
    else if (strcmp(pat, "iss") == 0)
        snprintf(out, out_sz, fmt, ints[0], strs[0], strs[1]);
    else
        snprintf(out, out_sz, "<uart unhandled pat [%s]>", pat);
}

static u32 moral_ap_align(u32 ap, u32 al)
{
    return (ap + al - 1u) & ~(al - 1u);
}

static int moral_ap_pull_u32(u32 *ap, u32 *dst)
{
    *ap = moral_ap_align(*ap, 4u);
    if (uc_mem_read(MTK, *ap, dst, 4) != UC_ERR_OK)
        return -1;
    *ap += 4u;
    return 0;
}

static void moral_guest_read_cstr_short(u32 gva, char *dst, size_t dst_sz)
{
    size_t i;
    if (dst_sz == 0)
        return;
    if (gva < 0x1000u)
    {
        dst[0] = '\0';
        return;
    }
    for (i = 0; i + 1 < dst_sz; i++)
    {
        u8 b = 0;
        if (uc_mem_read(MTK, gva + (u32)i, &b, 1) != UC_ERR_OK)
            break;
        dst[i] = (char)b;
        if (b == 0)
            return;
    }
    dst[dst_sz - 1] = '\0';
}

/* 从 Guest 的 va_list 首址猜「下一实参」指针（GCC ARM EABI 常见：首字为 __ap） */
static u32 moral_arm_va_list_guess_ap(u32 vl)
{
    u32 w0 = 0, w1 = 0;
    if (vl < 0x1000u)
        return 0;
    if (uc_mem_read(MTK, vl, &w0, 4) != UC_ERR_OK)
        return 0;
    if (w0 >= 0x1000u && w0 < 0xF0000000u)
        return w0;
    if (uc_mem_read(MTK, vl + 4u, &w1, 4) == UC_ERR_OK && w1 >= 0x1000u && w1 < 0xF0000000u)
        return w1;
    return vl;
}

/* fp 指向 '%'，复制整条转换说明到 mini，返回指向说明符之后的指针 */
static const char *moral_copy_one_printf_spec(const char *fp, char *mini, size_t maxk)
{
    size_t k = 0;

    if (*fp != '%' || maxk < 4)
        return fp;
    mini[k++] = *fp++;
    if (*fp == '%')
    {
        if (k < maxk - 1)
            mini[k++] = *fp++;
        mini[k] = '\0';
        return fp;
    }
    while (k + 1 < maxk &&
           (*fp == '-' || *fp == '+' || *fp == ' ' || *fp == '#' || *fp == '0'))
        mini[k++] = *fp++;
    while (k + 1 < maxk && *fp >= '0' && *fp <= '9')
        mini[k++] = *fp++;
    if (k + 1 < maxk && *fp == '*')
        mini[k++] = *fp++;
    if (k + 1 < maxk && *fp == '.')
    {
        mini[k++] = *fp++;
        while (k + 1 < maxk && *fp >= '0' && *fp <= '9')
            mini[k++] = *fp++;
        if (k + 1 < maxk && *fp == '*')
            mini[k++] = *fp++;
    }
    if (k + 2 < maxk && fp[0] == 'l' && fp[1] == 'l')
    {
        mini[k++] = *fp++;
        mini[k++] = *fp++;
    }
    else
    {
        while (k + 1 < maxk &&
               (*fp == 'l' || *fp == 'h' || *fp == 'z' || *fp == 'j' || *fp == 't'))
            mini[k++] = *fp++;
    }
    if (k + 1 < maxk && *fp != '\0')
        mini[k++] = *fp++;
    mini[k] = '\0';
    return fp;
}

/* 统计 mini 里「宽度 / 精度」各用了几个 '*'（%*s、%.*s、%*.*s） */
static void moral_mini_star_layout(const char *mini, int *width_stars, int *prec_stars)
{
    const char *p;
    int dot = 0;

    *width_stars = 0;
    *prec_stars = 0;
    for (p = mini + 1; *p != '\0'; p++)
    {
        if (*p == '.')
            dot = 1;
        else if (*p == '*')
        {
            if (!dot)
                (*width_stars)++;
            else
                (*prec_stars)++;
        }
    }
}

/*
 * 按格式串从 Guest 的 ap 依次取参并拼成一行（与 OEMOS_dbgprintf→vsprintf 用法一致）。
 * 支持常见 %*s / %*d、%.*s、%*.*s；不支持 %n；其余含 * 仍占位且不取参（可能错位）。
 */
static void moral_vsprintf_guest_format_line(u32 fmt_gva, u32 va_list_gva, char *out, size_t out_sz)
{
    char fmt[640];
    char mini[48];
    char frag[384];
    char gstr[288];
    u32 ap;
    const char *fp;
    char *op;
    size_t rem;
    unsigned guard;

    if (out_sz == 0)
        return;
    out[0] = '\0';
    if (fmt_gva < 0x1000u)
    {
        snprintf(out, out_sz, "<vsprintf bad fmt @0x%x>", fmt_gva);
        return;
    }
    {
        size_t i;
        for (i = 0; i < sizeof(fmt) - 1u; i++)
        {
            u8 b = 0;
            if (uc_mem_read(MTK, fmt_gva + (u32)i, &b, 1) != UC_ERR_OK)
                break;
            fmt[i] = (char)b;
            if (b == 0)
                break;
        }
        fmt[i] = '\0';
    }

    ap = moral_arm_va_list_guess_ap(va_list_gva);
    if (ap == 0)
    {
        snprintf(out, out_sz, "<vsprintf bad va_list @0x%x>", va_list_gva);
        return;
    }

    fp = fmt;
    op = out;
    rem = out_sz;
    guard = 0;
    while (*fp != '\0' && rem > 1u && guard < 4096u)
    {
        guard++;
        if (fp[0] == '%' && fp[1] == '%')
        {
            *op++ = '%';
            fp += 2;
            rem--;
            continue;
        }
        if (*fp != '%')
        {
            *op++ = *fp++;
            rem--;
            continue;
        }

        fp = moral_copy_one_printf_spec(fp, mini, sizeof(mini));
        if (mini[0] != '%')
            break;

        {
            char tc = mini[strlen(mini) - 1u];
            int n = -1;
            int w_star = 0, p_star = 0;

            if (strchr(mini, '*') != NULL)
                moral_mini_star_layout(mini, &w_star, &p_star);

            if (w_star != 0 || p_star != 0)
            {
                int w = 0, prec = 0;
                u32 uw = 0, up = 0;

                if (w_star > 1 || p_star > 1 ||
                    (w_star && p_star && tc != 's' && tc != 'd' && tc != 'i' && tc != 'u' && tc != 'x' &&
                     tc != 'X' && tc != 'o' && tc != 'p'))
                {
                    n = snprintf(frag, sizeof(frag), "<*spec:%s>", mini);
                }
                else
                {
                    if (w_star)
                    {
                        if (moral_ap_pull_u32(&ap, &uw) != 0)
                            n = snprintf(frag, sizeof(frag), "<* pull w err>");
                        else
                            w = (int)(int32_t)uw;
                    }
                    if (n < 0 && p_star)
                    {
                        if (moral_ap_pull_u32(&ap, &up) != 0)
                            n = snprintf(frag, sizeof(frag), "<* pull p err>");
                        else
                            prec = (int)(int32_t)up;
                    }
                    if (n < 0 && tc == 's')
                    {
                        u32 p = 0;
                        if (moral_ap_pull_u32(&ap, &p) != 0)
                            n = snprintf(frag, sizeof(frag), "<*s pull ptr err>");
                        else
                        {
                            moral_guest_read_cstr_short(p, gstr, sizeof(gstr));
                            if (w_star && p_star)
                                n = snprintf(frag, sizeof(frag), "%*.*s", w, prec, gstr);
                            else if (w_star)
                                n = snprintf(frag, sizeof(frag), "%*s", w, gstr);
                            else
                                n = snprintf(frag, sizeof(frag), "%.*s", prec, gstr);
                        }
                    }
                    else if (n < 0 && (tc == 'd' || tc == 'i'))
                    {
                        u32 v = 0;
                        if (moral_ap_pull_u32(&ap, &v) != 0)
                            n = snprintf(frag, sizeof(frag), "<*di pull err>");
                        else
                        {
                            if (w_star && p_star)
                                n = snprintf(frag, sizeof(frag), "%*.*d", w, prec, (int)(int32_t)v);
                            else if (w_star)
                                n = snprintf(frag, sizeof(frag), "%*d", w, (int)(int32_t)v);
                            else
                                n = snprintf(frag, sizeof(frag), "%.*d", prec, (int)(int32_t)v);
                        }
                    }
                    else if (n < 0 &&
                             (tc == 'u' || tc == 'x' || tc == 'X' || tc == 'o' || tc == 'p'))
                    {
                        u32 v = 0;
                        if (moral_ap_pull_u32(&ap, &v) != 0)
                            n = snprintf(frag, sizeof(frag), "<*ux pull err>");
                        else if (w_star && p_star)
                            n = snprintf(frag, sizeof(frag), mini, w, prec, (unsigned)v);
                        else if (w_star)
                            n = snprintf(frag, sizeof(frag), mini, w, (unsigned)v);
                        else
                            n = snprintf(frag, sizeof(frag), mini, prec, (unsigned)v);
                    }
                    else if (n < 0)
                        n = snprintf(frag, sizeof(frag), "<*spec:%s>", mini);
                }

                if (n > 0 && (size_t)n < rem)
                {
                    memcpy(op, frag, (size_t)n);
                    op += (size_t)n;
                    rem -= (size_t)n;
                }
                continue;
            }

            if (tc == 'n')
            {
                n = snprintf(frag, sizeof(frag), "<%%n>");
            }
            else if (strstr(mini, "lld") != NULL || strstr(mini, "lli") != NULL)
            {
                long long v = 0;
                ap = moral_ap_align(ap, 8u);
                if (uc_mem_read(MTK, ap, &v, 8) != UC_ERR_OK)
                    n = snprintf(frag, sizeof(frag), "<lld read err>");
                else
                {
                    ap += 8u;
                    n = snprintf(frag, sizeof(frag), mini, v);
                }
            }
            else if (strstr(mini, "llu") != NULL || strstr(mini, "llx") != NULL ||
                     strstr(mini, "llo") != NULL)
            {
                unsigned long long v = 0;
                ap = moral_ap_align(ap, 8u);
                if (uc_mem_read(MTK, ap, &v, 8) != UC_ERR_OK)
                    n = snprintf(frag, sizeof(frag), "<llu read err>");
                else
                {
                    ap += 8u;
                    n = snprintf(frag, sizeof(frag), mini, v);
                }
            }
            else if (tc == 's')
            {
                u32 p = 0;
                if (moral_ap_pull_u32(&ap, &p) != 0)
                    n = snprintf(frag, sizeof(frag), "<s pull err>");
                else
                {
                    moral_guest_read_cstr_short(p, gstr, sizeof(gstr));
                    n = snprintf(frag, sizeof(frag), mini, gstr);
                }
            }
            else if (tc == 'f' || tc == 'F' || tc == 'g' || tc == 'G' || tc == 'e' || tc == 'E' ||
                     tc == 'a' || tc == 'A')
            {
                double dv = 0.0;
                ap = moral_ap_align(ap, 8u);
                if (uc_mem_read(MTK, ap, &dv, 8) != UC_ERR_OK)
                    n = snprintf(frag, sizeof(frag), "<float read err>");
                else
                {
                    ap += 8u;
                    n = snprintf(frag, sizeof(frag), mini, dv);
                }
            }
            else if (tc == 'd' || tc == 'i')
            {
                u32 v = 0;
                if (moral_ap_pull_u32(&ap, &v) != 0)
                    n = snprintf(frag, sizeof(frag), "<di pull err>");
                else
                    n = snprintf(frag, sizeof(frag), mini, (int)(int32_t)v);
            }
            else if (tc == 'u' || tc == 'x' || tc == 'X' || tc == 'o' || tc == 'p')
            {
                u32 v = 0;
                if (moral_ap_pull_u32(&ap, &v) != 0)
                    n = snprintf(frag, sizeof(frag), "<ux pull err>");
                else
                    n = snprintf(frag, sizeof(frag), mini, (unsigned)v);
            }
            else if (tc == 'c')
            {
                u32 v = 0;
                if (moral_ap_pull_u32(&ap, &v) != 0)
                    n = snprintf(frag, sizeof(frag), "<c pull err>");
                else
                    n = snprintf(frag, sizeof(frag), mini, (unsigned char)v);
            }
            else
            {
                n = snprintf(frag, sizeof(frag), "<fmt:%s>", mini);
            }

            if (n < 0 || (size_t)n >= rem)
                break;
            memcpy(op, frag, (size_t)n);
            op += (size_t)n;
            rem -= (size_t)n;
        }
    }
    *op = '\0';
}

void hookCodeCallBack(uc_engine *uc, uint64_t address, uint32_t size, void *user_data)
{
    u32 tmp1, tmp2, tmp3, tmp4;
    (void)uc;
    (void)size;
    (void)user_data;

    if (MORAL_HOOK_SYS_IS_CDC_PC != 0u)
    {
        u32 apc = (u32)address & ~1u;
        u32 cdc_lo = MORAL_HOOK_SYS_IS_CDC_PC & ~1u;
        if (apc == cdc_lo)
        {
            static u32 moral_cdc_fast_init_log;
            u32 lr = 0;
            uc_reg_read(MTK, UC_ARM_REG_LR, &lr);
            if (MORAL_B_IS_CDC_FAST_INIT_GVA != 0u)
            {
                u8 b = 0;
                if (uc_mem_read(MTK, MORAL_B_IS_CDC_FAST_INIT_GVA, &b, 1) == UC_ERR_OK)
                {
                    if (moral_cdc_fast_init_log < 128u)
                    {
                        moral_cdc_fast_init_log++;
                        printf("[sysIsCdcFastInitMode] #%u pc=0x%x lr=0x%x -> return %u (bIsCdcFastInitMode@0x%x)\n",
                               moral_cdc_fast_init_log, apc, lr, (unsigned)b,
                               (unsigned)MORAL_B_IS_CDC_FAST_INIT_GVA);
                    }
                }
                else if (moral_cdc_fast_init_log < 8u)
                {
                    moral_cdc_fast_init_log++;
                    printf("[sysIsCdcFastInitMode] pc=0x%x lr=0x%x uc_mem_read bIsCdcFastInitMode@0x%x failed\n",
                           apc, lr, (unsigned)MORAL_B_IS_CDC_FAST_INIT_GVA);
                }
            }
            else if (moral_cdc_fast_init_log < 8u)
            {
                moral_cdc_fast_init_log++;
                printf("[sysIsCdcFastInitMode] pc=0x%x lr=0x%x — set MORAL_B_IS_CDC_FAST_INIT_GVA in config.h to read byte\n",
                       apc, lr);
            }
        }
    }

    if (MORAL_HOOK_VSPRINTF_PC != 0u)
    {
        u32 apc = (u32)address & ~1u;
        u32 vlo = MORAL_HOOK_VSPRINTF_PC & ~1u;
        if (apc == vlo)
        {
            static u32 moral_vsprintf_hook_count;
            u32 r0 = 0, r1 = 0, r2 = 0;
            char line[768];
            int allow = (MORAL_VSPRINTF_HOOK_LOG_MAX == 0u ||
                         moral_vsprintf_hook_count < MORAL_VSPRINTF_HOOK_LOG_MAX);

            if (allow)
            {
                moral_vsprintf_hook_count++;
                uc_reg_read(MTK, UC_ARM_REG_R0, &r0);
                uc_reg_read(MTK, UC_ARM_REG_R1, &r1);
                uc_reg_read(MTK, UC_ARM_REG_R2, &r2);
                moral_vsprintf_guest_format_line(r1, r2, line, sizeof(line));
                printf("[vsprintf] dest=0x%x fmt@0x%x va@0x%x -> %s\n",
                       r0, r1, r2, line);
            }
        }
    }

    if (((u32)address & ~1u) == 0x31b9c)
    {
        static u32 cached_rtc = 0;
        static uint64_t last_rtc_time = 0;
        uint64_t now = moral_get_ticks_ms();
        if (cached_rtc == 0 || (now - last_rtc_time) >= 1000)
        {
            last_rtc_time = now;
            cached_rtc = moral_fw_rtc_seconds_since_2000();
        }
        uc_reg_write(MTK, UC_ARM_REG_R5, &cached_rtc);
    }

#ifdef GDB_SERVER_SUPPORT
    tmp2 = gdbTarget.simulate_pc_count;
    if (tmp2 == 0)
    {
        for (tmp1 = 0; tmp1 < gdbTarget.num_breakpoints; tmp1++)
        {
            if (gdbTarget.breakpoints[tmp1] == address)
            {
                gdbTarget.running = 0;
                tmp2 = 1;
                // printf("break point hit %x\n", address);
                break;
            }
        }
    }
    else
    {
        // 因为步进暂停
        gdbTarget.running = 0;
        gdbTarget.simulate_pc_count--;
    }
    if (tmp2)
    {
        send_gdb_response(&clients[0], "S05");
        // printf("break point hit %x\n", address);
        while (gdbTarget.running == 0) // 断点命中，等待下一步
            ;
    }
#endif
    // if (address == lastAddress)
    // {
    //     printf("Simulator has crashed at %x\n", address);
    //     while (1)
    //         ;
    // }

    /*
     * 触摸 MsSend 返回点（IDA ARM 地址，Thumb 时 PC 可能 +1）：
     * 219712: MdlTouchScreenStatusReport 内 BL MsSend 后的 CMP R0,#10
     * 219df0: _MdlTouchscreenRepeatADCProcess 内 BL MsSend 后的 CMP R0,#10
     */
    if (((u32)address & ~1u) == 0x219712u || ((u32)address & ~1u) == 0x219df0u)
    {
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        if (tmp1 != 10u)
        {
            tmp1 = 10u;
            uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        }
    }

    /*
     * dev_accGetLCDStatus @0x30f34a（Thumb）：读 main_LCD_Sleep；模拟器 RAM 未镜像时易判「关屏」，
     * _MdlTouchScreenDoMainJob 不走 DrvTsGetAdcData，触摸栈空转。直接返回 1。
     */
    if (((u32)address & ~1u) == 0x30f34au)
    {
        tmp1 = 1;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
    }

    if (((u32)address & ~1u) == 0xcd44u)
    {
        tmp1 = 1;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
    }

    /*
     * HalDispTransAddr @0x1e574: 固件为 DE DMA 做虚拟→物理地址映射 (addr >= 0xC000000 时减去 0xC000000)。
     * 模拟器中没有这种映射，减去后读到的是另一块内存区域的数据 = 花屏。
     * 直接跳过，让 DE 寄存器保留固件原始地址。
     */
    if (((u32)address & ~1u) == 0x1e574u)
    {
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
        uc_mem_write(MTK, tmp1, &tmp2, 4);
        tmp3 = 0;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp3);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp3);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp3);
    }

    /* MdlVkeyParser/MdlVkeySendMsg logging removed for performance */

    /* HalDispSWFMarkTrigger/HalDispReset logging removed for performance */

    /*
     * 绕过固件触摸屏校准转换，直接返回 SDL 屏幕坐标。
     * 固件校准公式依赖 NVRAM 中的校准数据（X/Y offset+scale），在模拟器中
     * 这些值可能为零或与模拟器的 ADC 映射不匹配，导致坐标全部错误。
     */
    if (((u32)address & ~1u) == 0x219878u) /* _MdlTouchscreenGetXCoordination */
    {
        tmp1 = touchX;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
    }
    if (((u32)address & ~1u) == 0x219848u) /* _MdlTouchscreenGetYCoordination */
    {
        tmp1 = touchY;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
    }

    switch (address)
    {
    /*
     * MsSend @0x34d236：POP {R4-R6,PC}，RtkSend 返回后 R4=邮箱 R5=消息指针。
     * 触摸上报包：byte0=50，offset2 半字=52，offset6 半字=17233（MdlTouchScreenStatusReport / RepeatADC）。
     * 模拟器里 RtkSend 常非 10，导致 UI 收不到触摸；仅对触摸包强制 RTK 成功码 10。
     */
    case 0x34d236:
    {
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R4, &tmp2);
        uc_reg_read(MTK, UC_ARM_REG_R5, &tmp3);
        if (tmp3 >= 0x1000u && tmp3 < 0xF0000000u)
        {
            u8 hdr[8];
            if (uc_mem_read(MTK, tmp3, hdr, sizeof hdr) == UC_ERR_OK)
            {
                u16 w2 = (u16)hdr[2] | ((u16)hdr[3] << 8);
                u16 w6 = (u16)hdr[6] | ((u16)hdr[7] << 8);
                if (hdr[0] == 50u && w2 == 52u && w6 == 17233u)
                {
                    if (mssend_pop_log < 20)
                    {
                        mssend_pop_log++;
                        if (MORAL_LOG_TOUCH_DEBUG)
                            printf("[TS-DBG] MsSend-POP: real_R0=%u mbox=%u (touch msg)\n", tmp1, tmp2);
                    }
                    if (tmp1 != 10u)
                    {
                        if (MORAL_LOG_TOUCH_DEBUG)
                            printf("[TS-DBG] MsSend-POP: FORCING R0 from %u to 10!\n", tmp1);
                        tmp1 = 10;
                        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
                    }
                }
            }
        }
        break;
    }
    case 0xc166:
        uc_reg_read(MTK, UC_ARM_REG_R0, &Lcd_Update_X);
        uc_reg_read(MTK, UC_ARM_REG_R1, &Lcd_Update_Y);
        uc_reg_read(MTK, UC_ARM_REG_R2, &Lcd_Update_W);
        uc_reg_read(MTK, UC_ARM_REG_R3, &Lcd_Update_H);
        break;
    case 0x1DEF62: /* HalUtilPHY2MIUAddr */
    {
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        break;
    }
    case 0x1DEF6C:
    case 0x1DEF6D: /* HalUtilPHY2MIUAddr 越界；避免 MVNS 得到 -1 误触发 KER */
    {
        static u32 phy_clamp_log;
        if (phy_clamp_log < 4u)
        {
            phy_clamp_log++;
            uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
            printf("[PHY2MIU] clamp: 固件将返回-1；改为返回0并跳过MVNS, raw_R0=0x%08x\n", tmp1);
        }
        tmp1 = 0;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_CPSR, &tmp3);
        tmp2 = 0x1DEF70u;
        if (tmp3 & 0x20u)
            tmp2 |= 1u; /* Thumb */
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
        break;
    }
    case 0x33370: /* MDrvFCIEStorageR 入口 — 保存 a4（SD DMA物理地址） */
    {
        uc_reg_read(MTK, UC_ARM_REG_R3, &tmp1);
        g_sd_dma_phys_addr = tmp1;
        if (MORAL_LOG_SD_IO)
            printf("[STG_R_HOOK] a4(phys)=0x%08x\n", tmp1);
        break;
    }
    case 0x336C2: /* MDrvFCIEStorageW 入口 — 保存 a4（写操作 DMA 物理地址） */
    {
        uc_reg_read(MTK, UC_ARM_REG_R3, &tmp1);
        g_sd_dma_phys_addr = tmp1;
        if (MORAL_LOG_SD_IO)
            printf("[STG_W_HOOK] a4(phys)=0x%08x\n", tmp1);
        break;
    }
    case 0x40ED0: /* Drv_DoDataCompare — 跳过写后数据比较验证，直接返回 0（成功） */
    {
        u32 lr;
        uc_reg_read(MTK, UC_ARM_REG_LR, &lr);
        tmp1 = 0;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_write(MTK, UC_ARM_REG_PC, &lr);
        break;
    }
    case 0x1a605c:
    case 0x2AE1C:
        printf("ker_assert_func (%x)\n", lastAddress);
        while (1)
            ;

        break;
    case 0x1ED480:
        printf("_RtkExceptionRoutine (%x)\n", lastAddress);
        while (1)
            ;
        break;

    case 0x32DFA4:
        uc_reg_read(MTK, UC_ARM_REG_R4, &tmp2);
        printf("_RtkAssertRoutine (%x)(%x)\n", tmp2, lastAddress);
        while (1)
            ;
        break;
    case 0x1D4B96:
        uc_reg_read(MTK, UC_ARM_REG_R0, &nandDmaBuffPtr);
        break;
    case 0x3b5a52 + 1:
#if MORAL_LOG_KERNEL_TRACE
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        if (tmp1 == 0xe19)
        {
            uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
            uc_reg_read(MTK, UC_ARM_REG_R2, &tmp1);
            uc_reg_read(MTK, UC_ARM_REG_R3, &tmp3);
            uc_mem_read(MTK, tmp2, globalSprintfBuff, 128);
            printf("[ker_trace]");
            printf("%s,%x,%x", globalSprintfBuff, tmp1, tmp3);
            printf("\n");
        }
#endif
        break;
    case 0x1FE99C:
        uc_reg_read(MTK, UC_ARM_REG_R1, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R2, &tmp2);
        uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
        break;
    case 0x3B5A00 + 1:
#if MORAL_LOG_KERNEL_TRACE
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
        uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
        uc_reg_read(MTK, UC_ARM_REG_R3, &tmp4);
        if (tmp2 >= 0x1000u && tmp2 < 0xF0000000u &&
            uc_mem_read(MTK, tmp2, globalSprintfBuff, 128) == UC_ERR_OK)
        {
            globalSprintfBuff[127] = '\0';
            printf("[KER_VTRACE]%s,%x,%x(%x)\n", globalSprintfBuff, tmp3, tmp4, (u32)address);
        }
        (void)tmp1;
#endif
        break;
    case 0x36ed44:
    case 0x3b5ba4:
        break;
    case 0x2cb28: /* DrvDMA2DCmdFinish */
    {
        tmp1 = 1;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
        break;
    }
    case 0x2c55e: /* DrvDMA2DIsHWBitBlt → 软路径 */
    case 0x2c5dc: /* DrvDMA2DIsHWFillRect → 软路径 */
    {
        tmp1 = 0;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
        break;
    }
    case 0xA144:
        uc_reg_read(MTK, UC_ARM_REG_R4, &tmp1);
        uc_reg_write(MTK, UC_ARM_REG_R6, &tmp1);

        break;
    case 0x2d5eca:
    {
        static u32 uart_cnt = 0;
        char uart_line[320];
        uart_cnt++;
        if (MORAL_UART_HOOK_PRINT_MAX == 0u || uart_cnt <= MORAL_UART_HOOK_PRINT_MAX)
        {
            uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
            uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
            uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
            uc_reg_read(MTK, UC_ARM_REG_R3, &tmp4);
            moral_uart_hook_sprintf_line(tmp1, tmp2, tmp3, tmp4, uart_line, sizeof(uart_line));
            printf("[uart_print] %s\n", uart_line);
        }
        break;
    }
    case 0x3B5C54:
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        {
            static u32 kerr_cnt = 0;
            kerr_cnt++;
            if (kerr_cnt <= 30)
            {
                if (tmp1 == 0xc34 || tmp1 == 0x10034 || tmp1 == 0x34)
                {
                    uc_reg_read(MTK, UC_ARM_REG_R2, &tmp2);
                    uc_reg_read(MTK, UC_ARM_REG_R3, &tmp3);
                    uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
                    printf("[KER_VER]%s,%x,%x(call:%x)\n", globalSprintfBuff, tmp2, tmp3, lastAddress);
                }
                else
                    printf("KER_VERROR_DIAGNOSE R0:%x (%x)\n", tmp1, lastAddress);
            }
        }
        break;
    case 0x7C322C: /* 跳过 mrc */
        address += 8;
        uc_reg_write(MTK, UC_ARM_REG_PC, &address);
        break;
    case 0x7C3238:
        address += 8;
        uc_reg_write(MTK, UC_ARM_REG_PC, &address);
        break;
    case 0xEE9DC:
    case 0xEE9DD:
    {
#if MORAL_LOG_UC_CODE_PATCH
        static u32 moral_ee9dc_log;
        if (moral_ee9dc_log < 4u)
        {
            u32 tpc = (u32)address & ~1u;
            u16 hw = 0;
            moral_ee9dc_log++;
            if (uc_mem_read(MTK, tpc, &hw, 2) == UC_ERR_OK)
                printf("[UC] patch LDRH R1,[R1,#34] @%08x half=%04x\n", tpc, hw);
        }
#endif
        moral_emu_thumb_ldrh_r1_r1_plus34();
        break;
    }
    }
    lastAddress = address;
}

static u32 irq_inject_count = 0;

bool StartInterrupt(u32 irq_line, u32 lastAddr)
{
    u32 tmp, mode;
    u32 tmp2;
    bool flag = false;
    if (irq_line < 32)
    {
        flag = (IRQ_MASK_SET_L_Data & (1 << irq_line));
    }
    else
    {
        flag = (IRQ_MASK_SET_H_Data & (1 << (irq_line - 32)));
    }
    if (flag)
    {
        tmp = (irq_line << 2);
        uc_mem_write(MTK, 0x34001840, &tmp, 4);
        uc_reg_read(MTK, UC_ARM_REG_CPSR, &tmp);
        if (!isIRQ_Disable(tmp))
        {
            u32 cur_mode = tmp & 0x1F;
            if (cur_mode == 0x12 || cur_mode == 0x11)
                return false;

            u32 thumb = tmp & 0x20;
            tmp2 = (tmp & 0xFFFFFFE0) | 0x12; // IRQ模式
            tmp2 = tmp2 | 0xC0;               // IRQ/FIQ Disable
            uc_reg_write(MTK, UC_ARM_REG_CPSR, &tmp2);
            uc_reg_write(MTK, UC_ARM_REG_SPSR, &tmp);

            tmp = lastAddr + 4;
            uc_reg_write(MTK, UC_ARM_REG_LR, &tmp);
            uc_mem_write(MTK, IRQ_Status, &irq_line, 4);
            uc_reg_write(MTK, UC_ARM_REG_PC, &Interrupt_Handler_Entry);

            irq_inject_count++;
            Perf_IrqInjectCount++;
            if (MORAL_LOG_HOT_PATH && (irq_inject_count <= 20 || (irq_inject_count % 500) == 0))
                printf("[IRQ] #%u line=%u pc=0x%x lr=0x%x thumb=%u mode=0x%x\n",
                       irq_inject_count, irq_line, lastAddr, lastAddr + 4, thumb ? 1 : 0, cur_mode);
            return true;
        }
    }
    return false;
}
