#define GDB_SERVER_SUPPORT_
#define GDI_LAYER_DEBUG_

#include "main.h"
#include "hookRam.c"

#include "dsp.h"
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
pthread_mutex_t mutex; // 线程锁
u32 lastFlashTime;
u32 lastTaskTime;

u32 lastSaveAdr = 0;
u32 lastSaveAdr2 = 0;

u8 isStepNext = 0;

u32 debugAddress;

SDL_Keycode isKeyDown = SDLK_UNKNOWN;
bool isMouseDown = false;
pthread_t emu_thread;
pthread_t screen_render_thread;

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

/**
 * @brief 按键事件
 * @param type 4=按下 5=松开
 * @param key 按键值
 */
void keyEvent(int type, int key)
{
    int simulateKey = -1;
    // printf("keyboard(%x,type=%d)\n", key, type);

    if (key == 0x4000003a && type == 4)
    {
        // 按下F1导出0x40008000
        isStepNext = 1;
    }
    if (key == 0x4000003b && type == 4)
    {
        // 按下F2导出0x00008000
        u8 *p = SDL_malloc(size_8mb);
        uc_mem_read(MTK, 0, p, size_8mb);
        writeFile("0x0000.bin", p, size_8mb);
        printf("成功导出0x0000.bin\n");
    }
    if (key == 0x4000003c && type == 4)
    {
        // 按下F3导出0x00008000
        u8 *p = SDL_malloc(size_8mb);
        uc_mem_read(MTK, 0xf0000000, p, size_8mb);
        writeFile("0xf000.bin", p, size_8mb);
        printf("成功导出0xf000.bin\n");
    }
    // F5导出Cpu信息
    if (key == 0x4000003e && type == 4)
    {
        dumpCpuInfo();
    }
    // F6 exit
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
    else if (key == 0x63) // c
    {
        simulateKey = 23; // 挂机
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
        // printf("smimula ket\n");
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

    moral_vm_touch_adc_request((u32)x, (u32)y);

    if (type == MR_MOUSE_MOVE)
    {
        static clock_t last_move_enqueue = 0;
        clock_t now = clock();
        clock_t min_iv = (clock_t)(CLOCKS_PER_SEC / 30);
        if (min_iv < 1) min_iv = 1;
        if (last_move_enqueue != 0 && (now - last_move_enqueue) < min_iv)
            return;
        last_move_enqueue = now;
    }
    EnqueueVMEvent(VM_EVENT_TOUCH_SCREEN_IRQ, type, (x << 16) | y);
}

void loop()
{
    void *thread_ret;
    SDL_Event ev;
    bool isLoop = true;
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
        if (Lcd_Need_Update)
            renderGdiBufferToWindow();

        SDL_Delay(1);
    }

    // 等待线程结束
    pthread_join(&emu_thread, &thread_ret);
    pthread_join(&screen_render_thread, &thread_ret);
    if (SD_File_Handle != NULL)
        fclose(SD_File_Handle);
    saveFlashFile();
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
u8 *readSDFile(u32 startPos, u32 size)
{
    u8 *tmp;
    u8 flag;
    if (SD_File_Handle == NULL)
    {
        return NULL;
    }
    // 为 tmp 分配内存
    tmp = (u8 *)SDL_malloc(size);
    if (tmp == NULL)
    {
        printf("申请文件内存失败");
        return NULL;
    }
    if (fseek(SD_File_Handle, startPos, SEEK_SET) > 0)
    {
        printf("移动文件指针失败");
        return NULL;
    }
    // 读取文件内容到 tmp 中
    size_t result = fread(tmp, 1, size, SD_File_Handle);
    if (result != size)
    {
        printf("读取SD卡文件失败 %x <> %x\n", result, size);
        SDL_free(tmp);
        return NULL;
    }
    return tmp;
}

bool writeSDFile(u8 *Buffer, u32 startPos, u32 size)
{
    u8 flag;
    if (SD_File_Handle == NULL)
    {
        return false;
    }
    if (fseek(SD_File_Handle, startPos, SEEK_SET) > 0)
    {
        printf("移动文件指针失败\n");
        return false;
    }
    size_t result = fwrite(Buffer, 1, size, SD_File_Handle);
    if (result != size)
    {
        printf("写入文件失败\n");
        return false;
    }
    return true;
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

void readFlashFile()
{
    u8 *tmp;
    u8 flag;
    u32 startPos;
    u32 size;
    FILE *FLASH_File_Handle = fopen(FLASH_IMG_PATH, "rb");
    if (FLASH_File_Handle == NULL)
    {
        printf("没有Flash文件，加载失败\n");
        return;
    }
    if (FLASH_File_Handle == NULL)
        return;
    fseek(FLASH_File_Handle, 0, SEEK_END);
    size = ftell(FLASH_File_Handle);
    // 读取文件内容到 tmp 中
    tmp = (u8 *)SDL_malloc(size);
    if (tmp == NULL)
        return;
    fseek(FLASH_File_Handle, 0, SEEK_SET);
    size_t result = fread(tmp, 1, size, FLASH_File_Handle);
    if (result != size)
    {
        printf("Flash文件大小校验不通过，加载失败\n");
        fclose(FLASH_File_Handle);
        SDL_free(tmp);
        return;
    }
    moral_uc_write_low_and_xram(tmp, size);
    SDL_free(tmp);
    fclose(FLASH_File_Handle);
    printf("Flash文件加载成功\n");
}

void saveFlashFile()
{
    int flag;
    FILE *FLASH_File_Handle = fopen(FLASH_IMG_TEMP_PATH, "wb");
    if (FLASH_File_Handle == NULL)
        return;
    char *tmp = SDL_malloc(size_16mb);
    if (tmp == NULL)
        return;
    {
        const size_t low_n = (size_t)GUEST_LOW_IMAGE_MAP_SIZE;
        size_t xr = (size_t)size_16mb - low_n;
        if (xr > (size_t)GUEST_IDA_XRAM_SIZE)
            xr = (size_t)GUEST_IDA_XRAM_SIZE;
        uc_mem_read(MTK, ROM_ADDRESS, tmp, low_n);
        uc_mem_read(MTK, GUEST_IDA_XRAM_BASE, tmp + low_n, xr);
        if (low_n + xr < (size_t)size_16mb)
            memset(tmp + low_n + xr, 0, (size_t)size_16mb - low_n - xr);
    }
    size_t result = fwrite(tmp, 1, size_16mb, FLASH_File_Handle);
    if (result != size_16mb)
    {
        fclose(FLASH_File_Handle);
        printf("写入Flash文件失败\n");
        return;
    }
    fclose(FLASH_File_Handle);
    flag = remove(FLASH_IMG_PATH);
    if (flag != 0)
    {
        printf("删除Flash文件失败");
        printf("errno = %d\n", errno);
    }
    if (rename(FLASH_IMG_TEMP_PATH, FLASH_IMG_PATH) != 0)
        printf("重命名flash_tmp失败");
    return;
}
/**
 * 初始化模拟CPU引擎与内存
 *
 */
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

    ROM2_MEMPOOL = SDL_malloc(size_32mb);

    /* 勿映射满 16MB：须留出 0x00D00000 给 IDA XRAM，否则与触摸/MMI 全局量冲突 → err 11 */
    err = uc_mem_map_ptr(MTK, ROM_ADDRESS, GUEST_LOW_IMAGE_MAP_SIZE, UC_PROT_ALL, ROM_MEMPOOL);
    //??
    err = uc_mem_map_ptr(MTK, 0x8000000, size_32mb, UC_PROT_ALL, ROM2_MEMPOOL);
    //??
    err = uc_mem_map_ptr(MTK, 0x7000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    // 也是一段RAM?
    err = uc_mem_map_ptr(MTK, 0x1000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    if (err)
    {
        printf("Failed mem  Rom map: %u (%s)\n", err, uc_strerror(err));
        return NULL;
    }
    // 一段Internal_Ram内存
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

    //??
    err = uc_mem_map_ptr(MTK, 0x10000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    //??
    err = uc_mem_map_ptr(MTK, 0x60000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));

    // DSP
    err = uc_mem_map_ptr(MTK, 0x50000000, size_16mb, UC_PROT_ALL, SDL_malloc(size_16mb));
    //??
    err = uc_mem_map_ptr(MTK, 0x1c000000, size_32mb, UC_PROT_ALL, SDL_malloc(size_32mb));

    //??
    err = uc_mem_map_ptr(MTK, 0x74000000, size_4mb, UC_PROT_ALL, SDL_malloc(size_4mb));
    //??
    err = uc_mem_map_ptr(MTK, 0x34000000, size_4mb, UC_PROT_ALL, SDL_malloc(size_4mb));
    // LCD控制器寄存器
    err = uc_mem_map_ptr(MTK, 0x90000000, size_4mb, UC_PROT_ALL, SDL_malloc(size_4mb));
    if (err)
    {
        printf("Failed mem  Rom map: %u (%s)\n", err, uc_strerror(err));
        return NULL;
    }
    /*
     * MT6252 风格外设地址空间：RTC(0x810b****)、GPT(0x8106****)、AUX 触摸(0x8205****)、
     * IRQ 旧寄存器(0x8101****)、SDC/SIM 等。此前未映射时 uc_mem_write 无效，
     * MEM 钩子也不会触发，表现为时间卡在镜像值、触摸/电量逻辑不生效。
     * 若控制台曾大量出现 hookRamErrorBack「地址无法访问:810xxxxx / 820xxxxx」即此原因。
     */
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
    }
    /* 只对需要 hook 的地址范围注册 UC_HOOK_CODE，避免每条指令都调用回调 */
    {
        static uc_hook code_hooks[32];
        int hi = 0;
        static const u32 hook_ranges[][2] = {
            {0xA144,    0xA145},     /* pin config */
            {0xCD44,    0xCD45},     /* LCD params */
            {0x1E574,   0x1E575},    /* HalDispTransAddr */
            {0x2AE1C,   0x2AE1D},    /* ker_assert */
            {0x2C55E,   0x2C55F},    /* DrvDMA2DIsHWBitBlt */
            {0x2C5DC,   0x2C5DD},    /* DrvDMA2DIsHWFillRect */
            {0x2CB28,   0x2CB29},    /* DrvDMA2DCmdFinish */
            {0x2D5ECA,  0x2D5ECB},   /* uart_print */
            {0x31B9C,   0x31B9D},    /* RTC seconds hook */
            {0x1A605C,  0x1A605D},   /* ker_assert_func */
            {0x1D4B96,  0x1D4B97},   /* nand_translate_DMA */
            {0x1ED480,  0x1ED481},   /* _RtkExceptionRoutine */
            {0x1FE99C,  0x1FE99D},   /* LOG_SD */
            {0x219712,  0x219713},   /* MdlTouchScreenStatusReport MsSend return */
            {0x219848,  0x219849},   /* _MdlTouchscreenGetYCoordination */
            {0x219878,  0x219879},   /* _MdlTouchscreenGetXCoordination */
            {0x219DF0,  0x219DF1},   /* _MdlTouchscreenRepeatADCProcess MsSend return */
            {0x30F34A,  0x30F34B},   /* dev_accGetLCDStatus */
            {0x32DFA4,  0x32DFA5},   /* _RtkAssertRoutine */
            {0x34D236,  0x34D237},   /* MsSend POP */
            {0x36ED44,  0x36ED45},   /* fatal error check */
            {0x3B5A00,  0x3B5A01},   /* KER_VTRACE */
            {0x3B5A52,  0x3B5A53},   /* ker_trace */
            {0x3B5BA4,  0x3B5BA5},   /* fatal error check */
            {0x3B5C54,  0x3B5C55},   /* KER error */
            {0x7C322C,  0x7C3238},   /* skip mrc instructions */
            {0x800160C, 0x800160D},  /* KER error high addr */
            {0x1C007160,0x1C007161}, /* KER error high addr */
        };
        for (u32 ri = 0; ri < sizeof(hook_ranges)/sizeof(hook_ranges[0]); ri++)
        {
            err = uc_hook_add(MTK, &code_hooks[hi++], UC_HOOK_CODE,
                              hookCodeCallBack, 0,
                              hook_ranges[ri][0], hook_ranges[ri][1]);
        }
    }

    err = uc_hook_add(MTK, &trace[4], UC_HOOK_BLOCK, hookBlockCallBack, 0, 0, 0xefffffff);

    /* 只对 IO 寄存器范围注册 MEM hook，避免每次普通 RAM 读写都调用回调 */
    {
        static uc_hook mem_hooks[16];
        int mi = 0;
        static const u32 mem_ranges[][2] = {
            {0x34000000, 0x3400FFFF},  /* MTK 外设寄存器 (IRQ/Timer/UART/AUXADC) */
            {0x74000000, 0x74006FFF},  /* DE/DMA/FCIE/SPI */
            {0x81060000, 0x810600FF},  /* GPT */
            {0x82050000, 0x82050FFF},  /* AUX 触摸 */
            {0x90000000, 0x90000FFF},  /* LCD 控制器 */
            {0x00D00000, 0x00D0FFFF},  /* XRAM 全局量 (RF config) */
        };
        for (u32 ri = 0; ri < sizeof(mem_ranges)/sizeof(mem_ranges[0]); ri++)
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

void RunArmProgram(void *startAddr)
{
    u32 startAddress = (u32)startAddr;

    uc_err p;
    // 启动前工作

#ifdef GDB_SERVER_SUPPORT
    gdbTarget.running = 1;
    gdbTarget.breakpoints[gdbTarget.num_breakpoints++] = startAddress;
    // 等待连接
    readAllCpuRegFunc = ReadRegsToGdb;
    gdb_readMemFunc = readMemoryToGdb;
#endif
    p = uc_emu_start(MTK, startAddress, -1, 0, 0);
    // p = uc_emu_start(MTK, 3, 8, 0, 0);

    if (p == UC_ERR_READ_UNMAPPED)
        printf("模拟错误：此处内存不可读\n", p);
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
}

void MainUpdateTask()
{
    while (1)
    {
        currentTime = clock();

#ifdef GDB_SERVER_SUPPORT
        if (gdbTarget.running == 0)
        {
            usleep(1000);
            continue;
        }
#endif
        /*
         * 刷新显示/RTC 不得依赖 VmEventCount。LCD 中断等会高频入队且 Dequeue 很慢，
         * 队列接近满时若跳过 lcdTaskMain，周期性显存同步停止 → 锁屏后画面与时间“冻住”。
         */
        lcdTaskMain();
        RtcTaskMain();
        GptTaskMain();

        {
            static clock_t last_os_tick_time = 0;
            if (currentTime > last_os_tick_time)
            {
                last_os_tick_time = currentTime + 5;
                timer_event_pending = 1;
            }
        }
        if (currentTime > lastFlashTime)
        {
            lastFlashTime = currentTime + 300;
            fflush(stdout);
        }

        usleep(1000);
    }
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

        // // 每隔512字节进行一次检查
        // u8 *p = tmp;
        // int sector = 0;
        // while (1)
        // {
        //     int *q = p;
        //     int sum = *q;
        //     int check = nand_checksum(p + 4, 508);
        //     printf("check sector[%d] sum[%x] =? check[%x]\n", sector, sum, check);
        //     sector++;
        //     p += 512;
        //     if (sector > 1)
        //         break;
        // }

        // 设置nandflash的类型为0x4e00处的
        // tmp = readFile("Rom\\NANDINFO_v2.nni", &size);
        // my_memcpy(nandFlashData, tmp + 0x400, 0x200);
        // SDL_free(tmp);

        // // 读取PARTITION_INFO列表
        // tmp = readFile("Rom\\PARTITION_v2.pni", &size);
        // my_memcpy(nandFlashData + 0x800, tmp, 0x400);
        // SDL_free(tmp);

        // // 读取prt文件
        // tmp = readFile("Rom\\8533n_7835.prt", &size);
        // my_memcpy(nandFlashData + 0x4280 * 0x800, tmp, size);
        // SDL_free(tmp);

        // 读取FAT文件系统
        // tmp = readFile("Rom\\FatImage.fat", &size);
        // my_memcpy(nandFlashData + 0x42c0 * 0x800, tmp, size);
        // SDL_free(tmp);

        // 尝试完整固件
        // tmp = readFile("Rom\\TELEGO_T98_MSTAR.bin", &size);
        // moral-i9.bin
        // tmp = readFile("Rom\\hx555.bin", &size);
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

        SD_File_Handle = fopen(SD_CARD_IMG_PATH, "r+b");
        if (SD_File_Handle == NULL)
            printf("没有SD卡镜像文件，跳过加载\n");

        // 启动emu线程
        pthread_create(&emu_thread, NULL, RunArmProgram, ROM_ADDRESS);
        pthread_create(&screen_render_thread, NULL, MainUpdateTask, NULL);
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
    handleVmEvent_EMU(address);
}

/**
 * pc指针指向此地址时执行(未执行此地址的指令)
 */

u8 mssend_pop_log = 0;

void hookCodeCallBack(uc_engine *uc, uint64_t address, uint32_t size, void *user_data)
{
    u32 tmp1, tmp2, tmp3, tmp4;

    if (((u32)address & ~1u) == 0x31b9c)
    {
        static u32 cached_rtc = 0;
        static clock_t last_rtc_time = 0;
        clock_t now = clock();
        if (cached_rtc == 0 || (now - last_rtc_time) >= CLOCKS_PER_SEC)
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
        tmp1 = adc_snapshot_x;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
    }
    if (((u32)address & ~1u) == 0x219848u) /* _MdlTouchscreenGetYCoordination */
    {
        tmp1 = adc_snapshot_y;
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
                        printf("[TS-DBG] MsSend-POP: real_R0=%u mbox=%u (touch msg)\n", tmp1, tmp2);
                    }
                    if (tmp1 != 10u)
                    {
                        printf("[TS-DBG] MsSend-POP: FORCING R0 from %u to 10!\n", tmp1);
                        tmp1 = 10;
                        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
                    }
                }
            }
        }
        break;
    }
    // case 0x41BF4:
    //     printf("HalPagingSpiBusTransactionStart %x\n", lastAddress);
    //     break;
    // case 0x1DBCB0: // 强制FTL初始化成功
    //     tmp1 = 0;
    //     uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
    //     break;
    // case 0x46a02:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     printf("_DrvCARDREADER_SetCMD_RSP_BUF %x,%x\n", tmp1, tmp2);
    //     break;
    // case 0x46B3E:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     printf("HalFcieWaitMieEvent:%x\n", &tmp1);
    //     break;
    // case 0x1416ce:
    //     tmp1 = 0;
    //     uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
    //     printf("CusIsNandEnabled false\n");
    //     break;
    // case 0x37DD5E: // 跳过MS_Sleep
    //     tmp2 = 0x37DD66 + 1;
    //     uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
    //     break;
    // case 0x3B46A: // 强制在中断中清除
    //     // printf("clear mie flag\n");
    //     FICE_Status = 0; // 清零
    //     break;
    // case 0x1e574:
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp1);
    //     printf("HalDispTransAddr(%x)\n", tmp1);
    //     // uc_mem_read(MTK, tmp1, Lcd_Cache_Buffer, 240 * 400);
    //     Lcd_Need_Update = 1;
    //     break;
    // case 0x1604A:
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp1);
    //     printf("Read EEP ROM at %x\n", tmp1 + 0x14);
    //     break;
    // case 0x1CB84C:
    //     tmp2 = 1;
    //     uc_reg_write(MTK, UC_ARM_REG_R0, &tmp2);
    //     break;
    case 0xc166:
        uc_reg_read(MTK, UC_ARM_REG_R0, &Lcd_Update_X);
        uc_reg_read(MTK, UC_ARM_REG_R1, &Lcd_Update_Y);
        uc_reg_read(MTK, UC_ARM_REG_R2, &Lcd_Update_W);
        uc_reg_read(MTK, UC_ARM_REG_R3, &Lcd_Update_H);
        break;
    // case 0x1A6E70:
    //     tmp2 = 1;
    //     uc_reg_write(MTK, UC_ARM_REG_R3, &tmp2);
    //     break;
    // case 0x1A6E50:
    //     tmp2 = 2;
    //     uc_reg_write(MTK, UC_ARM_REG_R3, &tmp2);
    //     break;
    // case 0xAB5AC:
    // case 0xCABA2: // 跳过检测
    //     tmp2 = 8;
    //     uc_reg_write(MTK, UC_ARM_REG_R2, &tmp2);
    //     break;
    // case 0xAB5B6:
    // case 0xCABAC: // 跳过检测
    //     tmp2 = 2;
    //     uc_reg_write(MTK, UC_ARM_REG_R1, &tmp2);
    //     break;
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
        uc_reg_write(MTK, UC_ARM_REG_R4, &tmp2);
        printf("_RtkAssertRoutine (%x)(%x)\n", tmp2, lastAddress);
        while (1)
            ;
        break;
    case 0x1D4B96:
        uc_reg_read(MTK, UC_ARM_REG_R0, &nandDmaBuffPtr);
        // printf("nand_translate_DMA_address_Ex(%x)\n", nandDmaBuffPtr);
        break;
    // case 0x37f5ae:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
    //     printf("fms_E2pRead(%s,%x)\n", globalSprintfBuff, tmp2);
    //     break;
    // case 0x525e20:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
    //     uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
    //     printf("[vm_log_trace]");
    //     printf("%s,%x,%x", globalSprintfBuff, tmp2, tmp3);
    //     printf("\n");
    //     break;
    case 0x3b5a52 + 1:
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
        break;
    case 0x1FE99C:
        uc_reg_read(MTK, UC_ARM_REG_R1, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R2, &tmp2);
        printf("[LOG_SD]");
        uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
        printf("%s,%x", globalSprintfBuff, tmp2);
        printf("(call:%x)\n", lastAddress);
        break;
    case 0x3B5A00 + 1:
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R1, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_R2, &tmp2);
        uc_reg_read(MTK, UC_ARM_REG_R3, &tmp3);
        uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
        printf("[KER_VTRACE]");
        printf("%s,%x,%x", globalSprintfBuff, tmp2, tmp3);
        printf("(%x)\n", lastAddress);
        break;
    case 0x36ed44:
    case 0x3b5ba4:
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        if ((tmp1 & 0x110000) != 0)
        {
            printf("fatal error3: %x \n", lastAddress);
            while (1)
                ;
        }
        break;
    // case 0x12E92:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     printf("NC_EraseBlk(idx:%x)\n", tmp1);
    //     break;
    // case 0xF924:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_reg_read(MTK, UC_ARM_REG_SP, &tmp3);
    //     uc_mem_read(MTK, tmp3, &tmp4, 4);
    //     uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
    //     printf("NC_PageCopy(%x,%x,buff:%x,cnt:%x)\n", tmp1, tmp2, tmp3, tmp4);
    //     break;
    // case 0xF938:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_reg_read(MTK, UC_ARM_REG_SP, &tmp3);
    //     uc_mem_read(MTK, tmp3, &tmp4, 4);
    //     printf("NC_ReadSectors(idx:%x,sep:%x,cnt:%x)\n", tmp1, tmp2, tmp4);
    //     break;

    // case 0x1223C:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_reg_read(MTK, UC_ARM_REG_SP, &tmp3);
    //     uc_mem_read(MTK, tmp3, &tmp4, 4);
    //     printf("NC_WriteSectors(idx:%x,sep:%x,cnt:%x)\n", tmp1, tmp2, tmp4);
    //     break;
    // case 0xF916:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R3, &tmp2);
    //     printf("NC_ReadPages(idx:%x,cnt:%d)(%x)\n", tmp1, tmp2, lastAddress);
    //     break;
    // case 0xF908:
    //     debugType = 10;
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R3, &tmp2);
    //     printf("NC_WritePages(idx:%x,cnt:%d)(%x)\n", tmp1, tmp2, lastAddress);
    //     break;
    // case 0x9c68:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
    //     u32 tt = (tmp1 << 20) >> 28;
    //     if(tt==0){//g_ptKePadCtrl  0x74006600

    //     }else if(tt==1){//g_ptKeGpioCtrl0 0x74007400

    //     }else if(tt==2){//g_ptKeGpioCtrl1 0x74007600

    //     }else if(tt==3){//g_ptKeGpioCtrl2 0x74007800

    //     }

    //     printf("SetRegValue(%x,%x,%x)\n", tt, tmp2, tmp3);
    //     break;

    // case 0x1C00D124: // 跳过kmDevCheck
    //     tmp1 = 0x1C00D131;
    //     uc_reg_write(MTK, UC_ARM_REG_PC, &tmp1);
    //     break;
    case 0x2cb28: // DrvDMA2DCmdFinish - skip and return success
    {
        tmp1 = 1;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
        break;
    }
    case 0x2c55e: // DrvDMA2DIsHWBitBlt - force software path
    case 0x2c5dc: // DrvDMA2DIsHWFillRect - force software path
    {
        tmp1 = 0;
        uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
        uc_reg_read(MTK, UC_ARM_REG_LR, &tmp2);
        uc_reg_write(MTK, UC_ARM_REG_PC, &tmp2);
        break;
    }
    // case 0x1189DA:
    //     uc_reg_read(MTK, UC_ARM_REG_R2, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R5, &tmp2);
    //     printf("Compare R2<>R5 %x %x\n", tmp1, tmp2);
    //     break;
    // case 0x1C00D420:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     printf("KmMemoryGet(%x,%x)(%x)\n", tmp1, tmp2, lastAddress);
    //     break;
    // case 0x1C00D122:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
    //     uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
    //     printf("__KmDevCheck(%x,%x,%x)(%x)\n", tmp1, tmp2, tmp3, lastAddress);
    //     break;
    case 0xA144: // 强制引脚配置正确
        uc_reg_read(MTK, UC_ARM_REG_R4, &tmp1);
        uc_reg_write(MTK, UC_ARM_REG_R6, &tmp1);

        break;
    case 0x2d5eca:
    {
        static u32 uart_cnt = 0;
        uart_cnt++;
        if (uart_cnt <= 20)
        {
            uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
            uc_reg_read(MTK, UC_ARM_REG_R1, &tmp2);
            uc_reg_read(MTK, UC_ARM_REG_R2, &tmp3);
            uc_mem_read(MTK, tmp1, globalSprintfBuff, 128);
            if (strstr(globalSprintfBuff, "%30s") == NULL && strstr(globalSprintfBuff, "%s") == NULL)
                printf("[uart_print]%s,%x,%x(%x)\n", globalSprintfBuff, tmp2, tmp3, lastAddress);
            else
            {
                uc_mem_read(MTK, tmp2, sprintfBuff, 128);
                printf("[uart_print]%s,%x,%x\n", globalSprintfBuff, sprintfBuff);
            }
        }
        break;
    }
    // case 0x28c884:
    // case 0xbe0c:
    //     printf("DrvLcdWriteSingleCmd(%x)\n", lastAddress);
    //     break;
    // case 0x19c0fa: // DrvDispWriteLcm
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     printf("Cmd(%x)(%x)\n", tmp1, lastAddress);
    //     break;
    // case 0x19c112:
    //     uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
    //     printf("DrvDispWriteLcmData(%x)(%x)\n", tmp1, lastAddress);
    //     break;
    case 0x800160C:
    case 0x1C007160:
    case 0x3B5C54:
        uc_reg_read(MTK, UC_ARM_REG_R0, &tmp1);
        if ((tmp1 & 0x110000) != 0)
        {
            printf("fatal error2: %x \n", lastAddress);
            while (1)
                ;
        }
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
    // case 0x13C38: // HalClkgenBbtopSetClkSpeed强制返回0
    //     tmp1 = 0;
    //     uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
    //     break;
    // case 0x1A9C:
    //     uc_reg_read(MTK, UC_ARM_REG_R2, &tmp1);
    //     uc_reg_write(MTK, UC_ARM_REG_R1, &tmp1);
    //     break;
    // case 0x1AA0: // 跳过HalTimerUDelay
    //     tmp1 = 0;
    //     uc_reg_write(MTK, UC_ARM_REG_R0, &tmp1);
    //     break;
    case 0x7C322C: // 跳过mrc指令
        address += 8;
        uc_reg_write(MTK, UC_ARM_REG_PC, &address);
        break;
    case 0x7C3238: // 跳过tc_loop的mrc
        address += 8;
        uc_reg_write(MTK, UC_ARM_REG_PC, &address);
        break;
    }
    lastAddress = address;
}

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
            // printf("Start IRQ %x\n", irq_line);
            tmp2 = (tmp & 0xFFFFFFE0) | 0x12; // IRQ模式
            tmp2 = tmp2 | 0xC0;               // IRQ/FIQ Disable
            uc_reg_write(MTK, UC_ARM_REG_CPSR, &tmp2);
            uc_reg_write(MTK, UC_ARM_REG_SPSR, &tmp);
            tmp = lastAddr + 4;
            uc_reg_write(MTK, UC_ARM_REG_LR, &tmp);
            uc_mem_write(MTK, IRQ_Status, &irq_line, 4);
            uc_reg_write(MTK, UC_ARM_REG_PC, &Interrupt_Handler_Entry);
            return true;
        }
    }
    return false;
}
