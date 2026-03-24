// 定时中断配置，单位毫秒
#define Timer_Interrupt_Duration 1000
// SD卡镜像文件配置
#define SD_CARD_IMG_PATH "Rom\\fat32.img"

// 系统固件文件
#define ROM_PROGRAM_BIN "Rom\\output.bin"


#define ROM_ADDRESS 0

/*
 * IDA：XRAM 从 0x00D00000（触摸全局 _gtTouchScreenSusbribeData@0xD09198 等）。
 * 若 [0,0x1000000) 整段映射为 ROM，会与 0x00D00000 重叠 → uc_mem_map_ptr(XRAM) 失败，日志 err 11，触摸补丁无效。
 */
#define GUEST_IDA_XRAM_BASE       0x00D00000u
#define GUEST_IDA_XRAM_SIZE       0x00300000u /* 含 IDA XRAM 与原 16MB 映像尾 [0xD00000,0x1000000) */
#define GUEST_LOW_IMAGE_MAP_SIZE  0x00D00000u

/*
 * IDA「Program Segmentation」与本工程 Guest 映射对应关系（本固件）：
 *  - 0x00000000 .. ~0x00F41E58：ROM / CUST_* / XRAM / RAM_JAVA / DEBUG_AREA 等
 *      → loadRom 写入 [0, GUEST_LOW_IMAGE_MAP_SIZE) + [GUEST_IDA_XRAM_BASE, +GUEST_IDA_XRAM_SIZE)
 *  - IRAM_SECTION0 .. IRAM_RF_SECTION：0x08000000 .. 0x08005BAC
 *  - IRAM_SECTION2（含 .prgend/abs）：0x1C000000 .. ~0x1C010C9C
 * MAP_SIZE 取 32MB：覆盖段尾并留余量，避免固件偶发越界 UC_ERR_MAP（须 4KB 对齐）。
 */
#define GUEST_IRAM_SECTION0_BASE      0x08000000u
#define GUEST_IRAM_SECTION0_MAP_SIZE  0x02000000u
#define GUEST_IRAM_SECTION2_BASE      0x1C000000u
#define GUEST_IRAM_SECTION2_MAP_SIZE  0x02000000u

/*
 * HalRtcGetSecondCount 钩子注入的「秒计数」与 mktime(2000-01-01 00:00:00 本地) 的差值修正。
 * 若状态栏比宿主机慢整 16 小时（例：主机 17:18 显示 01:18），填 57600 (16*3600)。
 * 若已对齐则改为 0。
 */
#ifndef MORAL_RTC_SECOND_OFFSET
#define MORAL_RTC_SECOND_OFFSET 0
#endif

/*
 * 在 OFFSET 之后再叠加的秒数（可为负）。
 * 例：宿主机 2026-3-21 17:36 却显示 2034-3-22 01:36 → 约多 8 年 + 8 小时：
 *     -252460800(8×365.25天级近似) - 28800 = -252489600。仍差 1 小时可再 ±3600 微调。
 */
#ifndef MORAL_RTC_EXTRA_SECONDS
#define MORAL_RTC_EXTRA_SECONDS (-252489600LL)
#endif

/*
 * 周期性从模拟器显存同步到 SDL（毫秒）。固件若不再频繁写 DE 触发寄存器 0x7400313C，
 * 仅靠钩子无法刷新界面（时钟冻结、黑屏等）。设为 0 可关闭以省 CPU。
 */
#ifndef MORAL_LCD_PERIODIC_REFRESH_MS
#define MORAL_LCD_PERIODIC_REFRESH_MS 0
#endif

/*
 * DE 延迟刷新：两次从 Guest 读帧缓冲的最小间隔（毫秒）。
 * 设为 0 表示不限制（推荐，避免丢帧）。若设 >0，节流时须保留待刷新标志（旧实现曾错误丢弃整次刷新导致严重卡顿）。
 */
#ifndef MORAL_DE_FLUSH_MIN_INTERVAL_MS
#define MORAL_DE_FLUSH_MIN_INTERVAL_MS 0
#endif

/*
 * 全屏/周期拉显存时按固件 HalDisp 多图层（0x74003054 / 68 / 80）与 PIP（0x740030A0 / B0）
 * 做 RGB565 压盖合成。0=只读主层，Guest 读带宽更低但子层（状态栏叠加等）可能缺。
 */
#ifndef MORAL_DE_LAYER_COMPOSITE
#define MORAL_DE_LAYER_COMPOSITE 1
#endif

/*
 * 多图层合成时按 HalDisp 镜像寄存器做色键（0x740030DC..FC）与 Alpha（0x740030D0..D8）。
 * 0=始终不透明整块 memcpy，略快但透明区域会糊成实心色导致花屏/错位感。
 */
#ifndef MORAL_DE_BLEND_COLORKEY
#define MORAL_DE_BLEND_COLORKEY 1
#endif
#ifndef MORAL_DE_BLEND_ALPHA
#define MORAL_DE_BLEND_ALPHA 1
#endif

/*
 * SDL_UpdateWindowSurface 最小间隔（毫秒）。0=每帧可 present（更顺滑，略增 CPU）。
 */
#ifndef MORAL_LCD_PRESENT_MIN_INTERVAL_MS
#define MORAL_LCD_PRESENT_MIN_INTERVAL_MS 0
#endif

/* 主循环内连续 uc_emu_start 的时间预算（毫秒）；仅当 MORAL_EMU_DEDICATED_THREAD=0 时有效 */
#ifndef MORAL_EMU_MS_PER_MAIN_ITER
#define MORAL_EMU_MS_PER_MAIN_ITER 10u
#endif

/* 1：独立 pthread 跑模拟；0：在主循环里按时间片 uc_emu_start */
#ifndef MORAL_EMU_DEDICATED_THREAD
#define MORAL_EMU_DEDICATED_THREAD 1
#endif

/*
 * 1：uc_emu_start 使用 timeout=0、count=0（不按时间/条数截断，直到 uc_emu_stop 或出错）。
 * 主线程入队 VM 事件、HAL 定时器置位等须配合 uc_emu_stop 唤醒（见 vmEvent.c / main.c）。
 */
#ifndef MORAL_EMU_UNLIMITED_SLICE
#define MORAL_EMU_UNLIMITED_SLICE 1
#endif

#ifndef MORAL_PERF_STATS_INTERVAL_MS
#define MORAL_PERF_STATS_INTERVAL_MS 0
#endif

#ifndef MORAL_LOG_HOT_PATH
#define MORAL_LOG_HOT_PATH 0
#endif

/*
 * 0x2D5ECA uart_print 钩子最多打印次数（static 计数）；早期 NAND/KER 会占满，后面的 DrvLcdConfig 等 printf 会被吃掉。
 * 设为 0 表示不限制（可能极刷屏）。
 */
#ifndef MORAL_UART_HOOK_PRINT_MAX
#define MORAL_UART_HOOK_PRINT_MAX 512u
#endif

/* KER_VTRACE / ker_trace 代码钩子：默认关闭，避免固件刷屏时同步控制台 I/O 拖垮模拟 */
#ifndef MORAL_LOG_KERNEL_TRACE
#define MORAL_LOG_KERNEL_TRACE 0
#endif

/* 指令级补丁调试：如 0xEE9DC 软执行 LDRH、INSN_INVALID 等；默认关，避免控制台刷屏 */
#ifndef MORAL_LOG_UC_CODE_PATCH
#define MORAL_LOG_UC_CODE_PATCH 0
#endif

/*
 * 观测 sysIsCdcFastInitMode() 的返回值（实为 (u8)bIsCdcFastInitMode）：
 * 在 IDA 中打开函数，首条指令地址填入 MORAL_HOOK_SYS_IS_CDC_PC（Thumb 可用奇/偶 PC，内部会规范到偶地址）。
 * 在 IDA 中对 bIsCdcFastInitMode 按「G」或变量声明看地址，填入 MORAL_B_IS_CDC_FAST_INIT_GVA（与固件加载基址一致，通常 0 基）。
 * MORAL_HOOK_SYS_IS_CDC_PC 为 0 时不注册钩子。
 */
#ifndef MORAL_HOOK_SYS_IS_CDC_PC
#define MORAL_HOOK_SYS_IS_CDC_PC 0x127538u /* IDA：sysIsCdcFastInitMode 首址 */
#endif
#ifndef MORAL_B_IS_CDC_FAST_INIT_GVA
#define MORAL_B_IS_CDC_FAST_INIT_GVA 0x00D01DB0u /* IDA XRAM: bIsCdcFastInitMode DCB 0 */
#endif
/* Thumb 首条 16 位指令填 2；若为 ARM 首条 32 位则填 4 */
#ifndef MORAL_HOOK_SYS_IS_CDC_BYTES
#define MORAL_HOOK_SYS_IS_CDC_BYTES 2u
#endif

/*
 * vsprintf（IDA 首址）：R0=目标缓冲，R1=格式串，R2=va_list（GCC ARM 下多为指向 __ap 的结构）。
 * 为 0 时不挂接。MORAL_VSPRINTF_HOOK_LOG_MAX：最多打印次数，0=不限制。
 */
#ifndef MORAL_HOOK_VSPRINTF_PC
#define MORAL_HOOK_VSPRINTF_PC 0x303640u
#endif
#ifndef MORAL_HOOK_VSPRINTF_BYTES
#define MORAL_HOOK_VSPRINTF_BYTES 2u
#endif
#ifndef MORAL_VSPRINTF_HOOK_LOG_MAX
#define MORAL_VSPRINTF_HOOK_LOG_MAX 512u
#endif

#ifndef MORAL_LOG_SD_IO
#define MORAL_LOG_SD_IO 0
#endif

#ifndef MORAL_LOG_TOUCH_DEBUG
#define MORAL_LOG_TOUCH_DEBUG 0
#endif

#ifndef MORAL_LOG_SIM_IO
#define MORAL_LOG_SIM_IO 0
#endif

#ifndef MORAL_SD_WRITE_FLUSH_EVERY
#define MORAL_SD_WRITE_FLUSH_EVERY 16
#endif

/*
 * CBFS / vmio：drive 2、4 时固件走 /NAND/ 或 /CARD/ + ven_file_wfopen。
 * 将此类打开重定向到宿主机目录（与 moral-i9.bin 并列，便于放入 .system 资源）。
 * 地址来自 IDA（vmio.c）：若换固件需重对符号。
 */
#ifndef MORAL_CBFS_HOST_ENABLE
#define MORAL_CBFS_HOST_ENABLE 1
#endif
#ifndef MORAL_CBFS_VM_FILE_OPEN_PC
#define MORAL_CBFS_VM_FILE_OPEN_PC 0x004E04CCu
#endif
#ifndef MORAL_CBFS_VM_FILE_CLOSE_PC
#define MORAL_CBFS_VM_FILE_CLOSE_PC 0x004E05AAu
#endif
#ifndef MORAL_CBFS_VM_FILE_READ_PC
#define MORAL_CBFS_VM_FILE_READ_PC 0x004E083Eu
#endif
#ifndef MORAL_CBFS_VM_FILE_WRITE_PC
#define MORAL_CBFS_VM_FILE_WRITE_PC 0x004E08C2u
#endif
#ifndef MORAL_CBFS_VM_FILE_SEEK_PC
#define MORAL_CBFS_VM_FILE_SEEK_PC 0x004E0944u
#endif
#ifndef MORAL_CBFS_VM_FILE_TELL_PC
#define MORAL_CBFS_VM_FILE_TELL_PC 0x004E0976u
#endif
#ifndef MORAL_CBFS_VM_FILE_GETSIZE_PC
#define MORAL_CBFS_VM_FILE_GETSIZE_PC 0x004E0996u
#endif
#ifndef MORAL_CBFS_FILE_POOL_GVA
#define MORAL_CBFS_FILE_POOL_GVA 0x00EF84B0u
#endif
#ifndef MORAL_CBFS_CURR_RUN_DEV_GVA
#define MORAL_CBFS_CURR_RUN_DEV_GVA 0x00D1BE34u /* CurrRunDevType DCB */
#endif
#ifndef MORAL_CBFS_HOST_SUBDIR_NAND
#define MORAL_CBFS_HOST_SUBDIR_NAND "NAND\\"
#endif
#ifndef MORAL_CBFS_HOST_SUBDIR_CARD
#define MORAL_CBFS_HOST_SUBDIR_CARD "CARD\\"
#endif

#define EXT_RAM_ADDRESS 0x0000000

#define INT_RAM_ADDRESS 0x40000000