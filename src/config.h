// 定时中断配置，单位毫秒
#define Timer_Interrupt_Duration 1000
// SD卡镜像文件配置
#define SD_CARD_IMG_PATH "Rom\\fat32.img"
// Rom Flash文件配置
#define FLASH_IMG_PATH "Rom\\flash.img"
// Rom Flash Temp文件配置
#define FLASH_IMG_TEMP_PATH "Rom\\flash.img.tmp"
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
#define MORAL_LCD_PERIODIC_REFRESH_MS 33
#endif

#define EXT_RAM_ADDRESS 0x0000000

#define INT_RAM_ADDRESS 0x40000000