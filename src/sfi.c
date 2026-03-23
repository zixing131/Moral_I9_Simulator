#include "main.h"

#define SFI_base 0x810a0000

// 这个目前需要在固件中查找地址
#define FS_Base_Addr 0x730000
// SFI registers
#define RW_SFI_MAC_CTL (SFI_base + 0x0000)
#define RW_SFI_DIRECT_CTL (SFI_base + 0x0004) // out_length reg
#define RW_SFI_MISC_CTL (SFI_base + 0x0008)   // input length reg
#define RW_SFI_MISC_CTL2 (SFI_base + 0x000C)
#define RW_SFI_MAC_OUTL (SFI_base + 0x0010)
#define RW_SFI_MAC_INL (SFI_base + 0x0014)
#define RW_SFI_RESET_CTL (SFI_base + 0x0018)
#define RW_SFI_STA2_CTL (SFI_base + 0x001C)
#define RW_SFI_DLY_CTL1 (SFI_base + 0x0020)
#define RW_SFI_DLY_CTL2 (SFI_base + 0x0024)
#define RW_SFI_DLY_CTL3 (SFI_base + 0x0028) // busy reg
#define RW_SFI_DLY_CTL4 (SFI_base + 0x0030)
#define RW_SFI_DLY_CTL5 (SFI_base + 0x0034)
#define RW_SFI_DLY_CTL6 (SFI_base + 0x0038)
#define RW_SFI_DIRECT_CTL2 (SFI_base + 0x0040)
#define RW_SFI_MISC_CTL3 (SFI_base + 0x0044)
#define RW_SFI_GPRAM_DATA (SFI_base + 0x0800)

#define RW_SFI_GPRAM_BUSY_REG 0x83010a28 // Flash忙寄存器
#define RW_SFI_MAC_CTL (SFI_base + 0x0000)
#define RW_SFI_GPRAM_DATA (SFI_base + 0x0800)
#define RW_SFI_GPRAM_CMD_REG (SFI_base + 0x0800)  // 要发送的命令寄存器
#define RW_SFI_GPRAM_DATA_REG (SFI_base + 0x0804) // 要发送的数据寄存器?

#define RW_SFI_OUTPUT_LEN_REG (SFI_base + 0x0004) // 向Flash中写入多少字节
#define RW_SFI_INPUT_LEN_REG (SFI_base + 0x0008)  // 从Flash中读取多少字节

struct SF_Control
{
    u8 cmd;
    u32 address;
    u8 cmdRev; // 1 = 命令已接收
    u32 sendDataCount;
    u32 readDataCount;
    u32 cacheData[64];
    u8 write_enable;
};

struct SF_Control SF_C_Frame;
u8 Flash_Erase_Page[256];

void handleSfiReg(uint64_t address, u32 data, uint64_t value)
{
    switch (address)
    {
    case RW_SFI_OUTPUT_LEN_REG: // 要写入的数据长度
        if (data == 1)
        {
            SF_C_Frame.sendDataCount = value;
        }
        break;
    case RW_SFI_INPUT_LEN_REG: // 要读取的数据长度
        if (data == 1)
        {
            SF_C_Frame.readDataCount = value;
        }
        break;
    case RW_SFI_GPRAM_CMD_REG: // FLash数据寄存器
    {
        if (data == 1) // 写入数据
        {
            SF_C_Frame.cmd = value & 0xff;
            SF_C_Frame.address = (value >> 24) | (((value >> 16) & 0xff) << 8) | (((value >> 8) & 0xff) << 16); // 分别是原前8位，中8位，高8位
        }
        break;
    }

    case RW_SFI_MAC_CTL: // Flash控制寄存器
    {
        if (data == 1)
        {
            if ((value & 0xC) != 0)
            {
                SF_C_Frame.cmdRev = 1;
            }
        }
        else
        {
            if (SF_C_Frame.cmdRev)
            {
                switch (SF_C_Frame.cmd)
                {
                case 0x2:                                      // 写一页数据
                    changeTmp1 = SF_C_Frame.sendDataCount - 4; // 减去1cmd 3addr就是实际写入长度
                    changeTmp = ROM_ADDRESS | SF_C_Frame.address;
                    uc_mem_write(MTK, changeTmp, SF_C_Frame.cacheData, changeTmp1);
                    break;
                case 0x5:          // 读状态寄存器
                    changeTmp = 0; // 表示不忙
                    uc_mem_write(MTK, RW_SFI_GPRAM_CMD_REG, &changeTmp, 4);
                    break;
                case 0x1:  // 写状态寄存器
                case 0x6:  // 允许写入
                case 0xb9: // 切换到正常模式
                case 0x50: // 未知
                case 0x38: // 未知
                    break;
                case 0xaf: // SF_CMD_READ_ID_QPI
                case 0x9f: // 读取三字节Flash ID信息
                    uc_mem_write(MTK, RW_SFI_GPRAM_CMD_REG, &NorFlashID, 4);
                    break;
                default:
                    // printf("未处理的SPI FLASH命令(%x)\n", SF_C_Frame.cmd);

                    if ((NorFlashID == 0x3725c203 && SF_C_Frame.cmd == 0x52) || (NorFlashID == 0x3825c203 && SF_C_Frame.cmd == 0xd8)) // 不同型号的flash页擦除命令不一样
                    {
                        changeTmp = ROM_ADDRESS | SF_C_Frame.address;
                        my_memset(Flash_Erase_Page, 0xff, 256);
                        uc_mem_write(MTK, changeTmp, Flash_Erase_Page, 256);
                    }

                    break;
                }
                changeTmp = 2;
                uc_mem_write(MTK, RW_SFI_MAC_CTL, &changeTmp, 4);
                SF_C_Frame.cmdRev = 0;
            }
        }
        break;
    }
    default:
        if (address >= RW_SFI_GPRAM_DATA_REG && address <= RW_SFI_GPRAM_DATA_REG + 256) // 假设缓存256字节
        {
            if (data == 1)
            {
                u32 off = address - RW_SFI_GPRAM_DATA_REG;
                off /= 4;
                SF_C_Frame.cacheData[off] = value;
                // printf("Write Flash CacheData(off:%x,value:%x)\n", off, value);
            }
        }
        break;
    }
}
