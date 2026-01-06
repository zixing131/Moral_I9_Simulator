#include "main.h"

void handleSecureEngineReadWrite(uc_engine *uc, uc_mem_type type, uint64_t address, uint32_t size, int64_t value)
{
    switch (address)
    {

    case 0x810C0090: // 读寄存器，返回0x10过sub_8122d8c的while
    {
        if (type == UC_MEM_READ)
        {
            changeTmp = 0x10;
            uc_mem_write(MTK, (u32)address, &changeTmp, 4);
        }
        break;
    }
    }
}