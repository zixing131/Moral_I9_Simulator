#include "main.h"

int unkRegPx = 0;
void handleCutsomerReg(uint64_t address, u32 data, uint64_t value)
{
}

void handleCutsomerRomHook(uint64_t address)
{

}

/**
 *
 *
 *
 *
 *
    case 0x4000BB88:
        uc_mem_read(MTK, 0x4000BB88, &changeTmp, 2);
        printf("read 0x4000BB88 %x\n", changeTmp);
        break;
    case 0x80010034:
        if (data == 0)
        {
            uc_mem_read(MTK, 0x80010034, &changeTmp, 4);
            if (changeTmp > 0)
                changeTmp = 0;
            else
                changeTmp = 1;
            uc_mem_write(MTK, 0x80010034, &changeTmp, 4);
        }
        break;
    case 0x80010714:
        if (data == 0)
        {
            uc_mem_read(MTK, 0x80010714, &changeTmp, 4);
            if (unkRegPx == 0x3383)
            {
                changeTmp1 = 0x2000;
            }
            else if (unkRegPx == 0x3080)
            {
                if (changeTmp1 != 0x1000)
                    changeTmp1 = 0x1000;
                else
                    changeTmp1 = 0x4000;
            }
            else if (unkRegPx == 0x3783)
            {
                changeTmp1 = 0;
            }
            uc_mem_write(MTK, 0x80010714, &changeTmp1, 4);
        }
        break;
    case 0x80010118:
        if (data == 1)
        {
            unkRegPx = value;
            printf("[write 0x80010118 %x]\n", value);
        }
        break;
*/