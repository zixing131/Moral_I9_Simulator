#include "msdc.h"

u32 SEND_SDCMD_CACHE; // SD命令缓存
u32 SDCMD_CACHE;
u32 SEND_SDDATA_CACHE; // SD数据缓存
u32 SD_READ_ADDR;      // SD文件系统读取地址
u32 SD_Write_ADDR;     // SD文件系统写入地址
u8 *msdcDataPtr;

void handleMsdcReg(uint64_t addr, u32 data, uint64_t value)
{
    u32 tmp;
    switch (addr)
    {
    case SD_ARG_REG:
    {
        if (data == 1)
        {
            vm_dma_msdc_config.MSDC_DATA_ADDR = value;
        }
        break;
    }
    case SD_CMD_STAT_REG: // 读取SD 命令状态寄存器
    {
        tmp = 1;
        uc_mem_write(MTK, SD_CMD_STAT_REG, &tmp, 4); // 写1表示命令回复成功 2超时 4crc校验错误
        break;
    }
    case SD_DATA_RESP_REG0:
    { // SD 命令响应数据寄存器 r0,r1,r2,r3每个寄存器占用4字节
        switch (SDCMD_CACHE)
        {
        case SDC_CMD_CMD0: // 进入SPI模式
            // printf("SD卡 进入SPI模式(%x)\n", SEND_SDDATA_CACHE);
            break;
        case SDC_CMD_CMD1:
            break;
        case SDC_CMD_CMD2: // 用于请求 SD 卡返回 CID (Card Identification Number)数据(128位响应)
            // printf("SD卡 获取CID寄存器(%x)\n", SEND_SDDATA_CACHE);
            tmp = 0xF016C1C4;
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD7: // 用于选择或取消选择一张 SD 卡
            // printf("取消或选择SD卡(%x)\n", SEND_SDDATA_CACHE);
            break;
        case SDC_CMD_CMD8: // 询问SD卡的版本号和电压范围
            tmp = 0x1aa;
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD9: // 获取SD卡的CSD寄存器（Card-Specific Data Register）(128位响应)
            // printf("SD卡 获取CSD寄存器(%x)\n", SEND_SDDATA_CACHE);
            //  changeTmp1 = 0x400E0032;//原始数据
            tmp = 0x0000e004; // int*转换到char*
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD55: // 用于通知SD卡，下一个命令将是应用命令（ACMD）
            // printf("SD卡ACMD模式开启(%x)\n", SEND_SDDATA_CACHE);
            tmp = 0x20;
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD41_SD: // 初始化SD命令 (ACMD41 OCR)
            // bit31=1 上电完成；bit30=1 CCS，典型 SDHC/SDXC 镜像；bit23-8 电压窗口
            tmp = 0xC0FF8000;
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD3_SD: // SEND_RELATIVE_ADDR (RCA)在 SD 卡的初始化过程中为卡分配一个相对地址
            // printf("SD卡 分配相对地址(%x)\n", SEND_SDDATA_CACHE);
            tmp = 0x3001;
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD12: // 结束连续多数据块传输
            // printf("结束SD卡连续读\n");
            break;
        case SDC_CMD_CMD13: // 查询 SD 卡的状态，并返回卡的当前状态信息
            // printf("SD卡 查询SD卡状态(%x)\n", SEND_SDDATA_CACHE);
            // 0x100 = R1_READY_FOR_DATA_8
            tmp = 0x100;
            uc_mem_write(MTK, SD_DATA_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD16: // 该命令用于设置数据块的长度
            // printf("SD卡 设置SD数据块长度(%x)\n", SEND_SDDATA_CACHE);
            // DMA_Transfer_Bytes_Count = SEND_SDDATA_CACHE;
            break;
        case SDC_CMD_CMD18: // 读取多个数据块
        case SDC_CMD_CMD17: // 读取单个数据块
            if (vm_dma_msdc_config.config_finish == 1)
            {
                msdcDataPtr = readSDFile(vm_dma_msdc_config.MSDC_DATA_ADDR, vm_dma_msdc_config.transfer_count);
                if (msdcDataPtr != NULL)
                {
                    uc_mem_write(MTK, vm_dma_msdc_config.data_addr, msdcDataPtr, vm_dma_msdc_config.transfer_count);
                    SDL_free(msdcDataPtr);
                }
                vm_dma_msdc_config.config_finish = 0;
                changeTmp = 0x8000;
                uc_mem_write(MTK, SDC_DATSTA_REG, &changeTmp, 4);
                if (vm_dma_msdc_config.transfer_end_interrupt_enable == 1)
                {
                    vm_dma_msdc_config.transfer_end_interrupt_enable = 0;
                    EnqueueVMEvent(VM_EVENT_MSDC_IRQ, 0, 0);
                }
            }
            break;
        case SDC_CMD_CMD25: // 写多个数据块
        case SDC_CMD_CMD24: // 写单个数据块
            if (vm_dma_msdc_config.config_finish == 1)
            {
                vm_dma_msdc_config.config_finish = 0;
                uc_mem_read(MTK, vm_dma_msdc_config.data_addr, vm_dma_msdc_config.cacheBuffer, vm_dma_msdc_config.transfer_count);
                writeSDFile(vm_dma_msdc_config.cacheBuffer, vm_dma_msdc_config.MSDC_DATA_ADDR, vm_dma_msdc_config.transfer_count);
                changeTmp = 0x8000;
                uc_mem_write(MTK, SDC_DATSTA_REG, &changeTmp, 4);
                if (vm_dma_msdc_config.transfer_end_interrupt_enable == 1)
                {
                    vm_dma_msdc_config.transfer_end_interrupt_enable = 0;
                    EnqueueVMEvent(VM_EVENT_MSDC_IRQ, 0, 0);
                }
            }
            break;
        case SDC_CMD_ACMD42: // 卡检测信号通常用于检测 SD 卡是否插入或取出
            // printf("SD卡 检查是否插入或取出(%x)\n", SEND_SDDATA_CACHE);
            break;
        case SDC_CMD_ACMD51: // 请求 SD 卡返回其 SCR (SD Card Configuration Register)寄存器
            // printf("SD卡 读取SCR寄存器(%x)\n", SEND_SDCMD_CACHE);
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_0(%x,CMD:%x)", SEND_SDDATA_CACHE, SEND_SDCMD_CACHE);
            // printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_DATA_RESP_REG1:
    {
        switch (SDCMD_CACHE)
        {
        case SDC_CMD_CMD2: // 返回CID寄存器
            tmp = 0x77;
            uc_mem_write(MTK, SD_DATA_RESP_REG1, &tmp, 4);
            break;
        case SDC_CMD_CMD9: // 返回CSD寄存器
            // changeTmp1 = 0x77590000;
            tmp = 0x000ff577; // int*转换到char*
            uc_mem_write(MTK, SD_DATA_RESP_REG1, &tmp, 4);
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_1(CMD:%x)", SEND_SDCMD_CACHE);
            // printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_DATA_RESP_REG2:
    {
        switch (SDCMD_CACHE)
        {
        case SDC_CMD_CMD2: // 返回CID寄存器
            tmp = 0;
            uc_mem_write(MTK, SD_DATA_RESP_REG2, &tmp, 4);
            break;
        case SDC_CMD_CMD9: // 返回CSD寄存器
            // changeTmp1 = 0x7FF09000;
            tmp = 0x00090ff7; // int*转换到char*
            uc_mem_write(MTK, SD_DATA_RESP_REG2, &tmp, 4);
            break;
        case SDC_CMD_CMD17: //?
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_2(CMD:%x)", SEND_SDCMD_CACHE);
            // printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_DATA_RESP_REG3:
    {

        switch (SDCMD_CACHE)
        {
        case SDC_CMD_CMD2: // 返回CID寄存器
            tmp = 0x3;
            uc_mem_write(MTK, SD_DATA_RESP_REG3, &tmp, 4);
            break;
        case SDC_CMD_CMD9: // 返回CSD寄存器
            // changeTmp1 = 0x0A400000;
            tmp = 0x000004a0; // int*转换到char*
            uc_mem_write(MTK, SD_DATA_RESP_REG3, &tmp, 4);
            break;
        case SDC_CMD_ACMD51: // 读取SCR寄存器
            tmp = 0;
            uc_mem_write(MTK, SD_DATA_RESP_REG3, &tmp, 4);
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_3(CMD:%x)", SEND_SDCMD_CACHE);
            //  printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_CMD_RESP_REG0:
    {
        switch (SDCMD_CACHE)
        {
        case 0:
            break;
        case SDC_CMD_CMD2: // CID响应
            tmp = 0xF016C1C4;
            uc_mem_write(MTK, SD_CMD_RESP_REG0, &tmp, 4);
            break;
        case SDC_CMD_CMD7:
            break;
        case SDC_CMD_CMD13: //?
            break;
        case SDC_CMD_CMD16: //?
            break;
        case SDC_CMD_CMD17: //?
            break;
        case SDC_CMD_CMD18: //?
            break;
        case SDC_CMD_CMD24: //?
            break;
        case SDC_CMD_ACMD42:
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_0(ACMD:%x)", SEND_SDCMD_CACHE);
            //  printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_CMD_RESP_REG1:
    {
        switch (SDCMD_CACHE)
        {
        case SDC_CMD_CMD2:
            tmp = 0x77;
            uc_mem_write(MTK, SD_CMD_RESP_REG1, &tmp, 4);
            break;
        case SDC_CMD_CMD13:
            break;
        case SDC_CMD_CMD16:
            break;
        case SDC_CMD_CMD17:
            break;
        case SDC_CMD_CMD18:
            break;
        case SDC_CMD_CMD55:
            break;
        case SDC_CMD_ACMD51:
            break;
        default:
            tmp = 0;
            uc_mem_write(MTK, SD_CMD_RESP_REG1, &tmp, 4);
            //  printf("未处理SD_DATA_RESP_REG_1(ACMD:%x)", SEND_SDCMD_CACHE);
            //  printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_CMD_RESP_REG2:
    {
        switch (SDCMD_CACHE)
        {
        case 0:
            break;
        case SDC_CMD_CMD2:
            tmp = 0;
            uc_mem_write(MTK, SD_CMD_RESP_REG2, &tmp, 4);
            break;
        case SDC_CMD_CMD3_SD:
            break;
        case SDC_CMD_CMD7:
            break;
        case SDC_CMD_CMD8:
            break;
        case SDC_CMD_CMD9:
            break;
        case SDC_CMD_CMD12:
            break;
        case SDC_CMD_CMD13: // 查询 SD 卡的状态，并返回卡的当前状态信息
            break;
        case SDC_CMD_CMD17:
            break;
        case SDC_CMD_CMD18:
            break;
        case SDC_CMD_CMD24:
            break;
        case 0x90:
            break;
        case SDC_CMD_CMD55:
            break;
        case SDC_CMD_ACMD51:
            break;
        case 0x40000000:
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_2(ACMD:%x)", SEND_SDCMD_CACHE);
            //  printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }
    case SD_CMD_RESP_REG3:
    {
        switch (SDCMD_CACHE)
        {
        case SDC_CMD_CMD2:
            tmp = 0x3;
            uc_mem_write(MTK, SD_CMD_RESP_REG3, &tmp, 4);
            break;
        case 0x8b3:
            break;
        default:
            // printf("未处理SD_DATA_RESP_REG_3(ACMD:%x)", SEND_SDCMD_CACHE);
            // printf("(%x)\n", lastAddress);
            break;
        }
        break;
    }

    case SD_CMD_REG: // SD 命令寄存器
    {
        if (data == 1)
        {
            SEND_SDCMD_CACHE = value;
            // 取0x40000000后16位
            SDCMD_CACHE = value & 0xffff;
        }
        break;
    }

    case DMA_MSDC_CONTROL_REG:
        if (data == 1)
        {
            vm_dma_msdc_config.control = value;
            vm_dma_msdc_config.chanel = (value >> 20) & 0b11111;
            vm_dma_msdc_config.direction = (value >> 18) & 1;
            vm_dma_msdc_config.align = value & 0b11;
            vm_dma_msdc_config.transfer_end_interrupt_enable = (value >> 15) & 1;
        }
        break;
    case DMA_MSDC_DATA_ADDR_REG:
        if (data == 1)
            vm_dma_msdc_config.data_addr = value;
        break;
    case DMA_MSDC_TRANSFER_COUNT_REG:
        if (data == 1)
        {
            if (vm_dma_msdc_config.align == DMA_DATA_BYTE_ALIGN_FOUR)
                value *= 4;
            if (vm_dma_msdc_config.align == DMA_DATA_BYTE_ALIGN_TWO)
                value *= 2;
            vm_dma_msdc_config.transfer_count = value;
        }
        break;
    case DMA_MSDC_START_REG:
        // 写入0x8000表示DMA开始运行
        if (data == 1)
        {
            if (value == 0x8000)
            {
                if (vm_dma_msdc_config.chanel == MSDC)
                {
                    vm_dma_msdc_config.config_finish = 1;
                }
                else
                {
                    printf("unhandle msdc dma chanel[%x]\n", vm_dma_msdc_config.chanel);
                }
            }
        }
    }
}
