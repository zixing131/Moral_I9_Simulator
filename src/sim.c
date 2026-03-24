#include "sim.h"

clock_t last_sim_interrupt_time;

int simDataTransferCount = -1;
VM_SIM_DEV vm_sim1_dev;
VM_SIM_DEV vm_sim2_dev;
VM_DMA_CONFIG vm_dma_sim1_config;
VM_DMA_CONFIG vm_dma_sim2_config;

static void sim_write_count_reg(SIM_CARD_NUM sim_num, u32 count)
{
    uc_mem_write(MTK, sim_num == SIM_CARD_NUM_CARD1 ? SIM1_COUNT : SIM2_COUNT, &count, 4);
}

static void sim_load_rx_fifo(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, const u8 *data, u32 len)
{
    if (len > sizeof(sim_dev->rx_buffer))
        len = sizeof(sim_dev->rx_buffer);

    if (len > 0)
        my_memcpy(sim_dev->rx_buffer, (void *)data, len);

    sim_dev->rx_buffer_index = 0;
    sim_dev->rx_current_index = 0;
    sim_dev->rx_remain_count = (u8)((len > 0xffu) ? 0xffu : len);
    sim_write_count_reg(sim_num, sim_dev->rx_remain_count);
}

static void sim_get_selected_file_payload(VM_SIM_DEV *sim_dev, u8 **out_ptr, u32 *out_len)
{
    *out_ptr = SIM_RSP_SF_FFFF;
    *out_len = sizeof(SIM_RSP_SF_FFFF) / sizeof(SIM_RSP_SF_FFFF[0]);

    switch (sim_dev->selected_file_id)
    {
    case 0x2FE2:
        *out_ptr = SIM_RSP_SF_2FE2;
        *out_len = sizeof(SIM_RSP_SF_2FE2) / sizeof(SIM_RSP_SF_2FE2[0]);
        break;
    case 0x2F05:
        *out_ptr = SIM_RSP_SF_2F05;
        *out_len = sizeof(SIM_RSP_SF_2F05) / sizeof(SIM_RSP_SF_2F05[0]);
        break;
    case 0x6F05:
        *out_ptr = SIM_RSP_SF_6F05;
        *out_len = sizeof(SIM_RSP_SF_6F05) / sizeof(SIM_RSP_SF_6F05[0]);
        break;
    case 0x6F07:
        *out_ptr = SIM_RSP_SF_6F07;
        *out_len = sizeof(SIM_RSP_SF_6F07) / sizeof(SIM_RSP_SF_6F07[0]);
        break;
    case 0x6F14:
        *out_ptr = SIM_RSP_SF_6F14;
        *out_len = sizeof(SIM_RSP_SF_6F14) / sizeof(SIM_RSP_SF_6F14[0]);
        break;
    case 0x6F17:
        *out_ptr = SIM_RSP_SF_6F17;
        *out_len = sizeof(SIM_RSP_SF_6F17) / sizeof(SIM_RSP_SF_6F17[0]);
        break;
    case 0x6F20:
        *out_ptr = SIM_RSP_SF_6F20;
        *out_len = sizeof(SIM_RSP_SF_6F20) / sizeof(SIM_RSP_SF_6F20[0]);
        break;
    case 0x6F30:
        *out_ptr = SIM_RSP_SF_6F30;
        *out_len = sizeof(SIM_RSP_SF_6F30) / sizeof(SIM_RSP_SF_6F30[0]);
        break;
    case 0x6F31:
        *out_ptr = SIM_RSP_SF_6F31;
        *out_len = sizeof(SIM_RSP_SF_6F31) / sizeof(SIM_RSP_SF_6F31[0]);
        break;
    case 0x6F38:
        *out_ptr = SIM_RSP_SF_6F38;
        *out_len = sizeof(SIM_RSP_SF_6F38) / sizeof(SIM_RSP_SF_6F38[0]);
        break;
    case 0x6F40:
        *out_ptr = SIM_RSP_SF_6F40;
        *out_len = sizeof(SIM_RSP_SF_6F40) / sizeof(SIM_RSP_SF_6F40[0]);
        break;
    case 0x6F41:
        *out_ptr = SIM_RSP_SF_6F41;
        *out_len = sizeof(SIM_RSP_SF_6F41) / sizeof(SIM_RSP_SF_6F41[0]);
        break;
    case 0x6F42:
        *out_ptr = SIM_RSP_SF_6F42;
        *out_len = sizeof(SIM_RSP_SF_6F42) / sizeof(SIM_RSP_SF_6F42[0]);
        break;
    case 0x6F43:
        *out_ptr = SIM_RSP_SF_6F43;
        *out_len = sizeof(SIM_RSP_SF_6F43) / sizeof(SIM_RSP_SF_6F43[0]);
        break;
    case 0x6F44:
        *out_ptr = SIM_RSP_SF_6F44;
        *out_len = sizeof(SIM_RSP_SF_6F44) / sizeof(SIM_RSP_SF_6F44[0]);
        break;
    case 0x6F45:
        *out_ptr = SIM_RSP_SF_6F45;
        *out_len = sizeof(SIM_RSP_SF_6F45) / sizeof(SIM_RSP_SF_6F45[0]);
        break;
    case 0x6F46:
        *out_ptr = SIM_RSP_SF_6F46;
        *out_len = sizeof(SIM_RSP_SF_6F46) / sizeof(SIM_RSP_SF_6F46[0]);
        break;
    case 0x6F4A:
        *out_ptr = SIM_RSP_SF_6F4A;
        *out_len = sizeof(SIM_RSP_SF_6F4A) / sizeof(SIM_RSP_SF_6F4A[0]);
        break;
    case 0x6F4B:
        *out_ptr = SIM_RSP_SF_6F4B;
        *out_len = sizeof(SIM_RSP_SF_6F4B) / sizeof(SIM_RSP_SF_6F4B[0]);
        break;
    case 0x6F52:
        *out_ptr = SIM_RSP_SF_6F52;
        *out_len = sizeof(SIM_RSP_SF_6F52) / sizeof(SIM_RSP_SF_6F52[0]);
        break;
    case 0x6F53:
        *out_ptr = SIM_RSP_SF_6F53;
        *out_len = sizeof(SIM_RSP_SF_6F53) / sizeof(SIM_RSP_SF_6F53[0]);
        break;
    case 0x6F62:
        *out_ptr = SIM_RSP_SF_6F62;
        *out_len = sizeof(SIM_RSP_SF_6F62) / sizeof(SIM_RSP_SF_6F62[0]);
        break;
    case 0x6F74:
        *out_ptr = SIM_RSP_SF_6F74;
        *out_len = sizeof(SIM_RSP_SF_6F74) / sizeof(SIM_RSP_SF_6F74[0]);
        break;
    case 0x6F78:
        *out_ptr = SIM_RSP_SF_6F78;
        *out_len = sizeof(SIM_RSP_SF_6F78) / sizeof(SIM_RSP_SF_6F78[0]);
        break;
    case 0x6F7B:
        *out_ptr = SIM_RSP_SF_6F7B;
        *out_len = sizeof(SIM_RSP_SF_6F7B) / sizeof(SIM_RSP_SF_6F7B[0]);
        break;
    case 0x6F7E:
        *out_ptr = SIM_RSP_SF_6F7E;
        *out_len = sizeof(SIM_RSP_SF_6F7E) / sizeof(SIM_RSP_SF_6F7E[0]);
        break;
    case 0x6FAD:
        *out_ptr = SIM_RSP_SF_6FAD;
        *out_len = sizeof(SIM_RSP_SF_6FAD) / sizeof(SIM_RSP_SF_6FAD[0]);
        break;
    case 0x6FAE:
        *out_ptr = SIM_RSP_SF_6FAE;
        *out_len = sizeof(SIM_RSP_SF_6FAE) / sizeof(SIM_RSP_SF_6FAE[0]);
        break;
    case 0x6FC5:
        *out_ptr = SIM_RSP_SF_0000;
        *out_len = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        break;
    case 0x6FC6:
        *out_ptr = SIM_RSP_SF_6FC6;
        *out_len = sizeof(SIM_RSP_SF_6FC6) / sizeof(SIM_RSP_SF_6FC6[0]);
        break;
    case 0x6FCD:
        *out_ptr = SIM_RSP_SF_6FCD;
        *out_len = sizeof(SIM_RSP_SF_6FCD) / sizeof(SIM_RSP_SF_6FCD[0]);
        break;
    }
}

void SIM_TIDE_HANDLE(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, int64_t value)
{
    sim_dev->rx_trigger_count = (value & 0xf) + 1;
    sim_dev->tx_trigger_count = ((value >> 16) & 0xf) + 1;
    switch (sim_dev->event)
    {
    case SIM_DEV_EVENT_NONE:
        changeTmp2 = sizeof(SIM_ATR_RSP_DATA) / sizeof(SIM_ATR_RSP_DATA[0]);
        sim_load_rx_fifo(sim_dev, sim_num, SIM_ATR_RSP_DATA, changeTmp2);
        sim_dev->irq_channel = SIM_IRQ_RX;
        sim_dev->irq_start = true;
        sim_dev->event = SIM_DEV_EVENT_CMD;
        break;
    default: // 从这里开始考虑走DMA和不走DMA的情况
        sim_dev->rx_current_index = 0;
        sim_dev->event = SIM_DEV_EVENT_CMD;
        break;
    }
}

void SIM_IRQ_HANDLE(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, int64_t value)
{
    sim_dev->irq_enable = value;
}
void SIM_BASE_HANDLE(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, int64_t value)
{
    sim_dev->control = value;
}
void SIM_DATA_HANDLE(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, u8 isWrite, int64_t value)
{
    if (isWrite == 0)
    {
        if (sim_dev->rx_remain_count > 0)
        {
            changeTmp1 = sim_dev->rx_buffer[sim_dev->rx_buffer_index++];
            sim_dev->rx_current_index++;
            sim_dev->rx_remain_count--;
        }
        else
            changeTmp1 = 0xff;
        if (sim_num == SIM_CARD_NUM_CARD1)
            uc_mem_write(MTK, SIM1_DATA, &changeTmp1, 4);
        else if (sim_num == SIM_CARD_NUM_CARD2)
            uc_mem_write(MTK, SIM2_DATA, &changeTmp1, 4);
        sim_write_count_reg(sim_num, sim_dev->rx_remain_count);
    }
    else
    {
        sim_dev->tx_buffer[sim_dev->t0_tx_count++] = value;
    }
}
// 处理设备向SIM发送的命令
void handle_sim_tx_cmd(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, u32 data_count, u32 dma_data_addr)
{
    int tmp1;
    int tmp2;
    int tmp3;
    int cla; // 协议类型
    int ins; // 命令

    cla = sim_dev->tx_buffer[0];
    ins = sim_dev->tx_buffer[1];

    sim_dev->t0_tx_count = 0;

    uc_mem_read(MTK, dma_data_addr, &sim_dev->T0RxData, data_count);
    // 先发送a0 a4 00 00 02 处理命令
    // 再发送a0 c0 00 00 16 获取响应数据
    simDataTransferCount = -1;
    sim_dev->T0EndRxDataPtr = NULL;
    sim_dev->T0EndRxDataLen = 0;
    // printf("[SIM%d][处理命令][%x%x] %x %x \n", sim_num, cla, ins, sim_dev->T0RxData[0], sim_dev->T0RxData[1]);
    if (cla == 0xa0) // sim卡的命令响应 GSM11.11标准
    {
        if (ins == 0xa4) // select 命令
        {
            handleCmdLogic(sim_dev, sim_num, data_count, 0);
        }
        else if (ins == 0xc0) // get response命令
        {
            handleCmdLogic(sim_dev, sim_num, data_count, 1);
        }
        else if (ins == 0xb0 || ins == 0xb2) // READ BINARY / READ RECORD
        {
            sim_get_selected_file_payload(sim_dev, &sim_dev->T0EndRxDataPtr, &sim_dev->T0EndRxDataLen);
            tmp1 = 0x90;
            tmp2 = 0;
            if (sim_num == SIM_CARD_NUM_CARD1)
            {
                uc_mem_write(MTK, SIM1_SW1_REG, &tmp1, 4);
                uc_mem_write(MTK, SIM1_SW2_REG, &tmp2, 4);
            }
            else if (sim_num == SIM_CARD_NUM_CARD2)
            {
                uc_mem_write(MTK, SIM2_SW1_REG, &tmp1, 4);
                uc_mem_write(MTK, SIM2_SW2_REG, &tmp2, 4);
            }
            sim_dev->irq_channel = SIM_IRQ_T0END;
            sim_dev->irq_start = true;
        }
        else if (ins == 0x10) // 更新记录(update record)
        {
            tmp1 = 0x90;
            tmp2 = 0;
            if (MORAL_LOG_SIM_IO)
                printf("SIM命令:UPDATE RECORD\n");
            if (sim_num == SIM_CARD_NUM_CARD1)
            {
                uc_mem_write(MTK, SIM1_SW1_REG, &tmp1, 4);
                uc_mem_write(MTK, SIM1_SW2_REG, &tmp2, 4);
            }
            else if (sim_num == SIM_CARD_NUM_CARD2)
            {
                uc_mem_write(MTK, SIM2_SW1_REG, &tmp1, 4);
                uc_mem_write(MTK, SIM2_SW2_REG, &tmp2, 4);
            }
            sim_dev->irq_channel = SIM_IRQ_T0END; // 进入中断使接收命令完成，等待设备开启DMA接收响应数据
            sim_dev->irq_start = true;
        }
        else // 未知的命令
        {
            sim_dev->rx_buffer_index = 0;
            sim_dev->rx_current_index = 0;
            tmp1 = 0x90;
            tmp2 = 0;
            if (MORAL_LOG_SIM_IO)
                printf("未知的GSM命令[%x]\n", ins);
            sim_dev->irq_channel = SIM_IRQ_T0END; // 进入中断使接收命令完成，等待设备开启DMA接收响应数据
            sim_dev->irq_start = true;
        }
    }
    else
    {
        // 未知的标准
        sim_dev->rx_buffer_index = 0;
        sim_dev->rx_current_index = 0;
        tmp1 = 0;
        tmp2 = 0;
        if (MORAL_LOG_SIM_IO)
            printf("未知的GSM协议标准[%x]\n", sim_dev->T0RxData[0]);
    }
    // 命令处理完成置0
    sim_dev->tx_buffer_index = 0;
}

void handleCmdLogic(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, u32 data_count, u8 isC0Ins)
{
    u32 tmp1, tmp2;
    u8 *cmd_data = sim_dev->T0RxData;
    u32 cmd_len = data_count;
    u8 selected_file_cmd[2];

    if (isC0Ins && cmd_len < 2 && sim_dev->selected_file_id != 0)
    {
        selected_file_cmd[0] = (u8)(sim_dev->selected_file_id >> 8);
        selected_file_cmd[1] = (u8)(sim_dev->selected_file_id & 0xff);
        cmd_data = selected_file_cmd;
        cmd_len = 2;
    }

    if (my_mem_compare(cmd_data, SIM_CMD_SELECT_7F20, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_7F20) / sizeof(SIM_RSP_SF_7F20[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_7F20;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_7F10, cmd_len))
    {
        tmp1 = 0x90;
        tmp2 = 0;
        // printf("SIM命令:DF_TELECOM\n");
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_2FE2, cmd_len))
    { // ICCID
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_2FE2) / sizeof(SIM_RSP_SF_2FE2[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_2FE2;
        // printf("SIM命令:EF_ICCID\n");
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_3F00, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_3F00) / sizeof(SIM_RSP_SF_3F00[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_3F00;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_2F05, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_2F05) / sizeof(SIM_RSP_SF_2F05[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_2F05;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F05, cmd_len))
    { // Language indication
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F05) / sizeof(SIM_RSP_SF_6F05[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F05;
        // printf("SIM命令:语言偏好\n");
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6FAE, cmd_len))
    { // Phase identification
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6FAE) / sizeof(SIM_RSP_SF_6FAE[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6FAE;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F16, cmd_len))
    {
        tmp1 = 0x90;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F38, cmd_len))
    { // SIM service table
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F38) / sizeof(SIM_RSP_SF_6F38[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F38;
        // printf("SIM命令:EF_SST\n");
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F07, cmd_len))
    { // IMSI
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F07) / sizeof(SIM_RSP_SF_6F07[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F07;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F3E, cmd_len))
    { // GID1
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_0000;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F3F, cmd_len))
    { // GID2
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_0000;
        // printf("SIM命令:6f3f\n");
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F61, cmd_len))
    {
        tmp1 = 0x90;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F62, cmd_len))
    {
        tmp1 = 0x90;
        tmp2 = 0;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6FAD, cmd_len))
    { // Administrative data
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6FAD) / sizeof(SIM_RSP_SF_6FAD[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6FAD;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F78, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F78) / sizeof(SIM_RSP_SF_6F78[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F78;
        // printf("SIM命令:EF_ACC\n");
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F14, cmd_len))
    { // 这个成功后会出现数据账户，包括能进入短信，但无法使用功能
        // EF_CPHS_INFO_ID)CMCC
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F14) / sizeof(SIM_RSP_SF_6F14[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F14;
        // printf("SIM命令:6F14\n");
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F17, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_0000;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F7E, cmd_len))
    { // Location information
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F7E) / sizeof(SIM_RSP_SF_6F7E[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F7E;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F3C, cmd_len))
    { // Short messages
        tmp1 = 0x90;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F52, cmd_len))
    { // EF_KcGPRS
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F52) / sizeof(SIM_RSP_SF_6F52[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F52;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F53, cmd_len))
    { // EF_LOCIGPRS
        tmp1 = 0x94;
        tmp2 = 0x04;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F30, cmd_len))
    { // PLMN selector
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F30) / sizeof(SIM_RSP_SF_6F30[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F30;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F31, cmd_len))
    { // EF_HPLMN 控制是否重选 Home PLMN。
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F31) / sizeof(SIM_RSP_SF_6F31[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F31;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F7B, cmd_len))
    { // EF_FPLMN
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F7B) / sizeof(SIM_RSP_SF_6F7B[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F7B;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F3A, cmd_len))
    { // Abbreviated dialling numbers
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F3A) / sizeof(SIM_RSP_SF_6F3A[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F3A;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F13, cmd_len))
    {
        tmp1 = 0x9e;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6FC6, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6FC6) / sizeof(SIM_RSP_SF_6FC6[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6FC6;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6FCD, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6FCD) / sizeof(SIM_RSP_SF_6FCD[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6FCD;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6FC5, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_0000;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F40, cmd_len))
    { // MSISDN 用户电话号码（显示用，非注册过程必需）。
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F40) / sizeof(SIM_RSP_SF_6F40[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F40;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F44, cmd_len))
    { // EF_LND
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F44) / sizeof(SIM_RSP_SF_6F44[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F44;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F43, cmd_len))
    { // EF_SMSS
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F43) / sizeof(SIM_RSP_SF_6F43[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F43;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F42, cmd_len))
    { // SMS parameters
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F42) / sizeof(SIM_RSP_SF_6F42[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F42;
        // printf("SIM命令:6F42\n");
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F3B, cmd_len))
    { // Fixed dialing numbers
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_0000;
        // printf("SIM命令:6F3B\n");
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F4A, cmd_len))
    { // Extension 1
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F4A) / sizeof(SIM_RSP_SF_6F4A[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F4A;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F4B, cmd_len))
    { // Extension 2
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F4B) / sizeof(SIM_RSP_SF_6F4B[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F4B;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F4C, cmd_len))
    { // Extension 3
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_0000) / sizeof(SIM_RSP_SF_0000[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_0000;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F46, cmd_len))
    { // Service provider name
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F46) / sizeof(SIM_RSP_SF_6F46[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F46;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F74, cmd_len))
    { // BCCH
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F74) / sizeof(SIM_RSP_SF_6F74[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F74;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F20, cmd_len))
    { // EF_Kc
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F20) / sizeof(SIM_RSP_SF_6F20[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F20;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F60, cmd_len))
    { // EF_PLMNwACT_ID
        tmp1 = 0x9e;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F61, cmd_len))
    { // EF_OPLMNwACT_ID 	运营商定义的 PLMN 优先列表（可选）。
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F30) / sizeof(SIM_RSP_SF_6F30[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F30;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6FB7, cmd_len))
    {
        tmp1 = 0x9e;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F49, cmd_len))
    { // Service dialing numbers
        tmp1 = 0x90;
        tmp2 = 0;
    }

    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F41, cmd_len))
    { // EF_PUCT
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F41) / sizeof(SIM_RSP_SF_6F41[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F41;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F19, cmd_len))
    {
        tmp1 = 0x90;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F45, cmd_len))
    {
        tmp1 = 0x9f;
        tmp2 = sizeof(SIM_RSP_SF_6F45) / sizeof(SIM_RSP_SF_6F45[0]);
        sim_dev->T0EndRxDataPtr = SIM_RSP_SF_6F45;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F11, cmd_len))
    {
        tmp1 = 0x9e;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F50, cmd_len))
    {
        tmp1 = 0x9e;
        tmp2 = 0;
    }
    else if (my_mem_compare(cmd_data, SIM_CMD_SELECT_6F48, cmd_len))
    { // EF_CBMID
        tmp1 = 0x9e;
        tmp2 = 0;
        // printf("SIM命令:6F48\n");
    }
    else
    {
        tmp1 = 0x9e;
        tmp2 = 0;
        if (MORAL_LOG_SIM_IO)
            printf("SIM[%d]未处理的SELECT命令[%x %x]\n", sim_num, cmd_data[0], cmd_data[1]);
    }

    if (sim_num == SIM_CARD_NUM_CARD1)
    {
        uc_mem_write(MTK, SIM1_SW1_REG, &tmp1, 4);
        uc_mem_write(MTK, SIM1_SW2_REG, &tmp2, 4);
    }
    else if (sim_num == SIM_CARD_NUM_CARD2)
    {
        uc_mem_write(MTK, SIM2_SW1_REG, &tmp1, 4);
        uc_mem_write(MTK, SIM2_SW2_REG, &tmp2, 4);
    }
    if (!isC0Ins && data_count >= 7)
        sim_dev->selected_file_id = ((u16)sim_dev->T0RxData[data_count - 2] << 8) | sim_dev->T0RxData[data_count - 1];
    sim_dev->T0EndRxDataLen = tmp2;
    sim_dev->irq_channel = SIM_IRQ_T0END; // 进入中断使接收命令完成，等待设备开启DMA接收响应数据
    sim_dev->irq_start = true;
}

// 处理SIM卡向设备发送的数据(DMA数据接收开启)
void handle_sim_rx_cmd(VM_SIM_DEV *sim_dev, SIM_CARD_NUM sim_num, u32 data_count, u32 dma_data_addr)
{
    int tmp = 0;
    int tmp2 = 0;
    int tmp3 = 0;
    if (sim_dev->t0_tx_count > 0)
    {
        sim_dev->rx_buffer_index = 0;
        sim_dev->rx_current_index = 0;

        tmp = 0;
        if (sim_num == SIM_CARD_NUM_CARD1)
            uc_mem_write(MTK, SIM1_IMP3_REG, &tmp, 1);
        else
            uc_mem_write(MTK, SIM2_IMP3_REG, &tmp, 1);

        if (sim_dev->T0EndRxDataPtr != NULL)
        {
            // printf("[sim%d][响应数据] %x %x \n", sim_num,sim_dev->T0RxData[0],sim_dev->T0RxData[1]);
            u32 copy_len = data_count;
            if (sim_dev->T0EndRxDataLen != 0 && copy_len > sim_dev->T0EndRxDataLen)
                copy_len = sim_dev->T0EndRxDataLen;
            uc_mem_write(MTK, dma_data_addr, sim_dev->T0EndRxDataPtr, copy_len);
            if (copy_len < data_count)
            {
                u32 remain = data_count - copy_len;
                if (remain > sizeof(SIM_RSP_SF_FFFF))
                    remain = sizeof(SIM_RSP_SF_FFFF);
                uc_mem_write(MTK, dma_data_addr + copy_len, SIM_RSP_SF_FFFF, remain);
            }
            sim_dev->irq_channel = SIM_IRQ_T0END;
            sim_dev->irq_start = true;
            sim_dev->tx_buffer_index = 0;
        }
        else
        {
            printf("[sim%d][失败响应]没有设置返回数据", sim_num);
            for (tmp = 0; tmp < 2; tmp++)
            {
                printf("%x ", sim_dev->T0RxData[tmp]);
            }
            printf("\n");
        }
    }
}

void handleSimDevReg(uint64_t address, u32 data, uint64_t value)
{
    switch (address)
    {
    case SIM2_COUNT:
        if (data == 0)
            uc_mem_read(MTK, SIM2_COUNT, &value, 4);
        break;
    case SIM2_CONF:
        if (data == 0)
            uc_mem_read(MTK, SIM2_CONF, &value, 4);
        break;
    case SIM1_TIDE:
        if (data == 1)
            SIM_TIDE_HANDLE(&vm_sim1_dev, SIM_CARD_NUM_CARD1, value);
        break;
    case SIM2_TIDE:
        if (data == 1)
            SIM_TIDE_HANDLE(&vm_sim2_dev, SIM_CARD_NUM_CARD2, value);
        break;
    case SIM1_IRQ_ENABLE:
        if (data == 1)
            SIM_IRQ_HANDLE(&vm_sim1_dev, SIM_CARD_NUM_CARD1, value);
        break;
    case SIM2_IRQ_ENABLE:
        if (data == 1)
            SIM_IRQ_HANDLE(&vm_sim2_dev, SIM_CARD_NUM_CARD2, value);
        break;
    case SIM1_BASE:
        if (data == 1)
            SIM_BASE_HANDLE(&vm_sim1_dev, SIM_CARD_NUM_CARD1, value);
        break;
    case SIM2_BASE:
        if (data == 1)
            SIM_BASE_HANDLE(&vm_sim2_dev, SIM_CARD_NUM_CARD2, value);
        break;
    case SIM1_DATA:
        SIM_DATA_HANDLE(&vm_sim1_dev, SIM_CARD_NUM_CARD1, data, value);
        break;
    case SIM2_DATA:
        SIM_DATA_HANDLE(&vm_sim2_dev, SIM_CARD_NUM_CARD2, data, value);
        break;
    case DMA_SIM1_CONTROL_REG:
        if (data == 1)
        {
            vm_dma_sim1_config.control = value;
            vm_dma_sim1_config.chanel = (value >> 20) & 0b11111;
            vm_dma_sim1_config.direction = (value >> 18) & 1;
            vm_dma_sim1_config.align = value & 0b11;
            vm_dma_sim1_config.transfer_end_interrupt_enable = (value >> 15) & 1;
        }
        break;
    case DMA_SIM2_CONTROL_REG:
        if (data == 1)
        {
            vm_dma_sim2_config.control = value;
            vm_dma_sim2_config.chanel = (value >> 20) & 0b11111;
            vm_dma_sim2_config.direction = (value >> 18) & 1;
            vm_dma_sim2_config.align = value & 0b11;
            vm_dma_sim2_config.transfer_end_interrupt_enable = (value >> 15) & 1;
        }
        break;
    case DMA_SIM1_DATA_ADDR_REG:
        if (data == 1)
        {
            vm_dma_sim1_config.data_addr = value;
        }
        break;
    case DMA_SIM2_DATA_ADDR_REG:
        if (data == 1)
        {
            vm_dma_sim2_config.data_addr = value;
        }
        break;
    case DMA_SIM1_TRANSFER_COUNT_REG:
        if (data == 1)
        {
            if (vm_dma_sim1_config.align == DMA_DATA_BYTE_ALIGN_FOUR)
                value *= 4;
            if (vm_dma_sim1_config.align == DMA_DATA_BYTE_ALIGN_TWO)
                value *= 2;
            vm_dma_sim1_config.transfer_count = value;
        }
        break;
    case DMA_SIM2_TRANSFER_COUNT_REG:
        if (data == 1)
        {
            if (vm_dma_sim2_config.align == DMA_DATA_BYTE_ALIGN_FOUR)
                value *= 4;
            if (vm_dma_sim2_config.align == DMA_DATA_BYTE_ALIGN_TWO)
                value *= 2;
            vm_dma_sim2_config.transfer_count = value;
        }
        break;
    case DMA_SIM1_START_REG:
        // 写入0x8000表示DMA开始运行
        if (data == 1)
        {
            if (value == 0x8000)
            {
                vm_dma_sim1_config.config_finish = 1;
            }
        }
        break;
    case DMA_SIM2_START_REG:
        // 写入0x8000表示DMA开始运行
        if (data == 1)
        {
            if (value == 0x8000)
            {
                vm_dma_sim2_config.config_finish = 1;
            }
        }
        break;
    }
}

inline void handleSimVmEvent(vm_event *vmEvent, uint64_t address)
{
    switch (vmEvent->event)
    {
    case VM_EVENT_SIM_IRQ:
        // 进入usim中断
        changeTmp1 = vmEvent->r0;
        if (vmEvent->r1 == 0)
        {
            uc_mem_write(MTK, SIM1_IRQ_STATUS, &changeTmp1, 4); // 卡一
            if (!StartInterrupt(DEV_IRQ_CHANNEL_USIM, address))
            {
                EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
            }
        }
        if (vmEvent->r1 == 1)
        {
            uc_mem_write(MTK, SIM2_IRQ_STATUS, &changeTmp1, 4); // 卡二
            if (!StartInterrupt(DEV_IRQ_CHANNEL_USIM2, address))
            {
                EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
            }
        }
        break;
    case VM_EVENT_SIM_T0_TX_END:
        if (vmEvent->r0 == 0)
        {
            handle_sim_tx_cmd(&vm_sim1_dev, (SIM_CARD_NUM)vmEvent->r0, vm_dma_sim1_config.transfer_count, vm_dma_sim1_config.data_addr);
        }
        else if (vmEvent->r0 == 1)
        {
            handle_sim_tx_cmd(&vm_sim2_dev, (SIM_CARD_NUM)vmEvent->r0, vm_dma_sim2_config.transfer_count, vm_dma_sim2_config.data_addr);
        }
        break;
    case VM_EVENT_SIM_T0_RX_END:
        if (vmEvent->r0 == 0)
        {
            handle_sim_rx_cmd(&vm_sim1_dev, vmEvent->r0, vm_dma_sim1_config.transfer_count, vm_dma_sim1_config.data_addr);
        }
        else if (vmEvent->r0 == 1)
        {
            handle_sim_rx_cmd(&vm_sim2_dev, vmEvent->r0, vm_dma_sim2_config.transfer_count, vm_dma_sim2_config.data_addr);
        }
        break;
    case VM_EVENT_DMA_IRQ:
        if (!StartInterrupt(DEV_IRQ_CHANNEL_DMA, address))
        {
            EnqueueVMEvent(vmEvent->event, vmEvent->r0, vmEvent->r1);
        }
        break;
    }
}
void InitSimCard()
{
    vm_sim1_dev.event = VM_EVENT_NONE;
    vm_sim1_dev.is_rst = 0;
    vm_sim1_dev.tx_buffer_index = 0;
    vm_sim1_dev.rx_buffer_index = 0;
    vm_sim1_dev.T0EndRxDataPtr = NULL;
    vm_sim1_dev.T0EndRxDataLen = 0;
    vm_sim1_dev.selected_file_id = 0;

    vm_sim2_dev.event = VM_EVENT_NONE;
    vm_sim2_dev.is_rst = 0;
    vm_sim2_dev.tx_buffer_index = 0;
    vm_sim2_dev.rx_buffer_index = 0;
    vm_sim2_dev.T0EndRxDataPtr = NULL;
    vm_sim2_dev.T0EndRxDataLen = 0;
    vm_sim2_dev.selected_file_id = 0;
}

void InitSimCardRegs()
{
    u32 inserted = 1;
    u32 card_type = 0x100;
    u32 sim_status = 0x03;
    u32 imp3_ready = 0;
    u32 retry = 0;
    u32 tout = 0xffff;

    uc_mem_write(MTK, SIM1_INS_REG, &inserted, 4);
    uc_mem_write(MTK, SIM2_INS_REG, &inserted, 4);
    uc_mem_write(MTK, SIM1_STATUS_REG, &sim_status, 4);
    uc_mem_write(MTK, SIM2_STATUS_REG, &sim_status, 4);
    uc_mem_write(MTK, SIM1_CARD_TYPE_REG, &card_type, 4);
    uc_mem_write(MTK, SIM2_CARD_TYPE_REG, &card_type, 4);
    uc_mem_write(MTK, SIM1_IMP3_REG, &imp3_ready, 4);
    uc_mem_write(MTK, SIM2_IMP3_REG, &imp3_ready, 4);
    uc_mem_write(MTK, SIM1_RETRY, &retry, 4);
    uc_mem_write(MTK, SIM2_RETRY, &retry, 4);
    uc_mem_write(MTK, SIM1_TOUT, &tout, 4);
    uc_mem_write(MTK, SIM2_TOUT, &tout, 4);
    sim_write_count_reg(SIM_CARD_NUM_CARD1, 0);
    sim_write_count_reg(SIM_CARD_NUM_CARD2, 0);
}
void SimTaskMain()
{
    if ((vm_sim1_dev.irq_enable & vm_sim1_dev.irq_channel) != 0 && vm_sim1_dev.irq_start) // 允许对应通道中断
    {
        vm_sim1_dev.irq_start = false;
        EnqueueVMEvent(VM_EVENT_SIM_IRQ, vm_sim1_dev.irq_channel, SIM_CARD_NUM_CARD1);
    }
    if ((vm_sim2_dev.irq_enable & vm_sim2_dev.irq_channel) != 0 && vm_sim2_dev.irq_start) // 允许对应通道中断
    {
        vm_sim2_dev.irq_start = false;
        EnqueueVMEvent(VM_EVENT_SIM_IRQ, vm_sim2_dev.irq_channel, SIM_CARD_NUM_CARD2);
    }
    // 开启SIM_DMA后进行命令处理，处理完成后进入t0_end中断
    if (vm_dma_sim1_config.config_finish == 1)
    {
        vm_dma_sim1_config.config_finish = 0;
        if (vm_dma_sim1_config.direction == DMA_DATA_RAM_TO_REG)
        {
            EnqueueVMEvent(VM_EVENT_SIM_T0_TX_END, SIM_CARD_NUM_CARD1, 0);
        }
        else
        {
            EnqueueVMEvent(VM_EVENT_SIM_T0_RX_END, SIM_CARD_NUM_CARD1, 0);
        }
    }
    if (vm_dma_sim2_config.config_finish == 1)
    {
        vm_dma_sim2_config.config_finish = 0;
        if (vm_dma_sim2_config.direction == DMA_DATA_RAM_TO_REG) // 设备向SIM卡写入命令
        {
            EnqueueVMEvent(VM_EVENT_SIM_T0_TX_END, SIM_CARD_NUM_CARD2, 0);
        }
        else
        { // SIM卡向设备写入数据
            EnqueueVMEvent(VM_EVENT_SIM_T0_RX_END, SIM_CARD_NUM_CARD2, 0);
        }
    }
}