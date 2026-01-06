#include "main.h"
#define MAX_UART_BUFFER_SIZE 1024

u8 uart1Buffer[MAX_UART_BUFFER_SIZE]; // 1MB缓存
int uart1BufferIndex = 0;

u8 uart2Buffer[MAX_UART_BUFFER_SIZE]; // 1MB缓存
int uart2BufferIndex = 0;

u8 uart3Buffer[MAX_UART_BUFFER_SIZE]; // 1MB缓存
int uart3BufferIndex = 0;

#define UART1_FIFO_TX 0x78000300
#define UART2_FIFO_TX 0x78000400
#define UART3_FIFO_TX 0x78000500

void handleUartReg(uint64_t addr, u32 data, uint64_t value)
{
    switch (addr)
    {

    case UART1_FIFO_TX:
        if (data == 1)
        {
            uart1Buffer[uart1BufferIndex++] = (u8)value;
            if (value == '\n' || uart1BufferIndex == (MAX_UART_BUFFER_SIZE - 1))
            {
                printf("[uart_1]%s\n", uart1Buffer);
                uart1BufferIndex = 0;
            }
        }
        break;
    case UART2_FIFO_TX:
        if (data == 1)
        {
            uart2Buffer[uart2BufferIndex++] = (u8)value;
            if (value == '\n' || uart2BufferIndex == (MAX_UART_BUFFER_SIZE - 1))
            {
                printf("[uart_2]%s\n", uart2Buffer);
                uart2BufferIndex = 0;
            }
        }

        break;
    case UART3_FIFO_TX:
        if (data == 1)
        {
            uart3Buffer[uart3BufferIndex++] = (u8)value;
            if (value == '\n' || uart3BufferIndex == (MAX_UART_BUFFER_SIZE - 1))
            {
                printf("[uart_3]%s\n", uart1Buffer);
                uart3BufferIndex = 0;
            }
        }
        break;
    }
}