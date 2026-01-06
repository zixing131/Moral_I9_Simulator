#include "lcd.h"

void ClearLayer(lcdLayer *ll)
{
    if (ll != NULL)
    {
        ll->alpha = 0; // 0不透明 0xff 完全透明
        ll->buffer_ptr = 0;
        ll->height = 0;
        ll->width = 0;
    }
}

void InitLcd()
{
    for (int i = 0; i < 4; i++)
    {
        ClearLayer(&lcdLayerList[i]);
    }
}

void Draw_Loading_Process()
{
    u8 r = 0, g = 0, b = 0;
    int i, j, offset;
    SDL_Surface *screenSurface = SDL_GetWindowSurface(window);
    SDL_Rect rect;
    SDL_GetWindowSize(window, &rect.w, &rect.h);
    // 背景清空
    rect.x = 0;
    rect.y = 0;

    SDL_FillRect(screenSurface, &rect, SDL_MapRGB(screenSurface->format, 0xff, 0xff, 0xff));

    float p = (float)frameTicks / 100;
    int h2 = rect.h / 2;
    // 动态进度
    for (j = h2 - 10; j < h2 + 10; j++)
    {
        if (j < 0)
            continue;
        for (i = 0; i < (int)(rect.w * p); i++)
        {
            offset = (i + j * rect.w);
            *((Uint32 *)screenSurface->pixels + offset) = SDL_MapRGB(screenSurface->format, 0x00, 0x96, 0x88);
        }
    }
}

void renderGdiBufferToWindow()
{
    // 获取窗口的表面
    SDL_Surface *screenSurface = SDL_GetWindowSurface(window);
    lcdLayer *layer;
    u16 i;
    u16 color;
    u32 offset;
    u32 offset2;
    u32 colorCache;
    if (Lcd_Need_Update)
    {
        for (i = Lcd_Update_Y; i < (Lcd_Update_Y + Lcd_Update_H); i++)
        {
            if (i >= 400)
                continue;
            u16 *lineStart = (u16 *)(Lcd_Cache_Buffer + Lcd_Update_Pitch * (i - Lcd_Update_Y));
            for (u16 j = Lcd_Update_X; j < (Lcd_Update_X + Lcd_Update_W); j++)
            {
                if (j >= 240)
                    continue;
                color = *(lineStart + (j - Lcd_Update_X));
                offset = i * 240 + j;
                //  不是透明的，覆盖上个图层颜色
                *((Uint32 *)screenSurface->pixels + offset) = SDL_MapRGB(screenSurface->format, PIXEL565R(color), PIXEL565G(color), PIXEL565B(color));
            }
        }
        Lcd_Need_Update = 0;
        printf("LCD Update Ok\n");
    }
    // 更新窗口
    SDL_UpdateWindowSurface(window);
}

void lcdTaskMain()
{

    if (Lcd_Need_Update)
    {
        renderGdiBufferToWindow();
    }

    if (currentTime > render_time)
    {
        render_time = currentTime + 5;
        // EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
    }
}

inline void handleLcdReg(uint64_t address, u32 data, uint64_t value)
{
    u32 tmp;
    lcdLayer *layer;
    switch (address)
    {
    case 0x90000004:
        if (data == 1)
        {
            uc_mem_write(MTK, 0x90000008, &value, 4);
            EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
        }
        break;
    case 0x9000000c: // LCD Interface Frame Transfer Register
        if (data == 1 && value == 0x8000)
        {
            // 需要更新显示
            lcdUpdateFlag = true;
        }
        break;
    case 0x900000b0:
    case 0x900000e0:
    case 0x90000110:
    case 0x90000140:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xb0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->alpha = value & 0b11111111;
            layer->alpha_enable = (value >> 8) & 0b1;
            layer->src_key_enable = (value >> 14) & 0b1;
            layer->rotate = (value >> 11) & 0b111;
        }
        break;
    case 0x900000b4:
    case 0x900000e4:
    case 0x90000114:
    case 0x90000144:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xb0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->transparent_color = value & 0xffff;
        }
        break;
    case 0x900000b8: // window display offset
    case 0x900000e8:
    case 0x90000118:
    case 0x90000148:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xb0) / 0x30;
            layer = &lcdLayerList[tmp];
            if (layer->offset_x != 0 || layer->offset_y != 0)
            {
                layer->offset_x = (value & 0xffff);
                layer->offset_y = ((value >> 16) & 0xffff);
                printf("layer[%d] offset_x:%d offset_y:%d\n", tmp, layer->offset_x, layer->offset_y);
            }
        }
        break;
    case 0x900000bc: // display start address
    case 0x900000ec:
    case 0x9000011c:
    case 0x9000014c:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xb0) / 0x30;
            lcdLayerList[tmp].buffer_ptr = value;
        }
        break;
    case 0x900000c8:
    case 0x900000f8:
    case 0x900000128:
    case 0x900000158:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xc0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->memory_offx = value & 0xffff;
            layer->memory_offy = (value >> 16) & 0xffff;
            // printf("layer[%d] memory_offx:%d memory_offy:%d\n", tmp, layer->memory_offx, layer->memory_offy);
        }
        break;
    case 0x900000cc: // 一行像素多少字节
    case 0x900000fc:
    case 0x90000012c:
    case 0x90000015c:
        if (data == 1 && value > 0)
        {
            tmp = (address - LCD_BASE - 0xc0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->pitch = value;
            printf("Init Lcd layer[%d] Pitch %d\n", tmp, layer->pitch);
            if (layer->width > 0 && layer->height > 0)
            {
                if (layer->lcdBuffer == NULL) // 只初始化一次
                {
                    layer->lcdBuffer = malloc(layer->pitch * layer->height);
                    printf("Init Lcd layer[%d] %d %d\n", tmp, layer->width, layer->height);
                }
#ifdef GDI_LAYER_DEBUG_
                if (!LCD_Initialized)
                {
                    SDL_SetWindowSize(window, layer->width, layer->height);
                    LCD_Initialized = true;
                }
#else
                LCD_Initialized = true;
#endif
            }
        }
        break;
    case 0x900000c0:
    case 0x900000f0:
    case 0x90000120:
    case 0x90000150:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xc0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->width = value & 0xffff;
            layer->height = (value >> 16) & 0xffff;
        }
        break;
    case 0x90000084:
        if (data == 1)
        {
            lcdRegionOfInterest_st.roi_sx = value & 0xffff - 1024;         // x
            lcdRegionOfInterest_st.roi_sy = (value >> 16) & 0xffff - 1024; // y
            printf("set roi sx:%d sy:%d\n", lcdRegionOfInterest_st.roi_sx, lcdRegionOfInterest_st.roi_sy);
        }
        break;
    case 0x9000008c:
        if (data == 1)
        {
            lcdRegionOfInterest_st.data_addr_type = value;
        }
        break;
    case 0x90000090:
        if (data == 1 && value != 0)
        {
            lcdRegionOfInterest_st.roi_width = value & 0xffff;          // x
            lcdRegionOfInterest_st.roi_height = (value >> 16) & 0xffff; // y
            printf("set roi width:%d height:%d\n", lcdRegionOfInterest_st.roi_width, lcdRegionOfInterest_st.roi_height);
        }
        break;

    case 0x9000009c:
        if (data == 1)
        {
            lcdRegionOfInterest_st.background = value;
        }
        break;
    case 0x90000080: // roi control
        if (data == 1)
        {
            lcdLayerList[0].enable = (value & 0x80000000) == 0x80000000;
            lcdLayerList[1].enable = (value & 0x40000000) == 0x40000000;
            lcdLayerList[2].enable = (value & 0x20000000) == 0x20000000;
            lcdLayerList[3].enable = (value & 0x10000000) == 0x10000000;
        }
        break;
    default:
        break;
    }
}