#include "lcd.h"
#include "touchscreen.h"

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
    De_PeriodicRefreshAllowed = 0;
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
            SDL_PutPixel32(screenSurface, (unsigned)i, (unsigned)j,
                SDL_MapRGB(screenSurface->format, 0x00, 0x96, 0x88));
        }
    }
}

void renderGdiBufferToWindow()
{
    if (Lcd_Need_Update)
    {
        Lcd_Need_Update = 0;
        SDL_UpdateWindowSurface(window);
    }
}

void lcdTaskMain()
{
    /* 一次性调用触摸 patch（设置 bLCDisOn 等固件 RAM 变量），之后不再调用避免阻塞 */
    static u8 touch_patch_called = 0;
    if (!touch_patch_called)
    {
        touch_patch_called = 1;
        moral_touch_init_patch();
    }

#if MORAL_LCD_PERIODIC_REFRESH_MS > 0
    static clock_t last_de_poll = 0;
    clock_t threshold = (clock_t)(MORAL_LCD_PERIODIC_REFRESH_MS * CLOCKS_PER_SEC / 1000);
    if (threshold < 1)
        threshold = 1;
    if (last_de_poll == 0)
        last_de_poll = currentTime;
    if ((clock_t)(currentTime - last_de_poll) >= threshold)
    {
        last_de_poll = currentTime;
        de_emulator_periodic_refresh();
    }
#endif

    if (currentTime > render_time)
    {
        render_time = currentTime + 5;
    }
}

static u32 composite_call_cnt = 0;

void compositeLayers()
{
    SDL_Surface *sfc = SDL_GetWindowSurface(window);
    if (!sfc) return;
    composite_call_cnt++;
    if (composite_call_cnt <= 5 || (composite_call_cnt % 100) == 0)
        printf("[LCD-COMPOSITE] #%u layers: %d,%d,%d,%d\n", composite_call_cnt,
               lcdLayerList[0].enable, lcdLayerList[1].enable,
               lcdLayerList[2].enable, lcdLayerList[3].enable);

    u32 roiW = lcdRegionOfInterest_st.roi_width;
    u32 roiH = lcdRegionOfInterest_st.roi_height;
    if (roiW == 0 || roiW > 240) roiW = 240;
    if (roiH == 0 || roiH > 400) roiH = 400;

    u16 bgColor = (u16)lcdRegionOfInterest_st.background;
    for (u32 y = 0; y < roiH; y++)
    {
        for (u32 x = 0; x < roiW; x++)
        {
            SDL_PutPixel32(sfc, x, y, SDL_MapRGB(sfc->format,
                PIXEL565R(bgColor), PIXEL565G(bgColor), PIXEL565B(bgColor)));
        }
    }

    for (int l = 0; l < 4; l++)
    {
        lcdLayer *layer = &lcdLayerList[l];
        if (!layer->enable || layer->buffer_ptr == 0 ||
            layer->width == 0 || layer->height == 0 || layer->pitch == 0)
            continue;

        u32 bufSize = layer->pitch * layer->height;
        u32 maxRows = layer->height;
        if (bufSize > sizeof(Lcd_Cache_Buffer))
        {
            bufSize = sizeof(Lcd_Cache_Buffer);
            maxRows = bufSize / layer->pitch;
        }

        uc_mem_read(MTK, layer->buffer_ptr, Lcd_Cache_Buffer, bufSize);

        for (u32 y = 0; y < maxRows; y++)
        {
            u32 destY = y + layer->offset_y;
            if (destY >= roiH) break;

            u16 *lineStart = (u16 *)(Lcd_Cache_Buffer + layer->pitch * y);
            for (u32 x = 0; x < layer->width; x++)
            {
                u32 destX = x + layer->offset_x;
                if (destX >= roiW) break;

                u16 color = *(lineStart + x);

                if (layer->src_key_enable && color == (u16)layer->transparent_color)
                    continue;

                SDL_PutPixel32(sfc, destX, destY, SDL_MapRGB(sfc->format,
                    PIXEL565R(color), PIXEL565G(color), PIXEL565B(color)));
            }
        }
    }

    Lcd_Need_Update = 1;
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
    case 0x9000000c: // LCD_START - Frame Transfer
        if (data == 1 && (value & 0x8000))
        {
            compositeLayers();
            EnqueueVMEvent(VM_EVENT_LCD_IRQ, 0, 0);
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
            layer->offset_x = (value & 0xffff);
            layer->offset_y = ((value >> 16) & 0xffff);
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
    case 0x90000128:
    case 0x90000158:
        if (data == 1)
        {
            tmp = (address - LCD_BASE - 0xc0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->memory_offx = value & 0xffff;
            layer->memory_offy = (value >> 16) & 0xffff;
        }
        break;
    case 0x900000cc:
    case 0x900000fc:
    case 0x9000012c:
    case 0x9000015c:
        if (data == 1 && value > 0)
        {
            tmp = (address - LCD_BASE - 0xc0) / 0x30;
            layer = &lcdLayerList[tmp];
            layer->pitch = value;
            printf("Init Lcd layer[%d] Pitch %d\n", tmp, layer->pitch);
            if (layer->width > 0 && layer->height > 0)
            {
                if (layer->lcdBuffer == NULL)
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
            lcdRegionOfInterest_st.roi_sx = (value & 0xffff);
            lcdRegionOfInterest_st.roi_sy = ((value >> 16) & 0xffff);
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
            lcdRegionOfInterest_st.roi_width = value & 0xffff;
            lcdRegionOfInterest_st.roi_height = (value >> 16) & 0xffff;
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