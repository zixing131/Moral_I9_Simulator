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
        mtk_touch_regs_sync();
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