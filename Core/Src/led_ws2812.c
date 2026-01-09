#include "led_ws2812.h"
#include <string.h>

/* ===== RGB ДАННЫЕ ===== */
/* Буфер цветов для всех светодиодов.
 * Формат: [номер светодиода][0=R,1=G,2=B]
 * Заполняется пользователем, DMA его не трогает */
uint8_t ws2812_rgb[LED_COUNT][3];

/* ===== DMA PWM буфер ===== */
/* Кольцевой буфер значений CCR для TIM1.
 * Содержит ШИМ-длительности для кодирования битов WS2812 */
static uint16_t pwm_buf[DMA_BUF_BITS];

/* ===== внутреннее состояние ===== */
/* Текущий номер светодиода, который кодируется в DMA */
static uint32_t led_index = 0;

/* Флаг передачи reset-паузы (низкий уровень >50 мкс) */
static bool sending_reset = false;

/* ===== обработчик DMA ===== */
/* Обслуживает половинную (HT) и полную (TC) передачу DMA */
void DMA1_Channel1_IRQHandler(void)
{
    /* ===== Half Transfer ===== */
    /* DMA дошёл до середины буфера (0 … DMA_HALF_BITS-1) */
	if (LL_DMA_IsActiveFlag_HT1(DMA1))
    {
        LL_DMA_ClearFlag_HT1(DMA1);

        /* Если идёт reset — просто заливаем нулями */
        if (sending_reset)
        {
        	for (uint32_t i = 0; i < DMA_HALF_BITS; i++)
        	{
        		pwm_buf[i] = 0;
        	}
        	return;
        }

        /* Берём цвет текущего светодиода */
        uint8_t g = ws2812_rgb[led_index][1];
        uint8_t r = ws2812_rgb[led_index][0];
        uint8_t b = ws2812_rgb[led_index][2];

        /* Кодирование 24 бит (GRB) в PWM */
        pwm_buf[0] = (g & 0x80) ? WS_T1H : WS_T0H;
        pwm_buf[1] = (g & 0x40) ? WS_T1H : WS_T0H;
        pwm_buf[2] = (g & 0x20) ? WS_T1H : WS_T0H;
        pwm_buf[3] = (g & 0x10) ? WS_T1H : WS_T0H;
        pwm_buf[4] = (g & 0x08) ? WS_T1H : WS_T0H;
        pwm_buf[5] = (g & 0x04) ? WS_T1H : WS_T0H;
        pwm_buf[6] = (g & 0x02) ? WS_T1H : WS_T0H;
        pwm_buf[7] = (g & 0x01) ? WS_T1H : WS_T0H;

        pwm_buf[ 8] = (r & 0x80) ? WS_T1H : WS_T0H;
        pwm_buf[ 9] = (r & 0x40) ? WS_T1H : WS_T0H;
        pwm_buf[10] = (r & 0x20) ? WS_T1H : WS_T0H;
        pwm_buf[11] = (r & 0x10) ? WS_T1H : WS_T0H;
        pwm_buf[12] = (r & 0x08) ? WS_T1H : WS_T0H;
        pwm_buf[13] = (r & 0x04) ? WS_T1H : WS_T0H;
        pwm_buf[14] = (r & 0x02) ? WS_T1H : WS_T0H;
        pwm_buf[15] = (r & 0x01) ? WS_T1H : WS_T0H;

        pwm_buf[16] = (b & 0x80) ? WS_T1H : WS_T0H;
        pwm_buf[17] = (b & 0x40) ? WS_T1H : WS_T0H;
        pwm_buf[18] = (b & 0x20) ? WS_T1H : WS_T0H;
        pwm_buf[19] = (b & 0x10) ? WS_T1H : WS_T0H;
        pwm_buf[20] = (b & 0x08) ? WS_T1H : WS_T0H;
        pwm_buf[21] = (b & 0x04) ? WS_T1H : WS_T0H;
        pwm_buf[22] = (b & 0x02) ? WS_T1H : WS_T0H;
        pwm_buf[23] = (b & 0x01) ? WS_T1H : WS_T0H;

        /* Переходим к следующему светодиоду */
        led_index++;
        if (led_index >= LED_COUNT)
        {
            sending_reset = true;
        }
    }

    /* ===== Transfer Complete ===== */
    /* DMA дошёл до конца буфера (DMA_HALF_BITS … DMA_BUF_BITS-1) */
    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);

        /* Reset-фаза: заливаем вторую половину нулями */
        if (sending_reset)
        {
        	for (uint32_t i = DMA_HALF_BITS; i < DMA_BUF_BITS; i++)
        	{
        		pwm_buf[i] = 0;
        	}
        	return;
        }

        /* Берём цвет следующего светодиода */
        uint8_t g = ws2812_rgb[led_index][1];
        uint8_t r = ws2812_rgb[led_index][0];
        uint8_t b = ws2812_rgb[led_index][2];

        /* Кодирование битов во второй половине буфера */
        pwm_buf[24] = (g & 0x80) ? WS_T1H : WS_T0H;
        pwm_buf[25] = (g & 0x40) ? WS_T1H : WS_T0H;
        pwm_buf[26] = (g & 0x20) ? WS_T1H : WS_T0H;
        pwm_buf[27] = (g & 0x10) ? WS_T1H : WS_T0H;
        pwm_buf[28] = (g & 0x08) ? WS_T1H : WS_T0H;
        pwm_buf[29] = (g & 0x04) ? WS_T1H : WS_T0H;
        pwm_buf[30] = (g & 0x02) ? WS_T1H : WS_T0H;
        pwm_buf[31] = (g & 0x01) ? WS_T1H : WS_T0H;

        pwm_buf[32] = (r & 0x80) ? WS_T1H : WS_T0H;
        pwm_buf[33] = (r & 0x40) ? WS_T1H : WS_T0H;
        pwm_buf[34] = (r & 0x20) ? WS_T1H : WS_T0H;
        pwm_buf[35] = (r & 0x10) ? WS_T1H : WS_T0H;
        pwm_buf[36] = (r & 0x08) ? WS_T1H : WS_T0H;
        pwm_buf[37] = (r & 0x04) ? WS_T1H : WS_T0H;
        pwm_buf[38] = (r & 0x02) ? WS_T1H : WS_T0H;
        pwm_buf[39] = (r & 0x01) ? WS_T1H : WS_T0H;

        pwm_buf[40] = (b & 0x80) ? WS_T1H : WS_T0H;
        pwm_buf[41] = (b & 0x40) ? WS_T1H : WS_T0H;
        pwm_buf[42] = (b & 0x20) ? WS_T1H : WS_T0H;
        pwm_buf[43] = (b & 0x10) ? WS_T1H : WS_T0H;
        pwm_buf[44] = (b & 0x08) ? WS_T1H : WS_T0H;
        pwm_buf[45] = (b & 0x04) ? WS_T1H : WS_T0H;
        pwm_buf[46] = (b & 0x02) ? WS_T1H : WS_T0H;
        pwm_buf[47] = (b & 0x01) ? WS_T1H : WS_T0H;

        led_index++;
        if (led_index >= LED_COUNT)
        {
            sending_reset = true;
        }
    }
}



void ws2812_set_rgb(uint16_t led, uint8_t r, uint8_t g, uint8_t b)
{
    if (led >= LED_COUNT)
        return;

    ws2812_rgb[led][0] = r;
    ws2812_rgb[led][1] = g;
    ws2812_rgb[led][2] = b;
}

void TIM1_DMA_LED_Init(void)
{
    /* ===== внутреннее состояние ===== */
    sending_reset = true;
    led_index = 0;
    memset(pwm_buf, 0, sizeof(pwm_buf));

    /* ===== тактирование ===== */
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

    /* ===== GPIO PA8 -> TIM1_CH1 ===== */
    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin        = LL_GPIO_PIN_8;
    GPIO_InitStruct.Mode       = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull       = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate  = LL_GPIO_AF_2;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ===== TIM1 base ===== */
    LL_TIM_InitTypeDef TIM_InitStruct = {0};
    TIM_InitStruct.Prescaler         = 0;
    TIM_InitStruct.CounterMode       = LL_TIM_COUNTERMODE_UP;
    TIM_InitStruct.Autoreload        = PWM_ARR;
    TIM_InitStruct.ClockDivision     = LL_TIM_CLOCKDIVISION_DIV1;
    TIM_InitStruct.RepetitionCounter = 0;
    LL_TIM_Init(TIM1, &TIM_InitStruct);/**/

    LL_TIM_DisableARRPreload(TIM1);

    /* ===== OC CH1 ===== */
    LL_TIM_OC_InitTypeDef TIM_OC_InitStruct = {0};
    TIM_OC_InitStruct.OCMode       = LL_TIM_OCMODE_PWM1;
    TIM_OC_InitStruct.OCState      = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState     = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity   = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState  = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);

    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);/**/

    /* ===== TRGO / master ===== */
    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);

    /* ===== BDTR (CRITICAL) ===== */
    LL_TIM_BDTR_InitTypeDef TIM_BDTRInitStruct = {0};
    TIM_BDTRInitStruct.OSSRState       = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState       = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel       = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime        = 0;
    TIM_BDTRInitStruct.BreakState      = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);/**/

    /* ===== DMA ===== */

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    // ниже две строчки одно и тоже,
    //LL_DMA_CHANNEL_1=LL_DMAMUX_CHANNEL_0!!!
    // GPT на них путается, сам вычислил!!!
    LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMAMUX_REQ_TIM1_CH1);
    LL_DMAMUX_SetRequestID(DMAMUX1,
                           LL_DMAMUX_CHANNEL_0,
                           LL_DMAMUX_REQ_TIM1_CH1);

    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
        LL_DMA_PRIORITY_HIGH |
        LL_DMA_MODE_CIRCULAR |
        LL_DMA_PERIPH_NOINCREMENT |
        LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pwm_buf);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&TIM1->CCR1);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, DMA_BUF_BITS);

    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    /* ===== старт ===== */
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_EnableDMAReq_CC1(TIM1);
    LL_TIM_EnableAllOutputs(TIM1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_TIM_EnableCounter(TIM1);
}


/* ===== старт передачи ===== */
void ws2812_start(void)
{
    if (!sending_reset) return; // уже идёт передача
    led_index = 0;
    sending_reset = false;
}


void hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v,
                       uint8_t *r, uint8_t *g, uint8_t *b)
{
    uint8_t region = h / 43;
    uint16_t remainder = (h - region * 43) * 6;

    uint8_t p = (v * (255 - s)) >> 8;
    uint8_t q = (v * (255 - ((s * remainder) >> 8))) >> 8;
    uint8_t t = (v * (255 - ((s * (255 - remainder)) >> 8))) >> 8;

    switch (region)
    {
        case 0: *r = v; *g = t; *b = p; break;
        case 1: *r = q; *g = v; *b = p; break;
        case 2: *r = p; *g = v; *b = t; break;
        case 3: *r = p; *g = q; *b = v; break;
        case 4: *r = t; *g = p; *b = v; break;
        default:*r = v; *g = p; *b = q; break;
    }
}
