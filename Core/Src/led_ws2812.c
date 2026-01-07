#include "led_ws2812.h"
#include <string.h>

/* ===== RGB ДАННЫЕ ===== */
uint8_t ws2812_rgb[LED_COUNT][3];

/* ===== DMA PWM буфер ===== */
static uint16_t pwm_buf[DMA_BUF_BITS];

/* ===== внутреннее состояние ===== */
static uint32_t led_index = 0;
static bool sending_reset = false;

void DMA1_Channel1_IRQHandler(void)
{
	if (LL_DMA_IsActiveFlag_HT1(DMA1)) // здесь указатель массива равен 1/2 длинны DMA_HALF_BITS
    {
        LL_DMA_ClearFlag_HT1(DMA1);

        if (sending_reset)
        {
        	for (uint32_t i = 0; i < DMA_HALF_BITS; i++)
        	{
        		pwm_buf[i] = 0;
        	}
        	return;
        }

        uint8_t g = ws2812_rgb[led_index][1];
        uint8_t r = ws2812_rgb[led_index][0];
        uint8_t b = ws2812_rgb[led_index][2];

        pwm_buf[0] = (g & 0x80) ? WS_T1H : WS_T0H; // bit 7
        pwm_buf[1] = (g & 0x40) ? WS_T1H : WS_T0H; // bit 6
        pwm_buf[2] = (g & 0x20) ? WS_T1H : WS_T0H; // bit 5
        pwm_buf[3] = (g & 0x10) ? WS_T1H : WS_T0H; // bit 4
        pwm_buf[4] = (g & 0x08) ? WS_T1H : WS_T0H; // bit 3
        pwm_buf[5] = (g & 0x04) ? WS_T1H : WS_T0H; // bit 2
        pwm_buf[6] = (g & 0x02) ? WS_T1H : WS_T0H; // bit 1
        pwm_buf[7] = (g & 0x01) ? WS_T1H : WS_T0H; // bit 0

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

        led_index++;
        if (led_index >= LED_COUNT)
        {
            sending_reset = true;
        }
    }

    if (LL_DMA_IsActiveFlag_TC1(DMA1)) // здесь указатель массива равен 0 длинны
    {
        LL_DMA_ClearFlag_TC1(DMA1);

        if (sending_reset)
        {
        	for (uint32_t i = DMA_HALF_BITS; i < DMA_BUF_BITS; i++)
        	{
        		pwm_buf[i] = 0;
        	}
        	return;
        }


        uint8_t g = ws2812_rgb[led_index][1];
        uint8_t r = ws2812_rgb[led_index][0];
        uint8_t b = ws2812_rgb[led_index][2];

        pwm_buf[24] = (g & 0x80) ? WS_T1H : WS_T0H; // bit 7
        pwm_buf[25] = (g & 0x40) ? WS_T1H : WS_T0H; // bit 6
        pwm_buf[26] = (g & 0x20) ? WS_T1H : WS_T0H; // bit 5
        pwm_buf[27] = (g & 0x10) ? WS_T1H : WS_T0H; // bit 4
        pwm_buf[28] = (g & 0x08) ? WS_T1H : WS_T0H; // bit 3
        pwm_buf[29] = (g & 0x04) ? WS_T1H : WS_T0H; // bit 2
        pwm_buf[30] = (g & 0x02) ? WS_T1H : WS_T0H; // bit 1
        pwm_buf[31] = (g & 0x01) ? WS_T1H : WS_T0H; // bit 0

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

void TIM1_DMA_LED_Init(void) {
    // 0. Очистка буфера (Reset bits должны быть 0)
    sending_reset = true;
    memset(pwm_buf, 0, sizeof(pwm_buf));

/*
	 for (uint32_t i = 0; i < DMA_BUF_BITS; i++)
	 {
		 pwm_buf[i]=i+1;
		 pwm_buf1[i]=i+10;

	 }/**/


    // 1. Тактирование
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
 //   LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMAMUX1); // ← ОБЯЗАТЕЛЬНО
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);



    // 2. GPIO PA8 -> TIM1_CH1
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
    LL_GPIO_SetAFPin_8_15(GPIOA, LL_GPIO_PIN_8, LL_GPIO_AF_2);
    LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_VERY_HIGH);
    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);

    // 3. Настройка DMA1 Channel 1 и DMAMUX
    // В STM32G0 нужно обязательно связать запрос таймера с каналом DMA через DMAMUX
    LL_DMAMUX_SetRequestID(DMAMUX1, LL_DMAMUX_CHANNEL_1, LL_DMAMUX_REQ_TIM1_CH1);

    LL_DMA_ConfigTransfer(DMA1, LL_DMA_CHANNEL_1,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH |
        LL_DMA_PRIORITY_HIGH |
//		LL_DMA_MODE_NORMAL |
		LL_DMA_MODE_CIRCULAR |
        LL_DMA_PERIPH_NOINCREMENT |
        LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pwm_buf);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);

    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&TIM1->CCR1);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, DMA_BUF_BITS);

    // 4. Настройка таймера TIM1
    LL_TIM_SetPrescaler(TIM1, 0);
    LL_TIM_SetAutoReload(TIM1, PWM_ARR);
    LL_TIM_OC_SetMode(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_CC_EnableChannel(TIM1, LL_TIM_CHANNEL_CH1);

    // Особенности TIM1 (Advanced Timer)
    LL_TIM_EnableAllOutputs(TIM1); // MOE = 1
    LL_TIM_EnableDMAReq_CC1(TIM1); // Запрос DMA по событию Compare CH1

    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    NVIC_EnableIRQ(DMA1_Channel1_IRQn);

    LL_TIM_OC_SetPolarity(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
    LL_TIM_OC_SetIdleState(TIM1, LL_TIM_CHANNEL_CH1, LL_TIM_OCIDLESTATE_LOW);

    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_OC_SetCompareCH1(TIM1, 0);

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
