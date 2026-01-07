#include "led_ws2812.h"
#include <string.h>

/* ===== RGB ДАННЫЕ ===== */
uint8_t ws2812_rgb[LED_COUNT][3];

/* ===== DMA PWM буфер ===== */
static uint16_t pwm_buf[DMA_BUF_BITS];

/* ===== внутреннее состояние ===== */
static uint32_t led_index = 0;
static uint8_t bit_index = 0;
static bool sending_reset = false;



/* ===== заполнение половины буфера ===== */
static void fill_half(uint16_t *dst, uint32_t count)
{
    for (uint32_t i = 0; i < count; i++)
    {
        if (sending_reset)
        {
            dst[i] = 0;
            continue;
        }

        uint8_t byte;

        /* порядок WS2812: G R B */
        if (bit_index < 8)
            byte = ws2812_rgb[led_index][1];
        else if (bit_index < 16)
            byte = ws2812_rgb[led_index][0];
        else
            byte = ws2812_rgb[led_index][2];

        uint8_t bit = 7 - (bit_index & 7);
        dst[i] = (byte & (1 << bit)) ? WS_T1H : WS_T0H;

        bit_index++;
        if (bit_index == 24)
        {
            bit_index = 0;
            led_index++;
            if (led_index == LED_COUNT)
            {
                sending_reset = true;
            }
        }
    }
}

/* ===== IRQ half ===== */
void ws2812_dma_half_irq(void)
{
    fill_half(&pwm_buf[0], DMA_HALF_BITS);
}

/* ===== IRQ full ===== */
void ws2812_dma_full_irq(void)
{
    fill_half(&pwm_buf[DMA_HALF_BITS], DMA_HALF_BITS);
}

/* ===== DMA IRQ ===== */
void DMA1_Channel1_IRQHandler(void)
{
    if (LL_DMA_IsActiveFlag_HT1(DMA1))
    {
        LL_DMA_ClearFlag_HT1(DMA1);
        ws2812_dma_half_irq();
    }

    if (LL_DMA_IsActiveFlag_TC1(DMA1))
    {
        LL_DMA_ClearFlag_TC1(DMA1);
        ws2812_dma_full_irq();
    }
}


/*
void SetRGB(uint8_t r, uint8_t g, uint8_t b, uint16_t led_idx) {
    if (led_idx >= LED_COUNT) return;

    // Данные в WS2812B: GRB, MSB first
    for (int i = 0; i < 8; i++) {
        led_buffer[led_idx * 24 + i]      = (g & (0x80 >> i)) ? CW1 : CW0;
        led_buffer[led_idx * 24 + 8 + i]  = (r & (0x80 >> i)) ? CW1 : CW0;
        led_buffer[led_idx * 24 + 16 + i] = (b & (0x80 >> i)) ? CW1 : CW0;
    }
}*/

void TIM1_DMA_LED_Init(void) {
    led_index = 0;
    bit_index = 0;
    sending_reset = false;

    fill_half(&pwm_buf[0], DMA_HALF_BITS);
    fill_half(&pwm_buf[DMA_HALF_BITS], DMA_HALF_BITS);

    // 0. Очистка буфера (Reset bits должны быть 0)
  //  memset(led_buffer, 0, sizeof(led_buffer));

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
		LL_DMA_MODE_CIRCULAR |
        LL_DMA_PERIPH_NOINCREMENT |
        LL_DMA_MEMORY_INCREMENT);

    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, DMA_BUF_BITS);
    LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);
    LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);

  /*  LL_DMA_ConfigAddresses(DMA1, LL_DMA_CHANNEL_1,
        (uint32_t)led_buffer,
        (uint32_t)&TIM1->CCR1,
        LL_DMA_DIRECTION_MEMORY_TO_PERIPH);*/

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
}

/* ===== старт передачи ===== */
void ws2812_start(void)
{
    led_index = 0;
    bit_index = 0;
    sending_reset = false;

    fill_half(&pwm_buf[0], DMA_HALF_BITS);
    fill_half(&pwm_buf[DMA_HALF_BITS], DMA_HALF_BITS);

    LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)pwm_buf);
    LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&TIM1->CCR1);
    LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, DMA_BUF_BITS);

    LL_DMA_EnableIT_HT(DMA1, LL_DMA_CHANNEL_1);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_CHANNEL_1);

    LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
    LL_TIM_EnableCounter(TIM1);


}
