/*
 * led_ws2812.h
 *
 *  Created on: Jan 5, 2026
 *      Author: OkhrimenkoDI
 */

#ifndef INC_LED_WS2812_H_
#define INC_LED_WS2812_H_

#include <stdint.h>
#include <stdbool.h>

#include "stm32g0xx.h"          // CMSIS + device
#include "stm32g0xx_ll_bus.h"
#include "stm32g0xx_ll_rcc.h"
#include "stm32g0xx_ll_tim.h"
#include "stm32g0xx_ll_gpio.h"
#include "stm32g0xx_ll_dma.h"
#include "main.h"



#define LED_COUNT 144
#define RESET_BITS 100 // Время сброса > 280 мкс (зависит от периода таймера)
#define BUFFER_SIZE (LED_COUNT * 24 + RESET_BITS)

// Тайминги для WS2812B при тактовой частоте TIM1 64 МГц (период 1.25 мкс)
#define PWM_ARR 79       // 800 кГц
#define WS_T1H  51       // ~0.8 мкс (логическая "1")
#define WS_T0H  26       // ~0.4 мкс (логический "0") [4]


#define DMA_BUF_BITS       8*3*2 // буфер под два светодиода
#define DMA_HALF_BITS      (DMA_BUF_BITS / 2)

extern uint8_t ws2812_rgb[LED_COUNT][3];

void ws2812_start(void);
void TIM1_DMA_LED_Init(void);

void ws2812_dma_half_irq(void);
void ws2812_dma_full_irq(void);
void ws2812_set_rgb(uint16_t led, uint8_t r, uint8_t g, uint8_t b);


#endif /* INC_LED_WS2812_H_ */
