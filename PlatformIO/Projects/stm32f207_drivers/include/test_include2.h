#include "stm32f2xx_hal.h"

void LED_Init();

#define LED_PIN                                GPIO_PIN_7
#define LED_GPIO_PORT                          GPIOB
#define LED_GPIO_CLK_ENABLE()                  __HAL_RCC_GPIOB_CLK_ENABLE()
