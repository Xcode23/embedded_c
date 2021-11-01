#include "stm32f2xx_hal.h"
#include "stm32f2xx.h"
#include <stdint.h>
#include <string.h>

void GPIO_Init(void);
void UART_Init(void);
void UART_SendMessage(char* message);
void ADC_Init(void);
void DMA_LED_Init(void);
void DMA_UART_RX_Init(void);
void DMA_ADC_Init(void);
void DMA_UART_TX_Init(void);
void DMA_Transfer_Complete(DMA_HandleTypeDef *pHandle);

void ADC_Init(void);

DMA_HandleTypeDef hdma_led;
DMA_HandleTypeDef hdma_uart_rx;
DMA_HandleTypeDef hdma_uart_tx;
DMA_HandleTypeDef hdma_adc1;
UART_HandleTypeDef huart3;
ADC_HandleTypeDef hadc;

uint32_t buffer[4];

int main(void){
  HAL_Init();
  GPIO_Init();
  UART_Init();
  DMA_UART_TX_Init();
  DMA_LED_Init();
  ADC_Init();
  DMA_ADC_Init();
  while(1){
    uint8_t value = (0x1 << 7);
    HAL_DMA_Start_IT(&hdma_led, (uint32_t)&value, (uint32_t)&(GPIOB->ODR), 1);
    uint32_t current_ticks = HAL_GetTick();
		while( (current_ticks + 200 ) >= HAL_GetTick() );
    value = 0;
    HAL_DMA_Start_IT(&hdma_led, (uint32_t)&value, (uint32_t)&(GPIOB->ODR), 1);
    current_ticks = HAL_GetTick();
		while( (current_ticks + 200 ) >= HAL_GetTick() );

    HAL_ADC_Start_DMA(&hadc, buffer, 4);
    // uint8_t message[10];
    // HAL_UART_Receive_DMA(&huart3, message,10);
  }
}

void GPIO_Init(void){
  //PB7 LED2
  //PC13 user button
  
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef gpio_struct;
  gpio_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_struct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &gpio_struct);

  __HAL_RCC_GPIOC_CLK_ENABLE();
  gpio_struct.Mode = GPIO_MODE_INPUT;
  gpio_struct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &gpio_struct);

  EXTI->IMR |= (0x1 << 13);
  EXTI->RTSR |= (0x1 << 13);

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  SYSCFG->EXTICR[3] &= ~(0xf << 4);
  SYSCFG->EXTICR[3] |= (0x2 << 4);

  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /**USART3 GPIO Configuration
  PD8     ------> USART3_TX
  PD9     ------> USART3_RX
  */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  gpio_struct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  gpio_struct.Mode = GPIO_MODE_AF_PP;
  gpio_struct.Pull = GPIO_NOPULL;
  gpio_struct.Speed = GPIO_SPEED_FREQ_LOW;
  gpio_struct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &gpio_struct);
}

void ADC_Init(void){
  __HAL_RCC_ADC1_CLK_ENABLE();
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.ScanConvMode = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.NbrOfConversion = 16;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc);

  ADC_ChannelConfTypeDef chanconf;
  chanconf.Channel = ADC_CHANNEL_TEMPSENSOR;
  chanconf.Rank = 1;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 2;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 3;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 4;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 5;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 6;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 7;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 8;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 9;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 10;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 11;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 12;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 13;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 14;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 15;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
  chanconf.Rank = 16;
  chanconf.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &chanconf);
}

void DMA_LED_Init(void){
  __HAL_RCC_DMA2_CLK_ENABLE();
  hdma_led.Instance = DMA2_Stream0;
  hdma_led.Init.Channel = DMA_CHANNEL_0;
  hdma_led.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_led.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_led.Init.MemInc = DMA_MINC_DISABLE;
  hdma_led.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_led.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_led.Init.Mode = DMA_NORMAL;
  hdma_led.Init.Priority = DMA_PRIORITY_LOW;
  hdma_led.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_led.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_led.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_led.Init.PeriphBurst = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&hdma_led);

  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

void DMA_UART_RX_Init(void){
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_uart_rx.Instance = DMA1_Stream1;
  hdma_uart_rx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_uart_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart_rx.Init.Mode = DMA_NORMAL;
  hdma_uart_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_uart_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_uart_rx);

  __HAL_LINKDMA(&huart3, hdmarx, hdma_uart_rx);

  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
}

void DMA_ADC_Init(void){
  __HAL_RCC_ADC1_CLK_ENABLE();
  
    /* ADC1 DMA Init */
    /* ADC1 Init */
  hdma_adc1.Instance = DMA2_Stream4;
  hdma_adc1.Init.Channel = DMA_CHANNEL_0;
  hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  hdma_adc1.Init.Mode = DMA_NORMAL;
  hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
  hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
  hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
  hdma_adc1.Init.MemBurst = DMA_MBURST_SINGLE;
  hdma_adc1.Init.PeriphBurst = DMA_PBURST_SINGLE;
  HAL_DMA_Init(&hdma_adc1);
  __HAL_LINKDMA(&hadc,DMA_Handle,hdma_adc1);
  HAL_NVIC_EnableIRQ(DMA2_Stream4_IRQn);
}

void DMA_UART_TX_Init(void){
  __HAL_RCC_DMA1_CLK_ENABLE();
  hdma_uart_tx.Instance = DMA1_Stream3;
  hdma_uart_tx.Init.Channel = DMA_CHANNEL_4;
  hdma_uart_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  hdma_uart_tx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_uart_tx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_uart_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_uart_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_uart_tx.Init.Mode = DMA_CIRCULAR;
  hdma_uart_tx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_uart_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_uart_tx);

  __HAL_LINKDMA(&huart3, hdmatx, hdma_uart_tx);

  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
}

void UART_SendMessage(char* message){
  HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
}

void UART_Init(void){
  __HAL_RCC_USART3_CLK_ENABLE();
  
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart3);
}

void SysTick_Handler(void){
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void DMA2_Stream0_IRQHandler(void){
  HAL_DMA_IRQHandler(&hdma_led);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart_rx);
}

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart_tx);
}

void DMA2_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_adc1);
}

uint8_t calc_temp(uint32_t value){
  value = (value - 1) / 3 + 25;
  return (uint8_t)value;
}

void EXTI15_10_IRQHandler(void){
  char message[64];
  sprintf(message, "Temp: %d\n", calc_temp(buffer[0]));
  HAL_UART_Transmit_DMA(&huart3, (uint8_t*)message, strlen(message));
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  HAL_DMA_DeInit(&hdma_uart_tx);
  HAL_UART_DeInit(&huart3);
  HAL_UART_Init(&huart3);
  HAL_DMA_Init(&hdma_uart_tx);
  __HAL_LINKDMA(&huart3, hdmatx, hdma_uart_tx);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(hadc);
  /* NOTE : This function Should not be modified, when the callback is needed,
            the HAL_ADC_ConvCpltCallback could be implemented in the user file
   */
}

void DMA_Transfer_Complete(DMA_HandleTypeDef *pHandle){

}