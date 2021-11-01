#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "stm32f2xx.h"

//LOW POWER
//SleepOnExit/WFI***
//lower clock speed
//higher uart baudrate
//turn off peripherals rcc during sleep(sleep_disable)***
//gpio unused pins in analog mode***

void basic_hal_uart_usb(void);
void UART3_Init(void);
void TIM6_Init(void);
void CAN1_Init(void);
void CAN1_Send(void);
void CAN1_Receive(void);
void RTC_Init(void);
void clock_hse_init(void);
void clock_pll_init(void);
void Alarm_Config(void);

UART_HandleTypeDef huart3;
RCC_OscInitTypeDef oscillator_config;
RCC_ClkInitTypeDef clock_config;
TIM_HandleTypeDef htim6;
CAN_HandleTypeDef hcan1;
RTC_HandleTypeDef hrtc;
RCC_OscInitTypeDef rtc_osc;
RCC_PeriphCLKInitTypeDef rtc_clk;

uint8_t rtc_alarm_second_counter = 10;

int main(){
    basic_hal_uart_usb();
}

void basic_hal_uart_usb(void){
  HAL_Init();
  clock_pll_init();
  RTC_Init();
  Alarm_Config();
  UART3_Init();
  TIM6_Init();
  HAL_TIM_Base_Start_IT(&htim6);


  char user_data[200];
  memset(user_data, 0, sizeof(user_data));
  sprintf(user_data, "SYSCLK : %ldHz\r\n",HAL_RCC_GetSysClockFreq());
  HAL_UART_Transmit(&huart3, (uint8_t*)user_data, strlen(user_data), HAL_MAX_DELAY);
  memset(user_data, 0, sizeof(user_data));
  sprintf(user_data, "HCLK : %ldHz\r\n",HAL_RCC_GetHCLKFreq());
  HAL_UART_Transmit(&huart3, (uint8_t*)user_data, strlen(user_data), HAL_MAX_DELAY);
  memset(user_data, 0, sizeof(user_data));
  sprintf(user_data, "PCLK1 : %ldHz\r\n",HAL_RCC_GetPCLK1Freq());
  HAL_UART_Transmit(&huart3, (uint8_t*)user_data, strlen(user_data), HAL_MAX_DELAY);
  memset(user_data, 0, sizeof(user_data));
  sprintf(user_data, "PCLK2 : %ldHz\r\n",HAL_RCC_GetPCLK2Freq());
  HAL_UART_Transmit(&huart3, (uint8_t*)user_data, strlen(user_data), HAL_MAX_DELAY);

  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_SLEEP_DISABLE();
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  CAN1_Init();

  CAN1_Send();

  CAN1_Receive();

  // __HAL_RCC_CAN1_CLK_DISABLE();
  // __HAL_RCC_GPIOA_CLK_DISABLE();

  //HAL_PWR_EnableSleepOnExit();

  while (1){
    //for(int i=0; i<500000;i++);
    //HAL_UART_Transmit(&huart3, (uint8_t*)user_data2, strlen(user_data2), HAL_MAX_DELAY);
  }
}

void RTC_Init(void){
  //Startup RTC clock
  rtc_osc.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  rtc_osc.LSEState = RCC_LSE_ON;
  rtc_osc.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&rtc_osc);

  rtc_clk.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  rtc_clk.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  HAL_RCCEx_PeriphCLKConfig(&rtc_clk);

  __HAL_RCC_RTC_ENABLE();
  HAL_NVIC_SetPriority(RTC_Alarm_IRQn,15,0);
  HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);

  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_12;
  //LSE 32.768 kHz has to be devided to 1 Hz
  hrtc.Init.AsynchPrediv = 0x7F;//127+1
  hrtc.Init.SynchPrediv = 0xFF;//255+1
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_LOW;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;


  //config RTC clock
  HAL_RTC_Init(&hrtc);

  RTC_TimeTypeDef rtc_time;
  rtc_time.Hours = 10;
  rtc_time.Minutes = 30;
  rtc_time.Seconds = 0;
  rtc_time.TimeFormat = RTC_HOURFORMAT12_AM;

  HAL_RTC_SetTime(&hrtc, &rtc_time, RTC_FORMAT_BIN);

  RTC_DateTypeDef rtc_date;
  rtc_date.Date = 18;
  rtc_date.Month = RTC_MONTH_NOVEMBER;
  rtc_date.Year = 20;
  rtc_date.WeekDay = RTC_WEEKDAY_WEDNESDAY;

  HAL_RTC_SetDate(&hrtc, &rtc_date, RTC_FORMAT_BIN);

  //Alarm_Config();
}

void Alarm_Config(void){
  RTC_AlarmTypeDef rtc_alarm;
  memset(&rtc_alarm, 0, sizeof(rtc_alarm));
  HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

  //alarm every 2 seconds
  rtc_alarm.Alarm = RTC_ALARM_A;
  rtc_alarm.AlarmTime.Seconds = rtc_alarm_second_counter;
  rtc_alarm.AlarmTime.TimeFormat = RTC_FORMAT_BIN;
  rtc_alarm.AlarmMask = RTC_ALARMMASK_HOURS | RTC_ALARMMASK_MINUTES;
  rtc_alarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  rtc_alarm.AlarmDateWeekDay = RTC_WEEKDAY_WEDNESDAY;
  HAL_RTC_SetAlarm_IT(&hrtc, &rtc_alarm, RTC_FORMAT_BIN);

  rtc_alarm_second_counter = (rtc_alarm_second_counter + 2) % 60;
}

void CAN1_Send(void){

  hcan1.pTxMsg = (CanTxMsgTypeDef*)malloc(sizeof(CanTxMsgTypeDef));

  hcan1.pTxMsg->DLC = 6;
  hcan1.pTxMsg->IDE = CAN_ID_STD;
  hcan1.pTxMsg->RTR = CAN_RTR_DATA;
  hcan1.pTxMsg->StdId = 0x9D;
  hcan1.pTxMsg->Data[0] = 'H';
  hcan1.pTxMsg->Data[1] = 'E';
  hcan1.pTxMsg->Data[2] = 'L';
  hcan1.pTxMsg->Data[3] = 'L';
  hcan1.pTxMsg->Data[4] = 'O';
  hcan1.pTxMsg->Data[5] = '3';

  HAL_CAN_Transmit(&hcan1, HAL_MAX_DELAY);

  free(hcan1.pTxMsg);
}

void CAN1_Receive(void){
  hcan1.pRxMsg = (CanRxMsgTypeDef*)malloc(sizeof(CanRxMsgTypeDef));
  HAL_CAN_Receive(&hcan1, 0, HAL_MAX_DELAY);
  char user_data[128];
  hcan1.pRxMsg->Data[hcan1.pRxMsg->DLC]=0;
  sprintf(user_data, "Message received: %s\n",hcan1.pRxMsg->Data);
  free(hcan1.pRxMsg);
  HAL_UART_Transmit(&huart3, (uint8_t*)user_data, strlen(user_data), HAL_MAX_DELAY);
}

void CAN1_Init(void){
  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**CAN1 GPIO Configuration
  PA11     ------> CAN1_RX
  PA12     ------> CAN1_TX
  */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  //don't put PA13/PA14 in analog mode because they are required for STLINK
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  

  hcan1.Instance = CAN1;
  hcan1.Init.Mode = CAN_MODE_SILENT_LOOPBACK;
  hcan1.Init.ABOM = ENABLE;
  hcan1.Init.NART = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.TXFP = DISABLE;

  //can timing configs
  hcan1.Init.Prescaler = 5;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_8TQ;
  hcan1.Init.BS2 = CAN_BS2_1TQ;

  HAL_CAN_Init(&hcan1);

  //filter configs
  CAN_FilterConfTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.BankNumber = 0;
  filter.FilterNumber = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  HAL_CAN_ConfigFilter(&hcan1, &filter);
}

void UART3_Init(void){
  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_USART3_CLK_SLEEP_DISABLE();

  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_SLEEP_DISABLE();
  /**USART3 GPIO Configuration
  PD8     ------> USART3_TX
  PD9     ------> USART3_RX
  */
  GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  
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

void TIM6_Init(void){
  __HAL_RCC_TIM6_CLK_ENABLE();
  htim6.Instance = TIM6;
  htim6.Init.Prescaler=5000;
  htim6.Init.Period = 50000-1;

  HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn,15,0);

  HAL_TIM_Base_Init(&htim6);
}

void clock_pll_init(void){
  //configure oscillators
  memset(&oscillator_config,0,sizeof(oscillator_config));
  oscillator_config.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  oscillator_config.HSIState = RCC_HSI_ON;
  oscillator_config.HSICalibrationValue = 16;
  oscillator_config.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  oscillator_config.PLL.PLLState = RCC_PLL_ON;
  oscillator_config.PLL.PLLM = 16;
  oscillator_config.PLL.PLLN = 100;
  oscillator_config.PLL.PLLP = 2;
  oscillator_config.PLL.PLLQ = 2;
  HAL_RCC_OscConfig(&oscillator_config);

  //configure SysClock
  clock_config.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clock_config.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clock_config.APB1CLKDivider = RCC_HCLK_DIV2;
  clock_config.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&clock_config, FLASH_ACR_LATENCY_1WS);

  //configure SysTick for 1 ms with new SysClock frquency
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void clock_hse_init(void){

  //configure oscillators
  memset(&oscillator_config,0,sizeof(oscillator_config));
  oscillator_config.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator_config.HSEState = RCC_HSE_BYPASS;
  HAL_RCC_OscConfig(&oscillator_config);

  //configure SysClock
  clock_config.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clock_config.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  clock_config.AHBCLKDivider = RCC_SYSCLK_DIV2;
  clock_config.APB1CLKDivider = RCC_HCLK_DIV2;
  clock_config.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&clock_config, FLASH_ACR_LATENCY_0WS);

  //configure SysTick for 1 ms with new SysClock frquency
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  __HAL_RCC_HSI_DISABLE(); //Saves some current
}

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

void TIM6_DAC_IRQHandler(void){
  HAL_TIM_IRQHandler(&htim6);
}

void RTC_Alarm_IRQHandler(void)
{
	HAL_RTC_AlarmIRQHandler(&hrtc);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc){
  
  char message[80];
  sprintf(message, "RTC alarm triggerd\n");
  HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

  RTC_TimeTypeDef now;
  RTC_DateTypeDef dummy;

  HAL_RTC_GetTime(hrtc, &now, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(hrtc, &dummy, RTC_FORMAT_BIN);

  sprintf(message, "Time: %02d:%02d:%02d\n", now.Hours, now.Minutes, now.Seconds);
  HAL_UART_Transmit(&huart3, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);

  Alarm_Config();
}