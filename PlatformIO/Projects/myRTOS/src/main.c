#include <stdint.h>
#include <string.h>
#include "stm32f2xx_hal.h"
#include <Uart3.h>

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/queue.h"
#include "FreeRTOS/semphr.h"

TaskHandle_t htask_button = NULL;
TaskHandle_t htask_led = NULL;
TaskHandle_t htask_log = NULL;
TaskHandle_t htask1 = NULL;
TaskHandle_t htask2 = NULL;
QueueHandle_t hqueue = NULL;

void Task1Handler(void* params);
void Task2Handler(void* params);
void LedTaskHandler(void* params);
void ButtonTaskHandler(void* params);
void LogTaskHandler(void* params);
void setupGPIOs(void);

char* message = "Pressed!\n"; 
xSemaphoreHandle mutex = NULL;

int main(void) {

  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // enable cycle counter (ARM specific)

  UART_Init();
  UART_SendMessage("Testing...\n");

  setupGPIOs();

  //start recording
  SEGGER_SYSVIEW_Conf();
  SEGGER_SYSVIEW_Start();

  hqueue = xQueueCreate(2, sizeof(char*));
  mutex = xSemaphoreCreateMutex();

  xTaskCreate(Task2Handler, "Task 2", configMINIMAL_STACK_SIZE, NULL, 1, &htask2);
  xTaskCreate(Task1Handler, "Task 1", configMINIMAL_STACK_SIZE, NULL, 1, &htask1);

  // xTaskCreate(ButtonTaskHandler, "Button Task", configMINIMAL_STACK_SIZE, NULL, 1, &htask_button);
  // xTaskCreate(LedTaskHandler, "LED Task", configMINIMAL_STACK_SIZE, NULL, 1, &htask_led);
  // xTaskCreate(LogTaskHandler, "Log Task", configMINIMAL_STACK_SIZE, NULL, 1, &htask_log);

  vTaskStartScheduler();

  while(1);
}

void Task1Handler(void* params){
  while(1){
    xSemaphoreTake(mutex, portMAX_DELAY);
    UART_SendMessage("Task1 says Hi!\n");
    SEGGER_SYSVIEW_Print("Task1 is working");
    xSemaphoreGive(mutex);
    //traceISR_EXIT_TO_SCHEDULER();//for cooperative scheduling
    taskYIELD();
  }
}

void Task2Handler(void* params){
  while(1){
    xSemaphoreTake(mutex, portMAX_DELAY);
    UART_SendMessage("Task2 says Hi!\n");
    SEGGER_SYSVIEW_Print("Task2 is working");
    xSemaphoreGive(mutex);
    //traceISR_EXIT_TO_SCHEDULER();//for cooperative scheduling
    taskYIELD();
  }
}

void LedTaskHandler(void* params){
  while(1){
    if(xTaskNotifyWait(0,0,NULL,portMAX_DELAY) == pdTRUE){
      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
    }
  }
}

void ButtonTaskHandler(void* params){
  while(1){
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13)){
      xQueueSend(hqueue, &message, portMAX_DELAY);
      xTaskNotify(htask_log, 0, eIncrement);
      xTaskNotify(htask_led, 0, eIncrement);
      vTaskDelay(200);
    }
  }
}

void LogTaskHandler(void* params){
  while(1){
    char* temp = NULL;
    if(xTaskNotifyWait(0,0,NULL,portMAX_DELAY) == pdTRUE){
      xQueueReceive(hqueue, &temp, portMAX_DELAY);
      UART_SendMessage(temp);
    }
  }
}

void setupGPIOs(void){
  //PB7 LED2
  //PC13 user button
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIO_InitTypeDef gpio_struct;
  gpio_struct.Mode = GPIO_MODE_OUTPUT_PP;
  gpio_struct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &gpio_struct);
  gpio_struct.Mode = GPIO_MODE_INPUT;
  gpio_struct.Pin = GPIO_PIN_13;
  HAL_GPIO_Init(GPIOC, &gpio_struct);
}