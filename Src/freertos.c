/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "fonts.h"
#include "ssd1306.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern FontDef font_7x12_RU;
extern FontDef font_7x12;
extern FontDef font_8x15_RU;
extern FontDef font_8x14;
extern FontDef font_5x10_RU;
extern FontDef font_5x10;

uint8_t error_crc = 0;
uint8_t status = 0;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId myTask09Handle;
osThreadId myTask11Handle;
osThreadId myTask13Handle;
osThreadId myTask14Handle;
osThreadId myTask17Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Lights_Task(void const * argument);
void Display_Task(void const * argument);
void Modbus_Receive_Task(void const * argument);
void Modbus_Transmit_Task(void const * argument);
void Data_Storage_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
void vApplicationIdleHook(void);
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/* USER CODE BEGIN 2 */
__weak void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 4 */
__weak void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of myTask09 */
  osThreadDef(myTask09, Lights_Task, osPriorityNormal, 0, 128);
  myTask09Handle = osThreadCreate(osThread(myTask09), NULL);

  /* definition and creation of myTask11 */
  osThreadDef(myTask11, Display_Task, osPriorityNormal, 0, 128);
  myTask11Handle = osThreadCreate(osThread(myTask11), NULL);

  /* definition and creation of myTask13 */
  osThreadDef(myTask13, Modbus_Receive_Task, osPriorityNormal, 0, 128);
  myTask13Handle = osThreadCreate(osThread(myTask13), NULL);

  /* definition and creation of myTask14 */
  osThreadDef(myTask14, Modbus_Transmit_Task, osPriorityNormal, 0, 128);
  myTask14Handle = osThreadCreate(osThread(myTask14), NULL);

  /* definition and creation of myTask17 */
  osThreadDef(myTask17, Data_Storage_Task, osPriorityNormal, 0, 128);
  myTask17Handle = osThreadCreate(osThread(myTask17), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
	
	Task_manager_Init();
  /* Infinite loop */
  for(;;)
  {
		Task_manager_LoadCPU();		
		
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Lights_Task */
/**
* @brief Function implementing the myTask09 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Lights_Task */
void Lights_Task(void const * argument)
{
  /* USER CODE BEGIN Lights_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END Lights_Task */
}

/* USER CODE BEGIN Header_Display_Task */
/**
* @brief Function implementing the myTask11 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Display_Task */
void Display_Task(void const * argument)
{
  /* USER CODE BEGIN Display_Task */
	uint8_t temp_stat = 0;
	char buffer[64];
	// CS# (This pin is the chip select input. (active LOW))
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	
	ssd1306_Init();
	
  /* Infinite loop */
  for(;;)
  {		
		
				if(error_crc == 1)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Ошибка",font_8x15_RU,1);
					ssd1306_SetCursor(0,15);
					ssd1306_WriteString("CRC",font_8x14,1);				
					
//					ssd1306_SetCursor(30,15);				
//					snprintf(buffer, sizeof buffer, "%d", boot_timer_counter);				
//					ssd1306_WriteString(buffer,font_8x14,1);	
					ssd1306_UpdateScreen();		
				}
				
				if(error_crc == 0 && status == 0)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Загруз",font_8x15_RU,1);
					ssd1306_WriteString("-",font_8x14,1);
					ssd1306_SetCursor(0,15);				
					ssd1306_WriteString("чик",font_8x15_RU,1);

//					ssd1306_SetCursor(0,30);				
					snprintf(buffer, sizeof buffer, " %.01f", VERSION);				
					ssd1306_WriteString(buffer,font_8x14,1);	
									
					
//					ssd1306_SetCursor(30,15);				
//					snprintf(buffer, sizeof buffer, "%d", boot_timer_counter);				
//					ssd1306_WriteString(buffer,font_8x14,1);	
					ssd1306_UpdateScreen();		
				}
				
				if (status == 1)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("FLASH",font_8x14,1);
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("очищена",font_8x15_RU,1);					
						
					ssd1306_UpdateScreen();	
				}

				if (status == 2)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,0);
					ssd1306_WriteString("Обнов",font_8x15_RU,1);					
					ssd1306_WriteString("-",font_8x14,1);					
					ssd1306_SetCursor(0,15);	
					ssd1306_WriteString("ление",font_8x15_RU,1);					
					ssd1306_SetCursor(0,30);	
					ssd1306_WriteString("ПО",font_8x15_RU,1);					
					ssd1306_WriteString("...",font_8x14,1);
						
					ssd1306_UpdateScreen();	
				}
				
				if (status == 3)
				{
					ssd1306_Fill(0);
					ssd1306_SetCursor(0,15);
					ssd1306_WriteString("УСПЕШНО",font_8x15_RU,1);										
						
					ssd1306_UpdateScreen();	
				}				
			
	
			osDelay(100);
  }
  /* USER CODE END Display_Task */
}

/* USER CODE BEGIN Header_Modbus_Receive_Task */
/**
* @brief Function implementing the myTask13 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modbus_Receive_Task */
void Modbus_Receive_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Receive_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Modbus_Receive_Task */
}

/* USER CODE BEGIN Header_Modbus_Transmit_Task */
/**
* @brief Function implementing the myTask14 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Modbus_Transmit_Task */
void Modbus_Transmit_Task(void const * argument)
{
  /* USER CODE BEGIN Modbus_Transmit_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Modbus_Transmit_Task */
}

/* USER CODE BEGIN Header_Data_Storage_Task */
/**
* @brief Function implementing the myTask17 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Data_Storage_Task */
void Data_Storage_Task(void const * argument)
{
  /* USER CODE BEGIN Data_Storage_Task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Data_Storage_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
