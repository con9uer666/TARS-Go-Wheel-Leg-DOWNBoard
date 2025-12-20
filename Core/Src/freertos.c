/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId IMUHandle;
uint32_t IMUBuffer[ 4096 ];
osStaticThreadDef_t IMUControlBlock;
osThreadId CANHandle;
osThreadId LEDHandle;
osThreadId ErrorHandle;
osThreadId State_machineHandle;
osThreadId MotorHandle;
osThreadId ObserveHandle;
uint32_t ObserveBuffer[ 2048 ];
osStaticThreadDef_t ObserveControlBlock;
osThreadId Board2BoardTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void IMU_tasks(void const * argument);
void CAN_Transmit(void const * argument);
void LED_task(void const * argument);
void Error_task(void const * argument);
void State_machine_task(void const * argument);
void Motor_task(void const * argument);
void Observe_Tasks(void const * argument);
void OS_Board2BoardCallback(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of IMU */
  osThreadStaticDef(IMU, IMU_tasks, osPriorityHigh, 0, 4096, IMUBuffer, &IMUControlBlock);
  IMUHandle = osThreadCreate(osThread(IMU), NULL);

  /* definition and creation of CAN */
  osThreadDef(CAN, CAN_Transmit, osPriorityAboveNormal, 0, 512);
  CANHandle = osThreadCreate(osThread(CAN), NULL);

  /* definition and creation of LED */
  osThreadDef(LED, LED_task, osPriorityBelowNormal, 0, 128);
  LEDHandle = osThreadCreate(osThread(LED), NULL);

  /* definition and creation of Error */
  osThreadDef(Error, Error_task, osPriorityRealtime, 0, 128);
  ErrorHandle = osThreadCreate(osThread(Error), NULL);

  /* definition and creation of State_machine */
  osThreadDef(State_machine, State_machine_task, osPriorityNormal, 0, 128);
  State_machineHandle = osThreadCreate(osThread(State_machine), NULL);

  /* definition and creation of Motor */
  osThreadDef(Motor, Motor_task, osPriorityNormal, 0, 1024);
  MotorHandle = osThreadCreate(osThread(Motor), NULL);

  /* definition and creation of Observe */
  osThreadStaticDef(Observe, Observe_Tasks, osPriorityHigh, 0, 2048, ObserveBuffer, &ObserveControlBlock);
  ObserveHandle = osThreadCreate(osThread(Observe), NULL);

  /* definition and creation of Board2BoardTask */
  osThreadDef(Board2BoardTask, OS_Board2BoardCallback, osPriorityHigh, 0, 512);
  Board2BoardTaskHandle = osThreadCreate(osThread(Board2BoardTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_IMU_tasks */
/**
* @brief Function implementing the IMU thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IMU_tasks */
__weak void IMU_tasks(void const * argument)
{
  /* USER CODE BEGIN IMU_tasks */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END IMU_tasks */
}

/* USER CODE BEGIN Header_CAN_Transmit */
/**
* @brief Function implementing the CAN thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_Transmit */
__weak void CAN_Transmit(void const * argument)
{
  /* USER CODE BEGIN CAN_Transmit */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END CAN_Transmit */
}

/* USER CODE BEGIN Header_LED_task */
/**
* @brief Function implementing the LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LED_task */
__weak void LED_task(void const * argument)
{
  /* USER CODE BEGIN LED_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LED_task */
}

/* USER CODE BEGIN Header_Error_task */
/**
* @brief Function implementing the Error thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Error_task */
__weak void Error_task(void const * argument)
{
  /* USER CODE BEGIN Error_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Error_task */
}

/* USER CODE BEGIN Header_State_machine_task */
/**
* @brief Function implementing the State_machine thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_State_machine_task */
__weak void State_machine_task(void const * argument)
{
  /* USER CODE BEGIN State_machine_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END State_machine_task */
}

/* USER CODE BEGIN Header_Motor_task */
/**
* @brief Function implementing the Motor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_task */
__weak void Motor_task(void const * argument)
{
  /* USER CODE BEGIN Motor_task */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Motor_task */
}

/* USER CODE BEGIN Header_Observe_Tasks */
/**
* @brief Function implementing the Observe thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Observe_Tasks */
__weak void Observe_Tasks(void const * argument)
{
  /* USER CODE BEGIN Observe_Tasks */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END Observe_Tasks */
}

/* USER CODE BEGIN Header_OS_Board2BoardCallback */
/**
* @brief Function implementing the Board2BoardTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_OS_Board2BoardCallback */
__weak void OS_Board2BoardCallback(void const * argument)
{
  /* USER CODE BEGIN OS_Board2BoardCallback */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END OS_Board2BoardCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
