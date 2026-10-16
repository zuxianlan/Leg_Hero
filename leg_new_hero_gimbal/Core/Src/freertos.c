/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "INS_task.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for INS_TASK */
osThreadId_t INS_TASKHandle;
const osThreadAttr_t INS_TASK_attributes = {
  .name = "INS_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Gimbal_TASK */
osThreadId_t Gimbal_TASKHandle;
const osThreadAttr_t Gimbal_TASK_attributes = {
  .name = "Gimbal_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Other_TASK */
osThreadId_t Other_TASKHandle;
const osThreadAttr_t Other_TASK_attributes = {
  .name = "Other_TASK",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Shoot_TASK */
osThreadId_t Shoot_TASKHandle;
const osThreadAttr_t Shoot_TASK_attributes = {
  .name = "Shoot_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Vision_TASK */
osThreadId_t Vision_TASKHandle;
const osThreadAttr_t Vision_TASK_attributes = {
  .name = "Vision_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Can_Comm */
osThreadId_t Can_CommHandle;
const osThreadAttr_t Can_Comm_attributes = {
  .name = "Can_Comm",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RC_TASK */
osThreadId_t RC_TASKHandle;
const osThreadAttr_t RC_TASK_attributes = {
  .name = "RC_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void INS_Task(void *argument);
extern void Gimbal_Task(void *argument);
extern void Other_Task(void *argument);
extern void Shoot_Task(void *argument);
extern void Vision_Task(void *argument);
extern void Can_Comm_Task(void *argument);
extern void RC_Task(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of INS_TASK */
  INS_TASKHandle = osThreadNew(INS_Task, NULL, &INS_TASK_attributes);

  /* creation of Gimbal_TASK */
  Gimbal_TASKHandle = osThreadNew(Gimbal_Task, NULL, &Gimbal_TASK_attributes);

  /* creation of Other_TASK */
  Other_TASKHandle = osThreadNew(Other_Task, NULL, &Other_TASK_attributes);

  /* creation of Shoot_TASK */
  Shoot_TASKHandle = osThreadNew(Shoot_Task, NULL, &Shoot_TASK_attributes);

  /* creation of Vision_TASK */
  Vision_TASKHandle = osThreadNew(Vision_Task, NULL, &Vision_TASK_attributes);

  /* creation of Can_Comm */
  Can_CommHandle = osThreadNew(Can_Comm_Task, NULL, &Can_Comm_attributes);

  /* creation of RC_TASK */
  RC_TASKHandle = osThreadNew(RC_Task, NULL, &RC_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

