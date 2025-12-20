
// #include "UserFreertos.h"
// #include "main.h"
// #include "Board2Board.h"
// #include  "tim.h"
// #include "detect.h"
// #include  "user_can.h"
// #include "fdcan.h"
// //#include  "ins_task.h"
// //freertos测试程序 绿灯闪烁 上报任务占有率
// /************仅供测试使用，在产品发布时应关闭**************/
// /*
//     相关宏定义configGENERATE_RUN_TIME_STATS  
//               configUSE_STATS_FORMATTING_FUNCTIONS  
//               configUSE_TRACE_FACILITY
//     相关函数  configureTimerForRunTimeStats()   
//               getRunTimeCounterValue()    
//               vTaskListv((char *)&CPU_RunInfo)
//               TaskGetRunTimeStats((char *)&CPU_RunInfo)
//               HAL_TIM_PeriodElapsedCallback(&htim5)  
//     相关变量  RUN_Time
//               CPU_RunInfo
// */

// //uint32_t RUN_Time=0;

// //void configureTimerForRunTimeStats(void)
// //{
// //  RUN_Time=0;
// //	HAL_TIM_Base_Start_IT(&htim5);
// //}
// //unsigned long getRunTimeCounterValue(void)
// //{
// //return RUN_Time;
// //}

// //  uint8_t CPU_RunInfo[400];

// // DMA 传输完成回调函数
// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
// {
//     HAL_TIM_PWM_Stop_DMA(&htim3,TIM_CHANNEL_2);
// }
// uint16_t LEDGREEN[104] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
// 												 199,199,199,199,199,199,199,199,
//                          99,99,99,99,99,99,99,99,
//                          99,99,99,99,99,99,99,99,
// 		};

// uint16_t LEDRED[104] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
// 												 99,99,99,99,99,99,99,99,
//                          199,199,199,199,199,199,199,199,
//                          99,99,99,99,99,99,99,99,
// 		};

// uint16_t LEDTurnOff[104] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                          0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//                          99,99,99,99,99,99,99,99,
//                          99,99,99,99,99,99,99,99,
//                          99,99,99,99,99,99,99,99,
// 		};		
		
// void OS_LedCallback(void const * argument)
// {
  
//   for(;;)
//   { 

// 		 HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *)LEDGREEN,(104));    
	
		
//     osDelay(500);

//      HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *)LEDTurnOff,(104));
		
// 		 osDelay(500);
	
//     /**************上报任务占有情况******************/
// //    memset(CPU_RunInfo,0,400); //信息缓冲区清零
// //    vTaskList((char *)&CPU_RunInfo); //获取任务运行时间信息
// //    usb_printf("任务名   任务状态   优先级   剩余栈   任务序号\r\n");
// //    usb_printf("%s", CPU_RunInfo);
// //
// //    
// //    memset(CPU_RunInfo,0,400); //信息缓冲区清零
// //    vTaskGetRunTimeStats((char *)&CPU_RunInfo);
// //    usb_printf("任务名         运行计数         使用率\r\n");
// //    usb_printf("%s", CPU_RunInfo);
// //    osDelay(500);
//   }
// }

// //错误处理(急停)任务 
// /*
//  无os_delay最高优先级 在程序正常运行时不应该被调用 
//  在恢复执行时占据全部时间片饿死其他任务
// */



		
// void OS_ErrorCallback(void const * argument)
// {
// 	B2B_Init();
//  osThreadSuspend(ErrorTaskHandle); //第一次执行挂起自身 
// 	HAL_Delay(10);
// 	USER_CAN_SetMotorCurrent(&hfdcan2, 0x144, 0x8000, 0,
// 					0, 0);
// 	HAL_Delay(10);
// 	USER_CAN_SetMotorCurrent(&hfdcan2, 0x145, 0x8000, 0,
// 					0, 0);
// 	HAL_Delay(10);
// 	USER_CAN_SetMotorCurrent(&hfdcan2, 0x141, 0x8000, 0,
// 					0, 0);
// 	HAL_Delay(1);
// 	USER_CAN_SendCapData(&hfdcan1,0x1aa,0,0,0);
// 	HAL_Delay(1);

	
//  for(;;)
//  {
// 		B2B_Init();
// 		USER_CAN_SetMotorCurrent(&hfdcan1,0x200,0,0,0,0);//关断电机	
// 		HAL_Delay(1);
// 		USER_CAN_SendCapData(&hfdcan3,0x1aa,0,0,0);
// 		HAL_Delay(1);
// 		FEEDBACK = 1;
// 		B2B_ParseUsart();
// 		if(STOPFLAG != 1 && detectList[DeviceID_B2B].isLost != 1)        
//      HAL_NVIC_SystemReset();  //右拨杆回到中间重启系统
//    //红灯闪烁    

// 		HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *)LEDRED,(104)); 
//    HAL_Delay(75);

//    HAL_TIM_PWM_Start_DMA(&htim3,TIM_CHANNEL_2,(uint32_t *)LEDTurnOff,(104));
//    HAL_Delay(75);

//  }                                 //无os_delay 最高优先级 
// }

