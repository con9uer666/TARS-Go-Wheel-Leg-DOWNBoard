#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "memorymap.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

#include "BMI088driver.h"
#include "remoter.h"

//extern float gyro[3],accel[3],temperate;


//void IMU_tasks(void const * argument)
//{
//	BMI088_init();
//  for(;;)
//  {
//		BMI088_read(gyro,accel,&temperate);
//    osDelay(1);
//  }
//}
