/*
 * rtos-function.cpp
 *
 *  Created on: Mar 1, 2025
 *      Author: stanly
 */

/*stm32 include*/
#include "stm32f446xx.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/*microROS include*/
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
/*user include*/
#include "mission.h"
// #include "motion.h"
#include "timers.h"
#include "uros_init.h"

/**************** stm32 variable ****************/
extern UART_HandleTypeDef huart1;
/**************** stm32 variable ****************/

TimerHandle_t xTimer;

void vTimerCallback( TimerHandle_t xTimer )
{
  servo_refresh_angle();
  magnet_valve_refresh();
  air_pump_refresh();
}

/**************** freertos callback ****************/
void StartDefaultTask(void *argument)
{
  /*init*/
  mission_init(); // servo init
  uros_init();
  // rtos timer for refreshing servo angle
  xTimer = xTimerCreate("Timer", pdMS_TO_TICKS(100), pdTRUE, (void *)0, vTimerCallback);
  xTimerStart(xTimer, 0);

  mission_initial_pose(NULL); // initial pose

  for(;;)
  {
    uros_agent_status_check();
  }
}
/**************** freertos callback ****************/