/*
 * mission.cpp
 *
 *  Created on: Mar 2, 2025
 *      Author: stanly
 */


#include "mission.h"

#include "motion.h"
#include "stm32f4xx_hal.h"
#include "Servo.h"
#include "cmsis_os.h"


/* extern variable */
extern int32_t mission_status;
extern int32_t task_created;
/* extern variable */

void mission_init(void){
    motion_init();
}

void mission_initial_pose(void* pvParameters){
    front_lift_to_top();
    front_wood_release();
    front_lift_to_bottom();
    motion_initial_pose();
}

void front_grab_11(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_grab();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_release_10(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_release();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_release_outer_24(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    back_out_close();
    back_small_arm_release();
    back_out_release();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_small_arm_close_23(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    back_small_arm_release();
    back_small_arm_close();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_small_arm_open_22(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    back_small_arm_open();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_grab_21(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    back_in_grab();
    back_small_arm_grab();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_release_20(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    back_out_open();
    back_in_release();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_mid_lower_33(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_lift_to_middle_lower();
    front_release();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_top_32(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_lift_to_top();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_one_layer_31(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_lift_to_middle();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_bottom_30(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_lift_to_bottom();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_wood_takein_71(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_lift_to_top();
    front_wood_grab();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_wood_takeout_70(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_lift_to_top();
    front_wood_release();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_mag_valve_disable_90(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    front_mag_valve_off();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_mag_valve_disable_91(void* pvParameters){
    mission_status = 0;
    
    /* add motion here */
    back_mag_valve_off();
    /* add motion here */

    mission_status = 1;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}