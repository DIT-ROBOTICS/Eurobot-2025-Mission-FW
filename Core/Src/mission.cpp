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
extern int32_t mission_type;
extern int32_t mission_status;
extern int32_t task_created;
extern int32_t task_created_1;
extern int32_t task_created_2;
extern int32_t task_created_3;
extern int32_t task_created_7;
extern int32_t task_created_8;
extern int32_t task_created_9;
/* extern variable */

void mission_init(void){
    motion_init();
}

void mission_initial_pose(void* pvParameters){
    front_wood_push_close();
    dick_initial_pose();
    front_lift_to_top();
    front_wood_initial_pose();
    front_lift_to_bottom();
    motion_initial_pose();
}

void robot_start_1(void* pvParameter){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    air_pump_on();
    back_out_start_pos();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void robot_stop_0(void* pvParameter){
    mission_status = 0 + 10*mission_type;

    /* add motion here */
    air_pump_off();
    mission_initial_pose(NULL);
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_grab_11(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_grab();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_1 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_side_support_12(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_side_support();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_1 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_wood_push_13(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_1 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_wood_push_close_14(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_1 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_side_support_close_15(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_side_support_close();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_1 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_release_10(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_release();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_1 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_out_close_25(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_out_close();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_2 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_release_outer_24(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_out_close();
    back_small_arm_release();
    back_out_release();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_2 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_small_arm_close_23(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_small_arm_release();
    back_small_arm_close();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_2 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_small_arm_open_22(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_small_arm_open();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_2 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_grab_21(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_in_grab();
    back_small_arm_grab();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_2 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_release_20(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_out_open();
    back_in_release();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_2 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_top_short_35(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_dick_release_pose();
    dick_initial_pose();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_3 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_bot_higher_34(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_bottom_higher();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_3 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_mid_lower_33(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_middle_lower();
    front_release();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_3 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_top_32(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_top();
    // front_wood_push();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_3 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_one_layer_31(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_middle();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_3 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_bottom_30(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_bottom();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_3 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_wood_takein_71(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_top();
    front_wood_grab();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_7 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_wood_takeout_70(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_wood_push_close();
    front_lift_to_top();
    front_wood_release();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_7 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_arm_reset_80(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_lift_to_middle();
    front_arm_reset();
    front_lift_to_bottom();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_8 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}
void front_put_banner_81(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_banner_release();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_8 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void front_mag_valve_disable_90(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    front_mag_valve_off();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_9 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}

void back_mag_valve_disable_91(void* pvParameters){
    mission_status = 0 + 10*mission_type;
    
    /* add motion here */
    back_mag_valve_off();
    /* add motion here */

    mission_status = 1 + 10*mission_type;
    task_created_9 = 0;
    vTaskDelete(NULL);  // Delete current task when mission is complete
}