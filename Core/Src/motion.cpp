/*
 * motion.cpp
 *
 *  Created on: Mar 16, 2025
 *      Author: stanly
 */

#include "motion.h"

#include "stm32f4xx_hal.h"
#include "Servo.h"
#include "cmsis_os.h"

// linear servo angle 360~1440

/* extern variable */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim12;
/* extern variable */

/* servo declaration */
Servo F_LIFT_CASCADE_S1(&htim1, TIM_CHANNEL_1);
Servo F_LIFT_CASCADE_S2(&htim12, TIM_CHANNEL_1);
Servo F_PUSHING_SERVO(&htim12, TIM_CHANNEL_2);
Servo F_LIFTING_SERVO(&htim3, TIM_CHANNEL_1);
Servo F_GRAB_LINEAR_SERVO(&htim1, TIM_CHANNEL_4);
Servo F_GRAB_LOWER_S1(&htim3, TIM_CHANNEL_2);
Servo F_GRAB_LOWER_S2(&htim3, TIM_CHANNEL_3);

Servo BI_R_COL(&htim4, TIM_CHANNEL_1);
Servo BI_L_COL(&htim4, TIM_CHANNEL_2);
Servo BI_UP_DOMN_COL(&htim2, TIM_CHANNEL_2);
Servo BI_SMALL_LINEAR(&htim5, TIM_CHANNEL_1);

Servo BO_GRAB_LINEAR(&htim2, TIM_CHANNEL_3);
Servo BO_ROTATE(&htim2, TIM_CHANNEL_1);

Servo S_LINEAR_S1(&htim4, TIM_CHANNEL_3);
Servo S_LINEAR_S2(&htim4, TIM_CHANNEL_4);

Servo Backup(&htim3, TIM_CHANNEL_4);
/* servo declaration */

/*variable*/
int F_Lift_Cascade_angle = 1620; // front lift <1700 >>down
int F_Lift_Cascade_angle_prev = F_Lift_Cascade_angle;
int F_Lift_Cascade2_angle() { return (1800-F_Lift_Cascade_angle+20); } // front lift >100 >>up
int F_Pushing_angle = 500; // 取木板前後 out 1600 in 500
int F_Lifting_UpDown_angle = 1170; // 取木板上下 上1170 下900
int F_Grab_Linear_angle = 360; // 夾木板 open 800 close 500
int F_Grab_L2_angle = 1050; // 托木板 >900 down
int F_Grab_L1_angle() { return (1800-F_Grab_L2_angle+20); } // 托木板

int BI_R_COL_angle = 430; // >430 open
int BI_L_COL_angle() {return (1800-BI_R_COL_angle+10);}
int BI_UP_DOMN_COL_angle = 1500;
int BI_SMALL_LINEAR_angle = 400;

int BO_GRAB_LINEAR_angle = 400;
int BO_ROTATE_angle = 950; // <900 open

int S_LINEAR_S1_angle = 900;
int S_LINEAR_S2_angle = 900;

int Backup_angle = 900;

int F_MAG_VAL = 0;
int B_MAG_VAL = 0;
int B_OUT_MAG_VAL = 0;

int AIR_PUMP = 0;

static uint32_t f_magnet_start_time = 0;
const uint32_t F_MAGNET_TIMEOUT = 5000; // 5 seconds in milliseconds
static uint32_t b_magnet_start_time = 0;
const uint32_t B_MAGNET_TIMEOUT = 5000; // 5 seconds in milliseconds
static uint32_t b_out_magnet_start_time = 0;
const uint32_t B_OUT_MAGNET_TIMEOUT = 5000; // 5 seconds in milliseconds
/*variable*/

void motion_init(void){
    F_LIFT_CASCADE_S1.init();
    F_LIFT_CASCADE_S2.init();
    F_PUSHING_SERVO.init();
    F_LIFTING_SERVO.init();
    F_GRAB_LINEAR_SERVO.init();
    F_GRAB_LOWER_S1.init();
    F_GRAB_LOWER_S2.init();
    BI_R_COL.init();
    BI_L_COL.init();
    S_LINEAR_S1.init();
    S_LINEAR_S2.init();
    BI_UP_DOMN_COL.init();
    BI_SMALL_LINEAR.init();
    BO_GRAB_LINEAR.init();
    BO_ROTATE.init();
    Backup.init();
}

void motion_initial_pose(void){
    F_Lift_Cascade_angle = 1620;
    F_Pushing_angle = 500;
    F_Lifting_UpDown_angle = 1170;
    F_Grab_Linear_angle = 500;
    F_Grab_L2_angle = 1050;
    BI_R_COL_angle = 430;
    BI_UP_DOMN_COL_angle = 1500;
    BI_SMALL_LINEAR_angle = 400;
    BO_GRAB_LINEAR_angle = 400;
    BO_ROTATE_angle = 950;
    S_LINEAR_S1_angle = 900;
    S_LINEAR_S2_angle = 900;
    Backup_angle = 900;
    F_MAG_VAL = 0;
    B_MAG_VAL = 0;
    B_OUT_MAG_VAL = 0;
    AIR_PUMP = 0;
}

void servo_refresh_angle(void){
    F_LIFT_CASCADE_S1.setAngle(F_Lift_Cascade_angle);
    F_LIFT_CASCADE_S2.setAngle(F_Lift_Cascade2_angle());
    F_PUSHING_SERVO.setAngle(F_Pushing_angle);
    F_LIFTING_SERVO.setAngle(F_Lifting_UpDown_angle);
    F_GRAB_LINEAR_SERVO.setAngle(F_Grab_Linear_angle);
    F_GRAB_LOWER_S1.setAngle(F_Grab_L1_angle());
    F_GRAB_LOWER_S2.setAngle(F_Grab_L2_angle);
    BI_R_COL.setAngle(BI_R_COL_angle);
    BI_L_COL.setAngle(BI_L_COL_angle());
    S_LINEAR_S1.setAngle(S_LINEAR_S1_angle);
    S_LINEAR_S2.setAngle(S_LINEAR_S2_angle);
    BI_UP_DOMN_COL.setAngle(BI_UP_DOMN_COL_angle);
    BI_SMALL_LINEAR.setAngle(BI_SMALL_LINEAR_angle);
    BO_GRAB_LINEAR.setAngle(BO_GRAB_LINEAR_angle);
    BO_ROTATE.setAngle(BO_ROTATE_angle);
    Backup.setAngle(Backup_angle);
}

void magnet_valve_refresh(void){
    if(F_MAG_VAL){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, GPIO_PIN_RESET);
    }
    
    if(B_MAG_VAL){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
    }

    if(B_OUT_MAG_VAL){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);
    }

    mag_val_protect();
}

void air_pump_refresh(void){
    if(AIR_PUMP){
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);
    }
}

void back_in_grab(void){
    AIR_PUMP = 1;
    B_MAG_VAL = 0;
    if(BI_UP_DOMN_COL_angle !=1500){
        BI_UP_DOMN_COL_angle = 1500;
        osDelay(100);
    }
    if(BI_SMALL_LINEAR_angle != 1500){
        BI_SMALL_LINEAR_angle = 1500;
        osDelay(500);
    }
    BI_UP_DOMN_COL_angle = 600;
    osDelay(900);
}
void back_in_release(void){
    BI_UP_DOMN_COL_angle = 1500;
    osDelay(300);
    BI_SMALL_LINEAR_angle = 400;
    AIR_PUMP = 0;
    B_MAG_VAL = 1;
}
void back_out_open(void){
    BI_R_COL_angle = 900;
    osDelay(100);
}
void back_out_close(void){
    BI_R_COL_angle = 430;
    osDelay(100);
}
void back_out_release(void){
    if(BI_R_COL_angle != 430){
        BI_R_COL_angle = 430;
        osDelay(100);
    }
    if(BI_UP_DOMN_COL_angle != 1500){
        BI_UP_DOMN_COL_angle = 1500;
        osDelay(100);
    }
    if(BI_SMALL_LINEAR_angle != 400){
        BI_SMALL_LINEAR_angle = 400;
        osDelay(500);
    }
    B_OUT_MAG_VAL = 1;
}

void back_small_arm_open(void){
    BO_ROTATE_angle = 400;
    BI_SMALL_LINEAR_angle = 1500;
    osDelay(300);
    BO_GRAB_LINEAR_angle = 1200;
}
void back_small_arm_close(void){
    if(BO_GRAB_LINEAR_angle != 400){
        BO_GRAB_LINEAR_angle = 400;
        osDelay(400);
    }
    BO_ROTATE_angle = 950;
}
void back_small_arm_grab(void){
    if(BO_GRAB_LINEAR_angle != 1200){
        BO_ROTATE_angle = 400;
        osDelay(100);
        BO_GRAB_LINEAR_angle = 1200;
        osDelay(500);
    }
    BO_GRAB_LINEAR_angle = 500;
    osDelay(200);
}
void back_small_arm_release(void){
    BO_GRAB_LINEAR_angle = 1200;
    osDelay(300);
}


void front_grab(void){
    F_MAG_VAL = 0;
    AIR_PUMP = 1;
    F_Grab_L2_angle = 545;
    osDelay(400);
}
void front_release(void){ // release and close the magnet valve
    F_MAG_VAL = 1;
    F_Grab_L2_angle = 1050;
    AIR_PUMP = 0;
    osDelay(300);
}
void front_lift_to_top(void){
    F_Lift_Cascade_angle_prev = F_Lift_Cascade_angle;
    if(F_Lift_Cascade_angle != 530){
        while(abs(F_Lift_Cascade_angle-530) != 0){
            if(abs(F_Lift_Cascade_angle-530) < 20){
                F_Lift_Cascade_angle = 530;
            }else{
                F_Lift_Cascade_angle += (530-F_Lift_Cascade_angle_prev)/100;
            }
            osDelay(5);
        }
        osDelay(800);
    }
}
void front_lift_to_middle(void){
    F_Lift_Cascade_angle_prev = F_Lift_Cascade_angle;
    while( abs(F_Lift_Cascade_angle-890) != 0 ){
        if(abs(F_Lift_Cascade_angle-890) < 20){
            F_Lift_Cascade_angle = 890;
        }else{
            F_Lift_Cascade_angle += (890-F_Lift_Cascade_angle_prev)/40;
        }
        osDelay(4);
    }
}
void front_lift_to_middle_lower(void){
    F_Lift_Cascade_angle_prev = F_Lift_Cascade_angle;
    while( abs(F_Lift_Cascade_angle-1020) != 0 ){
        if(abs(F_Lift_Cascade_angle-1020) < 20){
            F_Lift_Cascade_angle = 1020;
        }else{
            F_Lift_Cascade_angle += (1020-F_Lift_Cascade_angle_prev)/10;
        }
        osDelay(4);
    }
}
void front_lift_to_bottom(void){
    F_Lift_Cascade_angle_prev = F_Lift_Cascade_angle;
    while( abs(F_Lift_Cascade_angle-1620) != 0 ){
        if(abs(F_Lift_Cascade_angle-1620) < 20){
            F_Lift_Cascade_angle = 1620;
        }else{
            F_Lift_Cascade_angle += (1620-F_Lift_Cascade_angle_prev)/100;
        }
        osDelay(6);
    }
}
void front_wood_grab(void){
    F_Pushing_angle = 1600;
    F_Lifting_UpDown_angle = 1170;
    F_Grab_Linear_angle = 1200;
    osDelay(500);
    F_Lifting_UpDown_angle = 900;
    osDelay(600);
    F_Grab_Linear_angle = 500;
    osDelay(700);
    F_Lifting_UpDown_angle = 1170;
    osDelay(500);
    F_Pushing_angle = 500;
    osDelay(300);
}
void front_wood_release(void){
    F_Pushing_angle = 1600;
    osDelay(500);
    F_Lifting_UpDown_angle = 900;
    osDelay(500);
    F_Grab_Linear_angle = 1000;
    osDelay(200);
    F_Lifting_UpDown_angle = 1170;
    osDelay(300);
    F_Pushing_angle = 500;
    F_Grab_Linear_angle = 500;
    osDelay(300);
}


void air_pump_on(void){
    AIR_PUMP = 1;
}

void air_pump_off(void){
    AIR_PUMP = 0;
}

void front_mag_valve_off(void){
    F_MAG_VAL = 0;
    osDelay(100);
}

void back_mag_valve_off(void){
    B_MAG_VAL = 0;
    osDelay(100);
}




int abs(int a){
    if(a<0){
        return -a;
    }else{
        return a;
    }
}

void mag_val_protect(void){
    if (F_MAG_VAL) {
        if (f_magnet_start_time == 0) {
            f_magnet_start_time = HAL_GetTick();
        } else if ((HAL_GetTick() - f_magnet_start_time) > F_MAGNET_TIMEOUT) {
            F_MAG_VAL = 0;
            f_magnet_start_time = 0;
        }
    } else {
        f_magnet_start_time = 0;
    }
    if (B_MAG_VAL) {
        if (b_magnet_start_time == 0) {
            b_magnet_start_time = HAL_GetTick();
        } else if ((HAL_GetTick() - b_magnet_start_time) > B_MAGNET_TIMEOUT) {
            B_MAG_VAL = 0;
            b_magnet_start_time = 0;
        }
    } else {
        b_magnet_start_time = 0;
    }
    if (B_OUT_MAG_VAL) {
        if (b_out_magnet_start_time == 0) {
            b_out_magnet_start_time = HAL_GetTick();
        } else if ((HAL_GetTick() - b_out_magnet_start_time) > B_OUT_MAGNET_TIMEOUT) {
            B_OUT_MAG_VAL = 0;
            b_out_magnet_start_time = 0;
        }
    } else {
        b_out_magnet_start_time = 0;
    }
}