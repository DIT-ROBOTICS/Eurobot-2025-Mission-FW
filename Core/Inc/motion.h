/*
 * motion.h
 *
 *  Created on: Mar 16, 2025
 *      Author: stanly
 */

#ifndef INC_MOTION_H_
#define INC_MOTION_H_

#ifdef __cplusplus
extern "C" {
#endif

// Add declarations here
void motion_init(void);
void motion_initial_pose(void);
void servo_refresh_angle(void);
void magnet_valve_refresh(void);
void air_pump_refresh(void);

void back_in_grab(void);
void back_in_release(void);
void back_out_open(void);
void back_out_close(void);
void back_out_release(void);
void back_small_arm_open(void);
void back_small_arm_close(void);
void back_small_arm_grab(void);
void back_small_arm_release(void);

void front_grab(void);
void front_grab_one_layer(void);
void front_release(void);
void front_lift_to_top(void);
void front_lift_to_middle(void);
void front_lift_to_middle_lower(void);
void front_lift_to_bottom_higher(void);
void front_lift_to_dick_release_pose(void);
void front_lift_to_bottom(void);
void front_wood_initial_pose(void);
void front_wood_grab(void);
void front_wood_release(void);

void front_banner_release(void);
void front_arm_reset(void);

// void side_stretch_out(void);
// void side_shrink_in(void);
// void side_outer_lift_down(void);
// void side_outer_lift_up(void);

void air_pump_on(void);
void air_pump_off(void);

void front_mag_valve_off(void);
void back_mag_valve_off(void);

void dick_initial_pose(void);
// void side_initial_pose(void);



int abs(int a);
void mag_val_protect(void);

#ifdef __cplusplus
}
#endif



#endif /* INC_MOTION_H_ */
