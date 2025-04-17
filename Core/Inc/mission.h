/*
 * mission.h
 *
 *  Created on: Mar 2, 2025
 *      Author: stanly
 */

#ifndef INC_MISSION_H_
#define INC_MISSION_H_

#ifdef __cplusplus
extern "C" {
#endif

// Add declarations here
void mission_init(void);

void mission_initial_pose(void);

void front_grab_11(void* pvParameters);
void front_release_10(void* pvParameters);
void back_release_outer_24(void* pvParameters);
void back_small_arm_close_23(void* pvParameters);
void back_small_arm_open_22(void* pvParameters);
void back_grab_21(void* pvParameters);
void back_release_20(void* pvParameters);
void front_mid_lower_33(void* pvParameters);
void front_top_32(void* pvParameters);
void front_one_layer_31(void* pvParameters);
void front_bottom_30(void* pvParameters);
void front_wood_takein_71(void* pvParameters);
void front_wood_takeout_70(void* pvParameters);
void front_mag_valve_disable_90(void* pvParameters);
void back_mag_valve_disable_91(void* pvParameters);

#ifdef __cplusplus
}
#endif


#endif /* INC_MISSION_H_ */
