/*
 * uros_init.cpp
 *
 *  Created on: Apr 9, 2025
 *      Author: stanly
 */


#include "uros_init.h"

rcl_publisher_t         mission_status_pub;
std_msgs__msg__Int32    mission_status_msg;
rcl_publisher_t         start_pub;
std_msgs__msg__Bool     start_msg;
rcl_subscription_t      mission_type_sub;
std_msgs__msg__Int32    mission_type_msg;
rcl_timer_t             status_pub_timer;
rcl_timer_t             start_pub_timer;


rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_init_options_t init_options;
rclc_executor_t executor;

agent_status_t status = AGENT_WAITING;

int32_t mission_type = 0;
int32_t mission_type_prev = 0;
int32_t mission_status = 1;
bool start_flag = 0;

int ping_fail_count = 0;
#define MAX_PING_FAIL_COUNT 5

// task created flag
int task_created = 0;
int task_created_1 = 0;
int task_created_2 = 0;
int task_created_3 = 0;
int task_created_7 = 0;
int task_created_8 = 0;
int task_created_9 = 0;

// float heap_usage[100] = {0}; // Array to store heap usage percentages
// int memory_usage_index = 0;
int time_delay[100] = {0};
int time_delay_index = 0;
uint32_t current_time = 0; // Variable to store the current time
uint64_t heap_remain = 0.0; // Variable to store heap usage percentage

extern UART_HandleTypeDef USARTx;

void uros_init(void) {
  // Initialize micro-ROS
  rmw_uros_set_custom_transport(
    true,
    (void *) &USARTx,
    cubemx_transport_open,
    cubemx_transport_close,
    cubemx_transport_write,
    cubemx_transport_read);
  
  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();

  freeRTOS_allocator.allocate = microros_allocate;
  freeRTOS_allocator.deallocate = microros_deallocate;
  freeRTOS_allocator.reallocate = microros_reallocate;
  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
  printf("Error on default allocators (line %d)\n", __LINE__); 
  }
}

void uros_agent_status_check(void) {
  switch (status) {
    case AGENT_WAITING:
      handle_state_agent_waiting();
      break;
    case AGENT_AVAILABLE:
      handle_state_agent_available();
      break;
    case AGENT_CONNECTED:
      handle_state_agent_connected();
      break;
    case AGENT_TRYING:
      handle_state_agent_trying();
      break;
    case AGENT_DISCONNECTED:
      handle_state_agent_disconnected();
      break;
    default:
      break;
  }
}

void handle_state_agent_waiting(void) {
  status = (rmw_uros_ping_agent(100, 10) == RMW_RET_OK) ? AGENT_AVAILABLE : AGENT_WAITING;
}
void handle_state_agent_available(void) {
  uros_create_entities();
  status = AGENT_CONNECTED;
}
void handle_state_agent_connected(void) {
  if(rmw_uros_ping_agent(20, 5) == RMW_RET_OK){
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_TRYING;
    }
  }
  current_time = HAL_GetTick();
  static uint32_t prev_time = 0;
  if (current_time - prev_time >= 100) {  // Every 100ms
    rmw_uros_sync_session(100);
    prev_time = current_time;
  }
}
void handle_state_agent_trying(void) {
  if(rmw_uros_ping_agent(50, 10) == RMW_RET_OK){
    status = AGENT_CONNECTED;
    ping_fail_count = 0; // Reset ping fail count
  } else {
    ping_fail_count++;
    if(ping_fail_count >= MAX_PING_FAIL_COUNT){
      status = AGENT_DISCONNECTED;
      ping_fail_count = 0;
    }
  }
}
void handle_state_agent_disconnected(void) {
  uros_destroy_entities();
  status = AGENT_WAITING;
}


void uros_create_entities(void) {
  allocator = rcl_get_default_allocator();

  init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, DOMAIN_ID);

  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator); // Initialize support structure

  rcl_init_options_fini(&init_options);
  
  rclc_node_init_default(&node, NODE_NAME, "", &support); // Initialize node
    
  rclc_publisher_init_default( // Initialize publisher for mission status
    &mission_status_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "mission_status");
  mission_status_msg.data = 0;
  rmw_uros_set_publisher_session_timeout( // Set session timeout for publisher
    rcl_publisher_get_rmw_handle(&mission_status_pub),
    10);

  rclc_publisher_init_default( // Initialize publisher for start message
    &start_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/robot/startup/plug");
  start_msg.data = 0;
  rmw_uros_set_publisher_session_timeout( // Set session timeout for start publisher
    rcl_publisher_get_rmw_handle(&start_pub),
    10);

  rclc_subscription_init_default( // Initialize subscriber for mission type
    &mission_type_sub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "mission_type");
  mission_type_msg.data = 0;

  rclc_timer_init_default(&status_pub_timer, &support, RCL_MS_TO_NS(1000/FREQUENCY), status_pub_cb); // Initialize status message timer
  rclc_timer_init_default(&start_pub_timer, &support, RCL_MS_TO_NS(1000/FREQUENCY), start_pub_cb); // Initialize start message timer
  
  rclc_executor_init(&executor, &support.context, 3, &allocator); // Create executor

  rclc_executor_add_subscription(&executor, &mission_type_sub, &mission_type_msg, &mission_type_sub_cb, ON_NEW_DATA); // Add subscriber to executor
  rclc_executor_add_timer(&executor, &status_pub_timer); // Add timer to executor
  rclc_executor_add_timer(&executor, &start_pub_timer); // Add start message timer to executor
}
void uros_destroy_entities(void) {
  rmw_context_t* rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  // Destroy timer
  rcl_timer_fini(&status_pub_timer);
  rcl_timer_fini(&start_pub_timer);

  // Destroy publisher
  rcl_publisher_fini(&mission_status_pub, &node);
  rcl_publisher_fini(&start_pub, &node);

  // Destroy subscriber
  rcl_subscription_fini(&mission_type_sub, &node);

  // Destroy executor
  rclc_executor_fini(&executor);

  // Destroy node
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void mission_type_sub_cb(const void * msgin) {
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *) msgin;
  mission_type = msg->data;

  do_mission(); // Call the function to handle the mission based on the received type
}
void status_pub_cb(rcl_timer_t * timer, int64_t last_call_time) {
    mission_status_msg.data = mission_status;
    rcl_publish(&mission_status_pub, &mission_status_msg, NULL);
}
void start_pub_cb(rcl_timer_t * timer, int64_t last_call_time) {
    start_msg.data = start_flag;
    rcl_publish(&start_pub, &start_msg, NULL);
}

void do_mission(void) {
  if (mission_type != mission_type_prev) {
    mission_type_prev = mission_type;
    mission_control();
  }
}
void mission_control(void) {
  switch(mission_type) {
    case 1:
      if(!task_created){
        xTaskCreate(robot_start_1, "Robot_Start_1", 256, NULL, osPriorityNormal, NULL);
        task_created = 1;
      }
      break;
    case 0:
      if(!task_created){
        start_flag = false;
        xTaskCreate(robot_stop_0, "Robot_Stop_0", 256, NULL, osPriorityNormal, NULL);
        task_created = 1;
      }
      break;
    case 15:
      if(!task_created_1){
        xTaskCreate(front_side_support_close_15, "Front_Side_Support_Close_15", 256, NULL, osPriorityNormal, NULL);
        task_created_1 = 1;
      }
      break;
    case 14:
      if(!task_created_1){
        xTaskCreate(front_wood_push_close_14, "Front_Wood_Push_Close_14", 256, NULL, osPriorityNormal, NULL);
        task_created_1 = 1;
      }
      break;
    case 13:
      if(!task_created_1){
        xTaskCreate(front_wood_push_13, "Front_Wood_Push_13", 256, NULL, osPriorityNormal, NULL);
        task_created_1 = 1;
      }
      break;
    case 12:
      if(!task_created_1){
        xTaskCreate(front_side_support_12, "Front_Side_Support_12", 256, NULL, osPriorityNormal, NULL);
        task_created_1 = 1;
      }
      break;
    case 11:
      if(!task_created_1){
        xTaskCreate(front_grab_11, "Front_Grab_11", 256, NULL, osPriorityNormal, NULL);
        task_created_1 = 1;
      }
      break;
    case 10:
      if(!task_created_1){
        xTaskCreate(front_release_10, "Front_Release_10", 256, NULL, osPriorityNormal, NULL);
        task_created_1 = 1;
      }
      break;
    case 25:
      if(!task_created_2){
        xTaskCreate(back_out_close_25, "Back_Out_Close_25", 256, NULL, osPriorityNormal, NULL);
        task_created_2 = 1;
      }
      break;
    case 24:
      if(!task_created_2){
        xTaskCreate(back_release_outer_24, "Back_Release_Outer_24", 256, NULL, osPriorityNormal, NULL);
        task_created_2 = 1;
      }
      break;
    case 23:
      if(!task_created_2){
        xTaskCreate(back_small_arm_close_23, "Back_Small_Arm_Close_23", 256, NULL, osPriorityNormal, NULL);
        task_created_2 = 1;
      }
      break;
    case 22:
      if(!task_created_2){
        xTaskCreate(back_small_arm_open_22, "Back_Small_Arm_Open_22", 256, NULL, osPriorityNormal, NULL);
        task_created_2 = 1;
      }
      break;
    case 21:
      if(!task_created_2){
        xTaskCreate(back_grab_21, "Back_Grab_21", 256, NULL, osPriorityNormal, NULL);
        task_created_2 = 1;
      }
      break;
    case 20:
      if(!task_created_2){
        xTaskCreate(back_release_20, "Back_Release_20", 256, NULL, osPriorityNormal, NULL);
        task_created_2 = 1;
      }
      break;
    case 35:
      if(!task_created_3){
        xTaskCreate(front_top_short_35, "Front_Top_Short_35", 256, NULL, osPriorityNormal, NULL);
        task_created_3 = 1;
      }
      break;
    case 34:
      if(!task_created_3){
        xTaskCreate(front_bot_higher_34, "Front_Bot_Higher_34", 256, NULL, osPriorityNormal, NULL);
        task_created_3 = 1;
      }
      break;
    case 33:
      if(!task_created_3){
        xTaskCreate(front_mid_lower_33, "Front_Mid_Lower_33", 256, NULL, osPriorityNormal, NULL);
        task_created_3 = 1;
      }
      break;
    case 32:
      if(!task_created_3){
        xTaskCreate(front_top_32, "Front_Top_32", 256, NULL, osPriorityNormal, NULL);
        task_created_3 = 1;
      }
      break;
    case 31: 
      if(!task_created_3){
        xTaskCreate(front_one_layer_31, "Front_One_Layer_31", 256, NULL, osPriorityNormal, NULL);
        task_created_3 = 1;
      }
      break;
    case 30:
      if(!task_created_3){
        xTaskCreate(front_bottom_30, "Front_Bottom_30", 256, NULL, osPriorityNormal, NULL);
        task_created_3 = 1;
      }
      break;
    case 71:
      if(!task_created_7){
        xTaskCreate(front_wood_takein_71, "Front_Wood_Takein_71", 256, NULL, osPriorityNormal, NULL);
        task_created_7 = 1;
      }
      break;
    case 70:
      if(!task_created_7){
        xTaskCreate(front_wood_takeout_70, "Front_Wood_Takeout_70", 256, NULL, osPriorityNormal, NULL);
        task_created_7 = 1;
      }
      break;
    case 80:
      if(!task_created_8){
        xTaskCreate(front_arm_reset_80, "Front_Arm_Reset_80", 256, NULL, osPriorityNormal, NULL);
        task_created_8 = 1;
      }
      break;
    case 81:
      if(!task_created_8){
        xTaskCreate(front_put_banner_81, "Front_Put_Banner_81", 256, NULL, osPriorityNormal, NULL);
        task_created_8 = 1;
      }
      break;
    case 90:
      if(!task_created_9){
        xTaskCreate(front_mag_valve_disable_90, "Front_Mag_Valve_Disable_90", 256, NULL, osPriorityNormal, NULL);
        task_created_9 = 1;
      }
      break;
    case 91:
      if(!task_created_9){
        xTaskCreate(back_mag_valve_disable_91, "Back_Mag_Valve_Disable_91", 256, NULL, osPriorityNormal, NULL);
        task_created_9 = 1;
      }
      break;
    default:
      break;
  }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if (GPIO_Pin == GPIO_PIN_10){ // Check if the interrupt is from PC10
    start_flag = true; // Set start_msg to true
  }
}
