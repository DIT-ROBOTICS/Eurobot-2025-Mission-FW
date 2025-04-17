/*
 * Servo.h
 *
 *  Created on: Feb 27, 2025
 *      Author: stanly
 */

#ifndef CONTROLLIB_INC_SERVO_H_
#define CONTROLLIB_INC_SERVO_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

class Servo
{
private:
    TIM_HandleTypeDef* _HTIMx = nullptr;
    uint32_t _channel = 0;
    uint16_t _goal_angle = 0;
    uint16_t _goal_pulse = 0;

    const uint16_t _min_pulse = 500;
    const uint16_t _max_pulse = 2500;
    const uint16_t _min_angle = 0;
    const uint16_t _max_angle = 1800;

    void _angleToPulse(uint16_t angle);
    void _setPulse(uint16_t pulse);

public:
    Servo(TIM_HandleTypeDef *HTIMx, uint32_t channel): _HTIMx(HTIMx), _channel(channel) {}
    void init();
    void setAngle(uint16_t angle);

};



#endif /* CONTROLLIB_INC_SERVO_H_ */
