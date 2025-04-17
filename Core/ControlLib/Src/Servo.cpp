/*
 * Servo.cpp
 *
 *  Created on: Feb 27, 2025
 *      Author: stanly
 */


#include "Servo.h"

void Servo::init()
{
    HAL_TIM_PWM_Start(_HTIMx, _channel);
}

void Servo::setAngle(uint16_t angle)
{
    if (angle < _min_angle)
    {
        angle = _min_angle;
    }
    else if (angle > _max_angle)
    {
        angle = _max_angle;
    }

    _goal_angle = angle;
    _angleToPulse(_goal_angle);
    _setPulse(_goal_pulse);
}

void Servo::_angleToPulse(uint16_t angle)
{
    _goal_pulse = (angle - _min_angle) * (_max_pulse - _min_pulse) / (_max_angle - _min_angle) + _min_pulse;
}

void Servo::_setPulse(uint16_t pulse)
{
    __HAL_TIM_SET_COMPARE(_HTIMx, _channel, pulse);
}



