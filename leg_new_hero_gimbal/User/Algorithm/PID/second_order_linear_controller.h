//
// Created by 25031 on 2026/2/1.
//

#ifndef CTRLBOARD_H7_IMU_SECOND_ORDER_LINEAR_CONTROLLER_H
#define CTRLBOARD_H7_IMU_SECOND_ORDER_LINEAR_CONTROLLER_H

#include "main.h"
#include "user_lib.h"

//云台电机二阶线性控制器
typedef struct
{
    //设定值
    fp32 set_angle;
    //当前角度   一阶状态
    fp32 cur_angle;
    //当前角速度 二阶状态
    fp32 cur_angle_speed;
    //角度误差项 一阶状态误差
    fp32 angle_error;
    //前馈项，用于消除系统固有扰动
    fp32 feed_forward;
    //输出值
    fp32 output;
    //最大输出值
    fp32 max_out;
    //最小输出值
    fp32 min_out;

    //前馈项系数
    fp32 k_feed_forward;
    //误差项系数
    fp32 k_angle_error;
    //二阶角速度项系数
    fp32 k_angle_speed;

}gimbal_motor_second_order_linear_controller_t;

void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out);
fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current);

#endif //CTRLBOARD_H7_IMU_SECOND_ORDER_LINEAR_CONTROLLER_H