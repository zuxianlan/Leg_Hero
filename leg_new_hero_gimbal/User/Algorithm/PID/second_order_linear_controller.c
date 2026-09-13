//
// Created by 25031 on 2026/2/1.
//

#include "second_order_linear_controller.h"

/**
 * @brief 云台二阶线性控制器初始化
 *
 * @param controller 云台二阶线性控制器结构体
 * @param k_feed_forward 前馈系数
 * @param k_angle_error 角度误差系数
 * @param k_angle_speed 角速度系数
 * @param max_out 最大输出值
 * @param min_out 最小输出值
 */
void gimbal_motor_second_order_linear_controller_init(gimbal_motor_second_order_linear_controller_t *controller, fp32 k_feed_forward, fp32 k_angle_error, fp32 k_angle_speed, fp32 max_out, fp32 min_out)
{
    // 前馈项系数
    controller->k_feed_forward = k_feed_forward;
    // 反馈矩阵系数
    controller->k_angle_error = k_angle_error;
    controller->k_angle_speed = k_angle_speed;
    // 设置最大输出值
    controller->max_out = max_out;
    // 设置最小输出值
    controller->min_out = min_out;
}

/**
 * @brief 云台二阶线性控制器计算
 *
 * @param controller 云台二阶线性控制器结构体
 * @param set_angle 角度设置值
 * @param cur_angle 当前角度
 * @param cur_angle_speed 当前角速度
 * @param cur_current 当前电流
 * @return 返回系统输入
 */
fp32 gimbal_motor_second_order_linear_controller_calc(gimbal_motor_second_order_linear_controller_t *controller, fp32 set_angle, fp32 cur_angle, fp32 cur_angle_speed, fp32 cur_current)
{
    // 赋值
    controller->cur_angle = cur_angle;
    controller->set_angle = set_angle;
    controller->cur_angle_speed = cur_angle_speed;
    // 将当前电流值乘以一个小于1的系数当作阻挡系统固有扰动的前馈项
    controller->feed_forward = controller->k_feed_forward * cur_current;
    // 计算误差 = 设定角度 - 当前角度
    controller->angle_error = controller->set_angle - controller->cur_angle;
    // 将误差值限制 -PI ~ PI 之间
    controller->angle_error = rad_format(controller->angle_error);
    // 计算输出值 = 前馈值 + 角度误差值 * 系数 + 角速度 * 系数
    controller->output = controller->feed_forward + controller->angle_error * controller->k_angle_error + (-controller->cur_angle_speed * controller->k_angle_speed);

    //限制输出值，防止出现电机崩溃的情况
    if (controller->output >= controller->max_out)
    {
        controller->output = controller->max_out;
    }
    else if (controller->output <= controller->min_out)
    {
        controller->output = controller->min_out;
    }

    return controller->output;
}