/**
  ******************************************************************************
  * @file           : task_and_callback.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/8
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <netinet/in.h>
#include "task_and_callback.h"
#include "CRC8_CRC16.h"
#include "Bsp/BspSerialPort.h"
#include "Interaction/robot.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/
float rc_buffer[5][19] = {0};
/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

// 冒泡排序：将数组元素从小到大排序
void bubble_sort(float arr[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = 0; j < n - i - 1; j++) {
            if (arr[j] > arr[j + 1]) {
                float temp = arr[j];
                arr[j] = arr[j + 1];
                arr[j + 1] = temp;
            }
        }
    }
}

/**
 * 中值滤波
 * @param input: 需要滤波的输入数据
 * @param buffer: 存储历史数据的滑动窗口缓冲区
 * @param size: 中值滤波窗口大小，取 3, 5 或 7
 */
float median_filter(float input, float* buffer, int size) {
    // 1. 将缓冲区数据依次向后移动一位
    for (int i = size - 1; i > 0; i--) {
        buffer[i] = buffer[i - 1];
    }
    buffer[0] = input;

    // 2. 将缓冲区数据拷贝到临时数组
    float temp_buf[size];
    for (int i = 0; i < size; i++) {
        temp_buf[i] = buffer[i];
    }

    // 3. 对临时数组排序并取中值
    bubble_sort(temp_buf, size);
    return temp_buf[size / 2];
}

Robot robot;
class_Timer timer_1ms;
class_Timer timer_2ms;
class_Timer timer_10ms;

using Clock = std::chrono::high_resolution_clock; // 高精度计时时钟
using Duration = std::chrono::duration<double>; // 以秒为单位的时间间隔

// MC02串口数据接收回调 - 处理串口数据
void mc02_callback(std::vector<uint8_t>& data)
{
    if (verify_CRC16_check_sum(data.data(), sizeof(mc_02_Rx_data) + 2))
    {
        const auto* tmp_buffer = reinterpret_cast<mc_02_Rx_data*>(data.data());
        if (tmp_buffer->header == 0x5A)
        {
            std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);
            // IMU
            robot.mc02_SerialPort.Rx_Data.gyro[0] = tmp_buffer->gyro[0];
            robot.mc02_SerialPort.Rx_Data.gyro[1] = tmp_buffer->gyro[1];
            robot.mc02_SerialPort.Rx_Data.gyro[2] = tmp_buffer->gyro[2];
            robot.mc02_SerialPort.Rx_Data.accel[0] = tmp_buffer->accel[0];
            robot.mc02_SerialPort.Rx_Data.accel[1] = tmp_buffer->accel[1];
            robot.mc02_SerialPort.Rx_Data.accel[2] = tmp_buffer->accel[2];

            // RC
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[1] = tmp_buffer->RC_ctrl.ch[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[2] = tmp_buffer->RC_ctrl.ch[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[3] = tmp_buffer->RC_ctrl.ch[3];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[4] = tmp_buffer->RC_ctrl.ch[4];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[1] = tmp_buffer->RC_ctrl.rotory_sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[2] = tmp_buffer->RC_ctrl.rotory_sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[1] = tmp_buffer->RC_ctrl.sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[2] = tmp_buffer->RC_ctrl.sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[3] = tmp_buffer->RC_ctrl.sw[3];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[4] = tmp_buffer->RC_ctrl.sw[4];

            // Motor
            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position = tmp_buffer->Motor_joint[0].Position;
            std::cout<< tmp_buffer->Motor_joint[0].Position << std::endl;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position = tmp_buffer->Motor_joint[1].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position = tmp_buffer->Motor_joint[2].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position = tmp_buffer->Motor_joint[3].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].Position = tmp_buffer->Motor_wheel[0].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].Position = tmp_buffer->Motor_wheel[1].Position;

            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].omega = tmp_buffer->Motor_joint[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].omega = tmp_buffer->Motor_joint[1].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].omega = tmp_buffer->Motor_joint[2].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].omega = tmp_buffer->Motor_joint[3].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].omega = tmp_buffer->Motor_wheel[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].omega = tmp_buffer->Motor_wheel[1].omega;

            robot.mc02_SerialPort.Rx_Data.roll = tmp_buffer->roll;
            robot.mc02_SerialPort.Rx_Data.pitch = tmp_buffer->pitch;
            robot.mc02_SerialPort.Rx_Data.yaw = tmp_buffer->yaw;
        }
    }
    else
    {
        std::cout<<"CRC Filed"<<std::endl;
    }
}

void mc02_callback_usb(const uint8_t* data, int length)
{
    if (length < static_cast<int>(sizeof(mc_02_Rx_data) + 2)) return;
    if (verify_CRC16_check_sum(data, sizeof(mc_02_Rx_data) + 2))
    {
        const auto* tmp_buffer = reinterpret_cast<const mc_02_Rx_data*>(data);
        if (tmp_buffer->header == 0x5A)
        {
            std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);
            // IMU
            robot.mc02_SerialPort.Rx_Data.gyro[0] = -tmp_buffer->gyro[0];
            robot.mc02_SerialPort.Rx_Data.gyro[1] = -tmp_buffer->gyro[1];
            robot.mc02_SerialPort.Rx_Data.gyro[2] = tmp_buffer->gyro[2];
            robot.mc02_SerialPort.Rx_Data.accel[0] = -tmp_buffer->accel[0];
            robot.mc02_SerialPort.Rx_Data.accel[1] = -tmp_buffer->accel[1];
            robot.mc02_SerialPort.Rx_Data.accel[2] = tmp_buffer->accel[2];

            // RC
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[1] = tmp_buffer->RC_ctrl.ch[1];
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[2] = tmp_buffer->RC_ctrl.ch[2];
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[3] = tmp_buffer->RC_ctrl.ch[3];
            // robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[4] = tmp_buffer->RC_ctrl.ch[4];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[1] = median_filter(tmp_buffer->RC_ctrl.ch[1], rc_buffer[0], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[2] = median_filter(tmp_buffer->RC_ctrl.ch[2], rc_buffer[1], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[3] = median_filter(tmp_buffer->RC_ctrl.ch[3], rc_buffer[2], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.ch[4] = median_filter(tmp_buffer->RC_ctrl.ch[4], rc_buffer[3], 19);
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[1] = tmp_buffer->RC_ctrl.rotory_sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.rotory_sw[2] = tmp_buffer->RC_ctrl.rotory_sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[1] = tmp_buffer->RC_ctrl.sw[1];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[2] = tmp_buffer->RC_ctrl.sw[2];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[3] = tmp_buffer->RC_ctrl.sw[3];
            robot.mc02_SerialPort.Rx_Data.RC_ctrl.sw[4] = tmp_buffer->RC_ctrl.sw[4];

            // Motor
            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position = tmp_buffer->Motor_joint[0].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position = tmp_buffer->Motor_joint[1].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position = tmp_buffer->Motor_joint[2].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position = tmp_buffer->Motor_joint[3].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].Position = tmp_buffer->Motor_wheel[0].Position;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].Position = tmp_buffer->Motor_wheel[1].Position;

            robot.mc02_SerialPort.Rx_Data.Motor_joint[0].omega = tmp_buffer->Motor_joint[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[1].omega = tmp_buffer->Motor_joint[1].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[2].omega = tmp_buffer->Motor_joint[2].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_joint[3].omega = tmp_buffer->Motor_joint[3].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[0].omega = tmp_buffer->Motor_wheel[0].omega;
            robot.mc02_SerialPort.Rx_Data.Motor_wheel[1].omega = tmp_buffer->Motor_wheel[1].omega;

            robot.mc02_SerialPort.Rx_Data.roll = tmp_buffer->roll;
            robot.mc02_SerialPort.Rx_Data.pitch = tmp_buffer->pitch;
            robot.mc02_SerialPort.Rx_Data.yaw = tmp_buffer->yaw;

            // robot.mc02_SerialPort.Rx_Data.current[0] = tmp_buffer->current[0];
            // robot.mc02_SerialPort.Rx_Data.current[1] = tmp_buffer->current[1];

            robot.mc02_SerialPort.Rx_Data.motor_yaw_angle = tmp_buffer->motor_yaw_angle;
            robot.mc02_SerialPort.Rx_Data.key = median_filter(tmp_buffer->key, rc_buffer[4], 19);
            
        }
    }
    else
    {
        std::cout<<"CRC Filed"<<std::endl;
    }
}

void mc02_send_data()
{
    auto* tmp_buffer = reinterpret_cast<mc_02_Tx_data*>(robot.mc02_SerialPort.Tx_buffer);

    tmp_buffer->header = 0xA5;

    tmp_buffer->joint_torque_set[0] = robot.Chassis.left_leg.Get_torque1()*1.0;
    tmp_buffer->joint_torque_set[1] = robot.Chassis.left_leg.Get_torque2()*1.0;
    tmp_buffer->joint_torque_set[2] = -robot.Chassis.right_leg.Get_torque1()*1.0;
    tmp_buffer->joint_torque_set[3] = -robot.Chassis.right_leg.Get_torque2()*1.0;
    tmp_buffer->wheel_torque_set[0] = robot.Chassis.Get_T_L()*1.0;
    tmp_buffer->wheel_torque_set[1] = -robot.Chassis.Get_T_R()*1.0;
    tmp_buffer->spin_flag = static_cast<uint8_t> (robot.RC.spin);
    // tmp_buffer->wheel_torque_set[0] = 0.5;
    // tmp_buffer->wheel_torque_set[1] = 0.5;


    append_CRC16_check_sum(robot.mc02_SerialPort.Tx_buffer, sizeof(mc_02_Tx_data) + 2);

    if (robot.mc02_SerialPort.usb.is_connected())
    {
        robot.mc02_SerialPort.usb.send(robot.mc02_SerialPort.Tx_buffer, sizeof(mc_02_Tx_data) + 2);
    }

    // int bytes = robot.mc02_SerialPort.serial_port_.send_data(robot.mc02_SerialPort.Tx_buffer, sizeof(mc_02_Tx_data));

    // if (bytes < 0)
    // {
    //     std::cout << "Serial port: Field to send data" << std::endl;
    // }
}

void TIM_1ms_PeriodElapsedCallback()
{
    //  互斥锁保护，Rx_Data 数据由 USB 回调函数更新
    std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);

    robot.INS.INS_Update();
    robot.INS.pitch = -robot.mc02_SerialPort.Rx_Data.pitch;
    robot.INS.yaw = robot.mc02_SerialPort.Rx_Data.yaw;
    robot.INS.roll = -robot.mc02_SerialPort.Rx_Data.roll;
    // robot.Fusion_AHRS_update();

    robot.Chassis.Self_Observe();
}

void TIM_2ms_PeriodElapsedCallback()
{
    auto start = std::chrono::high_resolution_clock::now(); 
    {
        //  互斥锁保护，Rx_Data 数据由 USB 回调函数更新
        std::lock_guard<std::mutex> lock(robot.mc02_SerialPort.rx_mutex);

        robot.FSM_Chassis.TIM_1ms_Calculate_PeriodElapsedCallback();
        robot.Jump_Control();
        robot.UP_staris_Control();

        robot.Chassis.feedback_update();
        robot.Chassis_Control();
        if(robot.Get_jump_flag() == 0)
        {
            robot.Chassis.Ground_Detection();
        }
        Vofa_start();
    }
    //  mc02_send_data() 只写 Tx_buffer + USB send，不读取 Rx_Data，故无需加锁
    //  因为 libusb_bulk_transfer 最多阻塞 1s 超时，会阻塞 USB 发送
    mc02_send_data();

    auto stop = std::chrono::high_resolution_clock::now();  
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count();  
  
    std::cout << "Elapsed time: " << duration << " us\n"; 

}

void TIM_10ms_PeriodElapsedCallback()
{
    // robot.Chassis.NMPC_Solve();
    // if(robot.chassis_mode == CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW && robot.FSM_Chassis.Get_Now_Status_Serial() == NORMAL)
    // {
    //     robot.Chassis.MPC_Body_Calc();
    // }
}



void Task_Init()
{
    robot.Init();
    // robot.mc02_SerialPort.serial_port_.SetCallbackFunction(mc02_callback);
    robot.mc02_SerialPort.usb.on_data(mc02_callback_usb);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // timer initialize
    timer_1ms.start(0.001, TIM_1ms_PeriodElapsedCallback);

    timer_2ms.start(0.001, TIM_2ms_PeriodElapsedCallback);

    timer_10ms.start(0.01, TIM_10ms_PeriodElapsedCallback);


}

void Vofa_start()
{
    robot.Vofa_TCP.send_message.v0 = robot.INS.yaw;
    robot.Vofa_TCP.send_message.v1 = robot.INS.pitch;
    robot.Vofa_TCP.send_message.v2 = robot.INS.roll;
    robot.Vofa_TCP.send_message.v3 = robot.mc02_SerialPort.Rx_Data.motor_yaw_angle;
    robot.Vofa_TCP.send_message.v4 = robot.Chassis.right_leg.Get_theta();
    robot.Vofa_TCP.send_message.v5 = robot.Chassis.left_leg.Get_theta();
    robot.Vofa_TCP.send_message.v6 = robot.FSM_Chassis.Get_Now_Status_Serial();
    robot.Vofa_TCP.send_message.v7 = robot.RC.axis[2];
    robot.Vofa_TCP.send_message.v8 = robot.Get_Target_height();
    robot.Vofa_TCP.send_message.v9 = robot.Chassis.left_leg.Get_L0();
    robot.Vofa_TCP.send_message.v10 = robot.Chassis.right_leg.Get_L0();
    robot.Vofa_TCP.send_message.v11 = robot.Chassis.left_leg.Get_torque1();
    robot.Vofa_TCP.send_message.v12 = robot.Chassis.Get_Heigh();
    robot.Vofa_TCP.send_message.v13 = robot.Chassis.left_leg.Get_Fn();
    robot.Vofa_TCP.send_message.v14 = robot.Chassis.right_leg.Get_Fn();
    robot.Vofa_TCP.send_message.v15 = robot.Chassis.Get_above_flag_r();
    robot.Vofa_TCP.send_message.v16 = robot.mc02_SerialPort.Rx_Data.Motor_joint[0].Position;
    robot.Vofa_TCP.send_message.v17 = robot.mc02_SerialPort.Rx_Data.Motor_joint[1].Position;
    robot.Vofa_TCP.send_message.v18 = robot.mc02_SerialPort.Rx_Data.Motor_joint[2].Position;
    robot.Vofa_TCP.send_message.v19 = robot.mc02_SerialPort.Rx_Data.Motor_joint[3].Position;

    robot.Vofa_TCP.Start();
}

float get_mahony_pitch()
{
    return robot.mc02_SerialPort.Rx_Data.pitch;
}

float get_mahony_yaw()
{
    return robot.mc02_SerialPort.Rx_Data.yaw;
}