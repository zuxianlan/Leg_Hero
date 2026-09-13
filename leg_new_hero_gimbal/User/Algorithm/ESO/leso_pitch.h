//
// Created by 25031 on 2026/7/24.
//

#ifndef CTRLBOARD_H7_IMU_LESO_PITCH_H
#define CTRLBOARD_H7_IMU_LESO_PITCH_H

// 初始化LESO（运行时状态清零）
void LESO_Init(float init_angle);

// 核心更新接口：输入当前拍原始控制量u，当前实测角度y，返回当前估计的总扰动值d_hat
float LESO_Update(float u, float y);

// 辅助接口：获取滤波后的角度和角速度（供控制器使用）
float LESO_GetAngle(void);
float LESO_GetRate(void);

#endif //CTRLBOARD_H7_IMU_LESO_PITCH_H