//
// Created by 25031 on 2026/7/24.
//

#include "leso_pitch.h"
#include <string.h>   // 如果需要memcpy，但这里用不到

// ========== 在这里粘贴 MATLAB 生成的四个数组 ==========

// J = 0.020000, Ts = 0.001000
const float Ad[3][3] = {
    {1.00000000, 0.00100000, 0.00002500},
    {0.00000000, 1.00000000, 0.05000000},
    {0.00000000, 0.00000000, 1.00000000}
};

const float Bd[3] = {0.00002500, 0.05000000, 0.00000000};

const float Cd[3] = {1.0, 0.0, 0.0};

const float Le[3] = {0.78000000, 192.87500000, 345.00000000};

// ========== 粘贴结束 ==========

// 运行时状态（静态变量，内部维护）
static float x_hat[3] = {0.0f, 0.0f, 0.0f};   // [角度估计, 角速度估计, 扰动估计]

// 初始化：把状态清零（有绝对零点偏置可在这里加）
void LESO_Init(float init_angle)
{
    x_hat[0] = init_angle;
    x_hat[1] = 0.0f;
    x_hat[2] = 0.0f;
}

// ========== 唯一核心接口 ==========
// 输入：u  -> 当前拍原始控制量（未经扰动补偿的），单位 N?m
// 输入：y  -> 当前拍实测角度（来自IMU或编码器），单位 rad
// 输出：返回当前估计的总扰动 d_hat，单位 N?m
float LESO_Update(float u, float y)
{
    float err;          // 观测误差 = 实测角度 - 估计角度
    float x_new[3];     // 新状态暂存
    int i, j;

    // 1. 计算误差
    err = y - x_hat[0];

    // 2. 一步更新: x_new = Ad * x_hat + Bd * u + Le * err
    for (i = 0; i < 3; i++) {
        x_new[i] = 0.0f;
        for (j = 0; j < 3; j++) {
            x_new[i] += Ad[i][j] * x_hat[j];
        }
        x_new[i] += Bd[i] * u;
        x_new[i] += Le[i] * err;
    }

    // 3. 状态更新
    x_hat[0] = x_new[0];
    x_hat[1] = x_new[1];
    x_hat[2] = x_new[2];

    // 4. 返回当前估计的总扰动
    return x_hat[2];
}

// 辅助：获取滤波后的角度（可直接用于位置环）
float LESO_GetAngle(void)
{
    return x_hat[0];
}

// 辅助：获取滤波后的角速度（可直接用于速度环/阻尼）
float LESO_GetRate(void)
{
    return x_hat[1];
}