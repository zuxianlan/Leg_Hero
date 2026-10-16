//
// Created by 25031 on 2026/3/30.
//

#ifndef CTRLBOARD_H7_IMU_TASK_CONFIG_AND_CALLBACK_H
#define CTRLBOARD_H7_IMU_TASK_CONFIG_AND_CALLBACK_H

#include "main.h"

extern bool_t bsp_init_finished_flag;
extern uint16_t ceshi_tim12_half_ms_counter;
extern uint16_t ceshi_tim15_1_ms_counter;

void Task_Init();

#endif //CTRLBOARD_H7_IMU_TASK_CONFIG_AND_CALLBACK_H