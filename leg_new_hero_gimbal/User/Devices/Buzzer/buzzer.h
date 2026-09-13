#ifndef RADAR_GIMBAL_BUZZER_H
#define RADAR_GIMBAL_BUZZER_H

#include "stdint.h"

/****蜂鸣器****/
#define BUZZER_HTIM htim12

typedef enum {
    BUZZER_IDLE = 0,
    BUZZER_PLAY
} buzzer_status_enum;

typedef enum {
    SINGLE = 0,
    CIRCLES = 1,
} buzzer_mode_enum;

// 音效序列结构体
typedef struct {
    uint16_t freq;      // 频率 (Hz)，0 表示静音
    uint16_t duration;  // 持续时间 (ms)
} sound_struct;

typedef struct {
    uint32_t currentTime;       // 当前音符已播放时间（ms）
    uint8_t  index;             // 当前音符索引
    buzzer_status_enum status;  // 状态机状态
    buzzer_mode_enum  mode;     // 播放模式
    uint8_t  priority;          // 当前任务优先级（数值越小优先级越高）

    // 当前播放任务
    sound_struct *seq;          // 指向音效序列数组
    uint8_t       seq_len;      // 序列长度
} buzzer_struct;

extern buzzer_struct buzzer;

// 音效序列声明
extern sound_struct startSound_1_9len[];
extern sound_struct startSound_2_8len[];
extern sound_struct underVoltSound_4len[];
extern sound_struct overVoltSound_6len[];

// API
void Buzzer_Init(void);
void Buzzer_Play(sound_struct *seq, uint8_t len, buzzer_mode_enum mode, uint8_t priority);
void Buzzer_Process(void);      // 需在定时中断中周期调用（如 1ms）

#endif