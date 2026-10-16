#include "buzzer.h"
#include "tim.h"

buzzer_struct buzzer;

/* 音效序列定义 */
sound_struct startSound_1_9len[] = {
    {1300, 40},
    {117, 15},
    {300, 5},
    {200, 15},
    {805, 15},
    {1175, 10},
    {0, 15},
    {1176, 10},
    {0, 10}
};

sound_struct startSound_2_8len[] = {
    {466, 10},
    {587, 15},
    {740, 15},
    {880, 15},
    {1175, 20},
    {0, 10},
    {1175, 24},
    {0, 10}
};

sound_struct underVoltSound_4len[] = {
    {880, 40},
    {0, 20},
    {880, 40},
    {0, 30}
};

sound_struct overVoltSound_6len[] = {
    {880, 15},
    {0, 5},
    {880, 15},
    {0, 5},
    {880, 15},
    {0, 50}
};


/* 内部函数：设置 PWM 频率 */
static void Buzzer_FreqSet(uint16_t freq) {
    if (freq == 0) {
        __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, 0);
        return;
    }
    uint32_t timer_clock = 240000000;
    uint32_t psc = BUZZER_HTIM.Init.Prescaler + 1;
    uint32_t period = timer_clock / freq / psc - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim12, period);
    __HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, period / 2);
}

/* 初始化 */
void Buzzer_Init(void) {
    buzzer.status = BUZZER_IDLE;
    buzzer.seq = NULL;
    buzzer.seq_len = 0;
    buzzer.mode = SINGLE;
    buzzer.priority = 255;      // 空闲时设为最大数值（最低优先级）
    buzzer.index = 0;
    buzzer.currentTime = 0;
    Buzzer_FreqSet(0);
}

/* 启动播放（优先级数值越小越高） */
void Buzzer_Play(sound_struct *seq, uint8_t len, buzzer_mode_enum mode, uint8_t priority) {
    if (seq == NULL || len == 0) return;

    // 如果当前正在播放同一个序列，则忽略（防止从头开始）
    if (buzzer.seq == seq && buzzer.status == BUZZER_PLAY) {
        return;
    }

    // 如果已有任务正在播放，且新任务优先级数值更大（即优先级更低），则忽略本次请求
    if (buzzer.seq != NULL && priority > buzzer.priority) {
        return;
    }

    // 覆盖或启动新任务
    buzzer.seq = seq;
    buzzer.seq_len = len;
    buzzer.mode = mode;
    buzzer.priority = priority;
    buzzer.status = BUZZER_IDLE;   // 下一轮 Process 会自动开始
    buzzer.index = 0;
    buzzer.currentTime = 0;
}

/* 状态机更新（每 1ms 调用一次） */
void Buzzer_Process(void) {
    // 没有任务则静音并返回
    if (buzzer.seq == NULL) {
        Buzzer_FreqSet(0);
        return;
    }

    switch (buzzer.status) {
        case BUZZER_IDLE:
            buzzer.index = 0;
            buzzer.currentTime = 0;
            buzzer.status = BUZZER_PLAY;
            Buzzer_FreqSet(buzzer.seq[0].freq);
            break;

        case BUZZER_PLAY:
            buzzer.currentTime++;
            if (buzzer.currentTime >= buzzer.seq[buzzer.index].duration) {
                buzzer.index++;
                buzzer.currentTime = 0;

                // 序列播放完毕
                if (buzzer.index >= buzzer.seq_len) {
                    if (buzzer.mode == CIRCLES) {
                        buzzer.status = BUZZER_IDLE;   // 循环：重新开始
                    } else {
                        // 单次播放完成：清空任务，回到空闲
                        buzzer.seq = NULL;
                        buzzer.status = BUZZER_IDLE;
                        buzzer.priority = 255;          // 恢复默认最低优先级
                        Buzzer_FreqSet(0);
                    }
                } else {
                    // 切换到下一音符
                    Buzzer_FreqSet(buzzer.seq[buzzer.index].freq);
                }
            }
            break;
    }
}