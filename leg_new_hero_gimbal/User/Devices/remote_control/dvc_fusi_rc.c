#include "dvc_fusi_rc.h"
#include "string.h"

// 前置声明
void Fusi_RC_Data_Process(Class_Fusi_RC *self, uint8_t *Rx_Data);
void Fusi_RC_Raw_To_Process(Class_Fusi_RC *self);
void Class_Fusi_Three_Stage_Judge_Switch(Enum_Fusi_Three_Stage_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
void Class_Fusi_Two_Stage_Judge_Switch(Enum_Fusi_Two_Stage_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);

// 全局实例
Class_Fusi_RC class_fusi_rc;

uint16_t fusi_up_to_down = 0;
uint16_t fusi_down_to_up = 0;
uint16_t fusi_down_to_mid = 0;
uint16_t fusi_mid_to_down = 0;
uint16_t fusi_up_to_mid = 0;
uint16_t fusi_mid_to_up = 0;

/**
 * @brief 初始化 Fusi_RC 对象
 * @param rc   Class_Fusi_RC 指针
 * @param huart UART 句柄，用于绑定对应的 UART 管理对象
 */
void Fusi_RC_Init(Class_Fusi_RC *rc, UART_HandleTypeDef *huart)
{
    // 根据串口实例选择对应的 UART 管理对象（与 DR16 相同逻辑）
    if (huart->Instance == USART1)
        rc->UART_Manage_Object = &UART1_Manage_Object;
    else if (huart->Instance == USART2)
        rc->UART_Manage_Object = &UART2_Manage_Object;
    else if (huart->Instance == USART3)
        rc->UART_Manage_Object = &UART3_Manage_Object;
    else if (huart->Instance == UART4)
        rc->UART_Manage_Object = &UART4_Manage_Object;
    else if (huart->Instance == UART5)
        rc->UART_Manage_Object = &UART5_Manage_Object;
    else if (huart->Instance == USART6)
        rc->UART_Manage_Object = &UART6_Manage_Object;
    else if (huart->Instance == UART7)
        rc->UART_Manage_Object = &UART7_Manage_Object;
    else if (huart->Instance == UART8)
        rc->UART_Manage_Object = &UART8_Manage_Object;

    // 清空状态与数据
    rc->Flag = 0;
    rc->Last_Flag = 0;
    rc->Fusi_RC_Status = Fusi_RC_Status_DISABLE;

    memset(&rc->Raw_Data, 0, sizeof(rc->Raw_Data));
    memset(&rc->Processed_Data, 0, sizeof(rc->Processed_Data));
    memset(&rc->Last_Processed_Data, 0, sizeof(rc->Last_Processed_Data));
}

/**
 * @brief UART 接收完成回调（在 DMA 或中断接收完一帧后调用）
 * @param self     Class_Fusi_RC 指针
 * @param Rx_Data  接收到的数据缓冲区
 * @param Length   数据长度（SBUS 帧一般为 25 字节）
 */
void Fusi_RC_UART_RxCpltCallback(Class_Fusi_RC *self, uint8_t *Rx_Data, uint16_t Length)
{
    Fusi_RC_Data_Process(self, Rx_Data);
}

/**
 * @brief 原始数据解析（按照用户提供的 SBUS 协议）
 * @param self     Class_Fusi_RC 指针
 * @param Rx_Data  SBUS 数据帧（长度至少 24 字节）
 */
void Fusi_RC_Data_Process(Class_Fusi_RC *self, uint8_t *Rx_Data)
{
    if (self == NULL || Rx_Data == NULL)
    {
        return;
    }

    // 断控保护标志提取（bit3 of byte23）
    self->Fusi_RC_Status = Fusi_RC_Status_ENABLE;//! ((Rx_Data[23] & 0x08) >> 3);

     if (Rx_Data[0] == 0x0f && Rx_Data[23] == 0x00)
     {
        // 解析 15 个通道（每个通道 11 位）
        self->Raw_Data.SBUS.CH[1]  = (Rx_Data[1]      | (Rx_Data[2] << 8)) & 0x07FF;
        self->Raw_Data.SBUS.CH[2]  = ((Rx_Data[2] >> 3) | (Rx_Data[3] << 5)) & 0x07FF;
        self->Raw_Data.SBUS.CH[3]  = ((Rx_Data[3] >> 6) | (Rx_Data[4] << 2) | (Rx_Data[5] << 10)) & 0x07FF;
        self->Raw_Data.SBUS.CH[4]  = ((Rx_Data[5] >> 1) | (Rx_Data[6] << 7)) & 0x07FF;
        self->Raw_Data.SBUS.CH[5]  = ((Rx_Data[6] >> 4) | (Rx_Data[7] << 4)) & 0x07FF;
        self->Raw_Data.SBUS.CH[6]  = ((Rx_Data[7] >> 7) | (Rx_Data[8] << 1) | (Rx_Data[9] << 9)) & 0x07FF;
        self->Raw_Data.SBUS.CH[7]  = ((Rx_Data[9] >> 2) | (Rx_Data[10] << 6)) & 0x07FF;
        self->Raw_Data.SBUS.CH[8]  = ((Rx_Data[10] >> 5) | (Rx_Data[11] << 3)) & 0x07FF;
        self->Raw_Data.SBUS.CH[9]  = (Rx_Data[12]       | (Rx_Data[13] << 8)) & 0x07FF;
        self->Raw_Data.SBUS.CH[10]  = ((Rx_Data[13] >> 3) | (Rx_Data[14] << 5)) & 0x07FF;
        self->Raw_Data.SBUS.CH[11] = ((Rx_Data[14] >> 6) | (Rx_Data[15] << 2) | (Rx_Data[16] << 10)) & 0x07FF;
        self->Raw_Data.SBUS.CH[12] = ((Rx_Data[16] >> 1) | (Rx_Data[17] << 7)) & 0x07FF;
        self->Raw_Data.SBUS.CH[13] = ((Rx_Data[17] >> 4) | (Rx_Data[18] << 4)) & 0x07FF;
        self->Raw_Data.SBUS.CH[14] = ((Rx_Data[18] >> 7) | (Rx_Data[19] << 1) | (Rx_Data[20] << 9)) & 0x07FF;
        self->Raw_Data.SBUS.CH[15] = ((Rx_Data[20] >> 2) | (Rx_Data[21] << 6)) & 0x07FF;

        self->Raw_Data.RC.ch[1] = (self->Raw_Data.SBUS.CH[1] - 1024);
        self->Raw_Data.RC.ch[2] = (self->Raw_Data.SBUS.CH[2] - 1024);
        self->Raw_Data.RC.ch[3] = (self->Raw_Data.SBUS.CH[3] - 1024);
        self->Raw_Data.RC.ch[4] = (self->Raw_Data.SBUS.CH[4] - 1024);
        self->Raw_Data.RC.rotary_sw[1] = (self->Raw_Data.SBUS.CH[5] - 240);
        self->Raw_Data.RC.rotary_sw[2] = (self->Raw_Data.SBUS.CH[6] - 240);

        switch (self->Raw_Data.SBUS.CH[7])
        {
        case 240:
            self->Raw_Data.RC.sw[1] = 1;
            break;

        case 1807:
            self->Raw_Data.RC.sw[1] = 2;
            break;
        }
        switch (self->Raw_Data.SBUS.CH[8])
        {
        case 240:
            self->Raw_Data.RC.sw[2] = 1;
            break;

        case 1807:
            self->Raw_Data.RC.sw[2] = 2;
            break;
        }
        switch (self->Raw_Data.SBUS.CH[9])
        {
        case 240:
            self->Raw_Data.RC.sw[3] = 1;
            break;

        case 1807:
            self->Raw_Data.RC.sw[3] = 2;
            break;
        case 1024:
            self->Raw_Data.RC.sw[3] = 3;
            break;
        }
        switch (self->Raw_Data.SBUS.CH[10])
        {
        case 240:
            self->Raw_Data.RC.sw[4] = 1;
            break;

        case 1807:
            self->Raw_Data.RC.sw[4] = 2;
            break;
        }

        // 转换为处理后数据
        Fusi_RC_Raw_To_Process(self);
    }

    if (self->Fusi_RC_Status==Fusi_RC_Status_DISABLE)
    {
        memset(&self->Raw_Data, 0, sizeof(self->Raw_Data));
        memset(&self->Processed_Data, 0, sizeof(self->Processed_Data));
        memset(&self->Last_Processed_Data, 0, sizeof(self->Last_Processed_Data));
    }

}

/**
 * @brief 将原始通道值转换为上层可用的控制量
 * @param self Class_Fusi_RC 指针
 */
void Fusi_RC_Raw_To_Process(Class_Fusi_RC *self)
{
    if (self == NULL)
    {
        return;
    }

    self->Processed_Data.Right_X = (self->Raw_Data.RC.ch[1]) / 783.0f;
    self->Processed_Data.Right_Y = (self->Raw_Data.RC.ch[2]) / 783.0f;
    self->Processed_Data.Left_Y = (self->Raw_Data.RC.ch[3]) / 783.0f;
    self->Processed_Data.Left_X = (self->Raw_Data.RC.ch[4]) / 783.0f;
    self->Processed_Data.Rotary_Switch_Left = (self->Raw_Data.RC.rotary_sw[1]) / 1567.0f;
    self->Processed_Data.Rotary_Switch_Right = (self->Raw_Data.RC.rotary_sw[2]) / 1567.0f;

    self->Processed_Data.Left_Switch = self->Raw_Data.RC.sw[1];
    self->Processed_Data.Left_Mid_Switch = self->Raw_Data.RC.sw[2];
    self->Processed_Data.Right_Mid_Switch = self->Raw_Data.RC.sw[3];
    self->Processed_Data.Right_Switch = self->Raw_Data.RC.sw[4];
}



void Class_Fusi_TIM_1ms_Calculate_PeriodElapsedCallback(Class_Fusi_RC *self)
{
    // 判断拨码触发
    Class_Fusi_Three_Stage_Judge_Switch(&self->Processed_Data.Right_Mid_Switch_Status, self->Processed_Data.Right_Mid_Switch, self->Last_Processed_Data.Right_Mid_Switch);

    Class_Fusi_Two_Stage_Judge_Switch(&self->Processed_Data.Left_Mid_Switch_Status, self->Processed_Data.Left_Mid_Switch, self->Last_Processed_Data.Left_Mid_Switch);
    Class_Fusi_Two_Stage_Judge_Switch(&self->Processed_Data.Left_Switch_Status, self->Processed_Data.Left_Switch, self->Last_Processed_Data.Left_Switch);
    Class_Fusi_Two_Stage_Judge_Switch(&self->Processed_Data.Right_Switch_Status, self->Processed_Data.Right_Switch, self->Last_Processed_Data.Right_Switch);

    // 保留数据
    memcpy(&self->Last_Processed_Data, &self->Processed_Data, sizeof(Fusi_RC_Processed_Data_t));
}

/**
 * @brief 判断拨动开关状态
 * @param Switch 三阶段拨动开关状态指针
 */
void Class_Fusi_Three_Stage_Judge_Switch(Enum_Fusi_Three_Stage_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
        case (FUSI_SWITCH_UP):
        {
            switch (Status)
            {
                case (FUSI_SWITCH_UP):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_UP;

                    break;
                }

                case (FUSI_SWITCH_MID):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_UP_TRIG_MIDDLE;

                        fusi_up_to_mid++;

                    break;
                }
            }

            break;
        }
        case (FUSI_SWITCH_DOWN):
        {
            switch (Status)
            {
                case (FUSI_SWITCH_DOWN):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_DOWN;

                    break;
                }
                case (FUSI_SWITCH_MID):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_DOWN_TRIG_MIDDLE;

                        fusi_down_to_mid++;

                    break;
                }
            }

            break;
        }
        case (FUSI_SWITCH_MID):
        {
            switch (Status)
            {
                case (FUSI_SWITCH_UP):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_MIDDLE_TRIG_UP;

                        fusi_mid_to_up++;

                    break;
                }
                case (FUSI_SWITCH_DOWN):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_MIDDLE_TRIG_DOWN;

                        fusi_mid_to_down++;

                    break;
                }
                case (FUSI_SWITCH_MID):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_MIDDLE;

                    break;
                }
            }

            break;
        }
    }
}


/**
 * @brief 判断拨动开关状态
 * @param Switch 二阶段拨动开关状态指针
 */
void Class_Fusi_Two_Stage_Judge_Switch(Enum_Fusi_Two_Stage_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
        case (FUSI_SWITCH_UP):
        {
            switch (Status)
            {
                case (FUSI_SWITCH_UP):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_UP;

                    break;
                }

                case (FUSI_SWITCH_DOWN):
                {
                    *Switch = Fusi_Two_Stage_Switch_Status_UP_TRIG_DOWN;

                        fusi_up_to_down++;

                    break;
                }
            }

            break;
        }
        case (FUSI_SWITCH_DOWN):
        {
            switch (Status)
            {
                case (FUSI_SWITCH_DOWN):
                {
                    *Switch = Fusi_Three_Stage_Switch_Status_DOWN;

                    break;
                }
                case (FUSI_SWITCH_UP):
                {
                    *Switch = Fusi_Two_Stage_Switch_Status_DOWN_TRIG_UP;

                        fusi_down_to_up++;

                    break;
                }
            }

            break;
        }

    }
}



/**
 * @brief 获取原始数据指针
 * @return const Fusi_RC_Raw_Data_t* 原始数据指针
 */
const Fusi_RC_Raw_Data_t *get_fusi_rc_raw_data_point(void)
{
    return &class_fusi_rc.Raw_Data;
}

/**
 * @brief 获取处理后数据指针
 * @return const Fusi_RC_Processed_Data_t* 处理后数据指针
 */
const Fusi_RC_Processed_Data_t *get_fusi_rc_processed_data_point(void)
{
    return &class_fusi_rc.Processed_Data;
}