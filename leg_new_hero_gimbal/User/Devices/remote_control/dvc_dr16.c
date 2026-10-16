#include "dvc_dr16.h"
#include "string.h"

//函数前置声明
void DR16_UART_RxCpltCallback(Class_DR16 *self,uint8_t *Rx_Data, uint16_t Length);
void Class_DR16_Data_Process(Class_DR16 *self, uint8_t *Rx_Data);
void Class_DR16_Data_Raw_TO_Process(Class_DR16 *self);
void Class_DR16_Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status);
void Class_DR16_Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status);

Class_DR16 class_dr16;

uint16_t dr16_down_to_mid = 0;
uint16_t dr16_mid_to_down = 0;
uint16_t dr16_up_to_mid = 0;
uint16_t dr16_mid_to_up = 0;

void DR16_Init(Class_DR16 *dr16, UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        dr16->UART_Manage_Object = &UART1_Manage_Object;
    }
    else if (huart->Instance == USART2)
    {
        dr16->UART_Manage_Object = &UART2_Manage_Object;
    }
    else if (huart->Instance == USART3)
    {
        dr16->UART_Manage_Object = &UART3_Manage_Object;
    }
    else if (huart->Instance == UART4)
    {
        dr16->UART_Manage_Object = &UART4_Manage_Object;
    }
    else if (huart->Instance == UART5)
    {
        dr16->UART_Manage_Object = &UART5_Manage_Object;
    }
    else if (huart->Instance == USART6)
    {
        dr16->UART_Manage_Object = &UART6_Manage_Object;
    }
    else if (huart->Instance == UART7)
    {
        dr16->UART_Manage_Object = &UART7_Manage_Object;
    }
    else if (huart->Instance == UART8)
    {
        dr16->UART_Manage_Object = &UART8_Manage_Object;
    }

    dr16->Rocker_Offset = 1024.0f;
    dr16->Rocker_Num = 660.0f;
    dr16->Flag = 0;
    dr16->Last_Flag = 0;
    dr16->DR16_Status = DR16_Status_DISABLE;

    memset(&dr16->Raw_Data, 0, sizeof(dr16->Raw_Data));
    memset(&dr16->Processed_Data, 0, sizeof(dr16->Processed_Data));
    memset(&dr16->Last_Processed_Data, 0, sizeof(dr16->Last_Processed_Data));
}

void DR16_UART_RxCpltCallback(Class_DR16 *self,uint8_t *Rx_Data, uint16_t Length)
{
    self->Flag += 1;
    Class_DR16_Data_Process(self,Rx_Data);
}

/**
 * @brief TIM定时器中断定期检测遥控器DR16是否存活
 *
 */
void Class_DR16_TIM_100ms_Alive_PeriodElapsedCallback(Class_DR16 *self)
{
    // 判断该时间段内是否接收过遥控器DR16数据
    if (self->Flag == self->Last_Flag)
    {
        // 遥控器DR16断开连接
        self->DR16_Status = DR16_Status_DISABLE;

        memset(&self->Raw_Data, 0, sizeof(self->Raw_Data));
        memset(&self->Processed_Data, 0, sizeof(self->Processed_Data));
        memset(&self->Last_Processed_Data, 0, sizeof(self->Last_Processed_Data));
        //UART_Reinit(self->UART_Manage_Object->UART_Handler);
    }
    else
    {
        // 遥控器DR16保持连接
        self->DR16_Status = DR16_Status_ENABLE;
    }

    self->Last_Flag = self->Flag;

}

// ==============================
void Class_DR16_Data_Process(Class_DR16 *self, uint8_t *Rx_Data)
{
    // ==================== 通道解析 ====================
    self->Raw_Data.ch[0] = (Rx_Data[0] | (Rx_Data[1] << 8)) & 0x07ff;        //!< Channel 0
    self->Raw_Data.ch[1] = ((Rx_Data[1] >> 3) | (Rx_Data[2] << 5)) & 0x07ff; //!< Channel 1
    self->Raw_Data.ch[2] = ((Rx_Data[2] >> 6) | (Rx_Data[3] << 2) | (Rx_Data[4] << 10)) &0x07ff;//!< Channel 2
    self->Raw_Data.ch[3] = ((Rx_Data[4] >> 1) | (Rx_Data[5] << 7)) & 0x07ff; //!< Channel 3
    self->Raw_Data.ch[4] = Rx_Data[16] | (Rx_Data[17] << 8);                 //NULL

    // ==================== 开关解析 ====================
    self->Raw_Data.s[0] =((Rx_Data[5] >> 4) & 0x0003);                  //!< Switch left
    self->Raw_Data.s[1] =((Rx_Data[5] >> 4) & 0x000C) >> 2;                       //!< Switch right

    // ==================== 鼠标解析 ====================
    self->Raw_Data.x = Rx_Data[6] | (Rx_Data[7] << 8);                    //!< Mouse X axis
    self->Raw_Data.y = Rx_Data[8] | (Rx_Data[9] << 8);                    //!< Mouse Y axis
    self->Raw_Data.z = Rx_Data[10] | (Rx_Data[11] << 8);                  //!< Mouse Z axis
    self->Raw_Data.press_l = Rx_Data[12];                                  //!< Mouse Left Is Press ?
    self->Raw_Data.press_r = Rx_Data[13];                                  //!< Mouse Right Is Press ?
    // ==================== 键盘按键解析 ====================
    self->Raw_Data.key = Rx_Data[14] | (Rx_Data[15] << 8);                    //!< KeyBoard value

    self->Raw_Data.ch[0] -= self->Rocker_Offset;
    self->Raw_Data.ch[1] -= self->Rocker_Offset;
    self->Raw_Data.ch[2] -= self->Rocker_Offset;
    self->Raw_Data.ch[3] -= self->Rocker_Offset;
    self->Raw_Data.ch[4] -= self->Rocker_Offset;

    Class_DR16_Data_Raw_TO_Process(self);
}

void Class_DR16_Data_Raw_TO_Process(Class_DR16 *self)
{
    self->Processed_Data.Right_X = self->Raw_Data.ch[0] / self->Rocker_Num;
    self->Processed_Data.Right_Y = self->Raw_Data.ch[1] / self->Rocker_Num;
    self->Processed_Data.Left_X = self->Raw_Data.ch[2] / self->Rocker_Num;
    self->Processed_Data.Left_Y = self->Raw_Data.ch[3] / self->Rocker_Num;
    self->Processed_Data.Yaw = self->Raw_Data.ch[4] / self->Rocker_Num;

    self->Processed_Data.Mouse_X = self->Raw_Data.x / 32768.0f;
    self->Processed_Data.Mouse_Y = self->Raw_Data.y / 32768.0f;
    self->Processed_Data.Mouse_Z = self->Raw_Data.z / 32768.0f;

    self->Processed_Data.Right_Switch = self->Raw_Data.s[0];
    self->Processed_Data.Left_Switch = self->Raw_Data.s[1];

    self->Processed_Data.Mouse_Left_Key = self->Raw_Data.press_l;
    self->Processed_Data.Mouse_Right_Key = self->Raw_Data.press_r;
    self->Processed_Data.Key = self->Raw_Data.key;
}


void Class_DR16_TIM_1ms_Calculate_PeriodElapsedCallback(Class_DR16 *self)
{
    // 判断拨码触发
    Class_DR16_Judge_Switch(&self->Processed_Data.Left_Switch_Status, self->Processed_Data.Left_Switch, self->Last_Processed_Data.Left_Switch);
    Class_DR16_Judge_Switch(&self->Processed_Data.Right_Switch_Status, self->Processed_Data.Right_Switch, self->Last_Processed_Data.Right_Switch);

    // 判断鼠标触发
    Class_DR16_Judge_Key(&self->Processed_Data.Mouse_Left_Key_Status, self->Processed_Data.Mouse_Left_Key, self->Last_Processed_Data.Mouse_Left_Key);
    Class_DR16_Judge_Key(&self->Processed_Data.Mouse_Right_Key_Status, self->Processed_Data.Mouse_Right_Key, self->Last_Processed_Data.Mouse_Right_Key);

    // 判断键盘触发
    for (int i = 0; i < 16; i++)
    {
        Class_DR16_Judge_Key(&self->Processed_Data.Keyboard_Key_Status[i], (self->Processed_Data.Key >> i) & 0x1, (self->Last_Processed_Data.Key >> i) & 0x1);
    }

    // 保留数据
    memcpy(&self->Last_Processed_Data, &self->Processed_Data, sizeof(DR16_Processed_Data_t));
}

/**
 * @brief 判断拨动开关状态
 *
 */
void Class_DR16_Judge_Switch(Enum_DR16_Switch_Status *Switch, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
        case (DR16_SWITCH_UP):
        {
            switch (Status)
            {
                case (DR16_SWITCH_UP):
                {
                    *Switch = DR16_Switch_Status_UP;

                    break;
                }
                case (DR16_SWITCH_MID):
                {
                    *Switch = DR16_Switch_Status_UP_TRIG_MIDDLE;

                    dr16_up_to_mid++;

                    break;
                }

            }

            break;
        }
        case (DR16_SWITCH_DOWN):
        {
            switch (Status)
            {
                case (DR16_SWITCH_DOWN):
                {
                    *Switch = DR16_Switch_Status_DOWN;

                    break;
                }
                case (DR16_SWITCH_MID):
                {
                    *Switch = DR16_Switch_Status_DOWN_TRIG_MIDDLE;

                        dr16_down_to_mid++;

                    break;
                }
            }

            break;
        }
        case (DR16_SWITCH_MID):
        {
            switch (Status)
            {
                case (DR16_SWITCH_UP):
                {
                    *Switch = DR16_Switch_Status_MIDDLE_TRIG_UP;

                        dr16_mid_to_up++;

                    break;
                }
                case (DR16_SWITCH_DOWN):
                {
                    *Switch = DR16_Switch_Status_MIDDLE_TRIG_DOWN;

                        dr16_mid_to_down++;

                    break;
                }
                case (DR16_SWITCH_MID):
                {
                    *Switch = DR16_Switch_Status_MIDDLE;

                    break;
                }
            }

            break;
        }
    }
}

/**
 * @brief 判断按键状态
 *
 */
void Class_DR16_Judge_Key(Enum_DR16_Key_Status *Key, uint8_t Status, uint8_t Pre_Status)
{
    // 带触发的判断
    switch (Pre_Status)
    {
    case (DR16_KEY_FREE):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
        {
            *Key = DR16_Key_Status_FREE;

            break;
        }
        case (DR16_KEY_PRESSED):
        {
            *Key = DR16_Key_Status_TRIG_FREE_PRESSED;

            break;
        }
        }

        break;
    }
    case (DR16_KEY_PRESSED):
    {
        switch (Status)
        {
        case (DR16_KEY_FREE):
        {
            *Key = DR16_Key_Status_TRIG_PRESSED_FREE;

            break;
        }
        case (DR16_KEY_PRESSED):
        {
            *Key = DR16_Key_Status_PRESSED;

            break;
        }
        }

        break;
    }
    }
}



const DR16_Raw_Data_t *get_dr16_raw_data_point(void)
{
    return &class_dr16.Raw_Data;
}

const DR16_Processed_Data_t *get_dr16_processed_data_point(void)
{
    return &class_dr16.Processed_Data;
}