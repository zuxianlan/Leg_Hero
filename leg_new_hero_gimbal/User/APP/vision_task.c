#include "vision_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usbd_cdc_if.h"

#include "dvc_referee.h"
#include "gimbal_task.h"
#include "gimbal_behaviour.h"
#include "shoot_task.h"


// 视觉任务初始化
static void vision_task_init(vision_control_t* init);
static void vision_task_update(vision_control_t* vison_update);

static void FYT_Vision_Send_Packet_Update(FYT_vision_t *vision);
static void FYT_Vision_Send_Packet(FYT_vision_t *vision);
static void FYT_Vision_Data_Update(FYT_vision_t *vision);

static void SP_Vision_Send_Packet_Update(SP_vision_t *vision);
static void SP_Vision_Send_Packet(SP_vision_t *vision);
static void SP_Vision_Data_Update(SP_vision_t *vision);

uint16_t Vision_Verify_CRC_16(uint8_t *Message, uint32_t Length);


// 总视觉任务结构体
vision_control_t vision_control = { 0 };
// FYT自瞄任务结构体
FYT_vision_t FYT_vision = {0};
// SP自瞄任务结构体
SP_vision_t SP_vision = {0};


void Vision_Task(void const* pvParameters)
{
    // 延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);

    // 总视觉任务初始化
    vision_task_init(&vision_control);

    // 系统延时
    vTaskDelay(VISION_CONTROL_TIME_MS);

    while(1)
	{

        // 更新总视觉数据
        vision_task_update(&vision_control);


        // //FYT码------------------------------------------
        // // 更新发送数据包
        //  FYT_Vision_Send_Packet_Update(&FYT_vision);
        // //  发送数据包
        //  FYT_Vision_Send_Packet(&FYT_vision);
        //
        // // 自瞄数据接受更新
        //  FYT_Vision_Data_Update(&FYT_vision);
        //------------------------------------------------------


        // //SP码------------------------------------------
         // // 更新发送数据包
         SP_Vision_Send_Packet_Update(&SP_vision);
         // // 发送数据包
         SP_Vision_Send_Packet(&SP_vision);

         // //自瞄数据接受更新
         SP_Vision_Data_Update(&SP_vision);

        //-------------------------------------------------------


        // 系统延时
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

//总视觉任务初始化
static void vision_task_init(vision_control_t* init)
{
    // 获取陀螺仪绝对角指针
    init->vision_INS_point = get_INS_point();

    // 更新机器人id
    init->robot_id = class_referee.Referee_Rx_Data.Robot_Status.robot_id;

    // 判断识别颜色
    if (init->robot_id < ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        init->detect_armor_color = BLUE;
    }
    else
    {
        init->detect_armor_color = RED;
    }
}

// 更新总视觉任务结构体
static void vision_task_update(vision_control_t* vison_update)
{
    // 更新裁判们系统id
    vison_update->robot_id = class_referee.Referee_Rx_Data.Robot_Status.robot_id;

    // 判断识别颜色
    if (vison_update->robot_id < ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        vison_update->detect_armor_color = BLUE;
    }
    else
    {
        vison_update->detect_armor_color = RED;
    }

}


//FYT自瞄任务---------------------------------------------------------------
static void FYT_Vision_Send_Packet_Update(FYT_vision_t *vision)
{
    // 更新发送上位机数据
    vision->vision_send_packet.header = 0xff;
    vision->vision_send_packet.mod = vision_control.detect_armor_color;
    vision->vision_send_packet.roll = vision_control.vision_INS_point->Roll;
    vision->vision_send_packet.pitch = -vision_control.vision_INS_point->Pitch;
    vision->vision_send_packet.yaw = vision_control.vision_INS_point->Yaw;
    // 待定暂且不使用
    vision->vision_send_packet.fill = 0;
    vision->vision_send_packet.ender = 0x0d;

}

static void FYT_Vision_Send_Packet(FYT_vision_t *vision)
{
    FYT_vision_send_packet_t *tmp_buff = (FYT_vision_send_packet_t*) vision->tx_buf;

    tmp_buff->header = vision->vision_send_packet.header;
    tmp_buff->mod = vision->vision_send_packet.mod;
    tmp_buff->roll = vision->vision_send_packet.roll;
    tmp_buff->pitch = vision->vision_send_packet.pitch;
    tmp_buff->yaw = vision->vision_send_packet.yaw;
    tmp_buff->fill = 0;
    tmp_buff->ender = vision->vision_send_packet.ender;

    CDC_Transmit_HS(vision->tx_buf, sizeof(FYT_vision_send_packet_t));
}

static void FYT_Vision_Data_Update(FYT_vision_t *vision)
{
    static float last_yaw = 0;
    static float last_pitch = 0;

    //数据更新----------
    if (vision->Rx_data.distance == -1)
    {
        vision_control.gimbal_vision_control.gimbal_yaw = last_yaw;
        vision_control.gimbal_vision_control.gimbal_pitch = last_pitch;
    }
    else
    {
        vision_control.gimbal_vision_control.gimbal_yaw = vision->Rx_data.yaw;
        vision_control.gimbal_vision_control.gimbal_pitch = vision->Rx_data.pitch;
    }

    if (vision->Rx_data.distance != -1)
    {
        last_yaw = vision->Rx_data.yaw;
        last_pitch = vision->Rx_data.pitch;
    }
    //---------------------

    //模式更新---------------
    if ( vision->Rx_data.distance != -1 && vision->Rx_data.fire == 1 )
    {
        vision_control.vision_control_mode = VISION_CONTROL_FIRE;
    }
    else if (vision->Rx_data.distance != -1 && vision->Rx_data.fire != 1)
    {
        vision_control.vision_control_mode = VISION_ONLY_CONTROL_GIMBAL;
    }
    else
    {
        vision_control.vision_control_mode = VISION_NO_CONTROL;
    }

    if (vision_control.vision_control_mode == VISION_NO_CONTROL )
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 0;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 0;

        vision_control.shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }
    else if (vision_control.vision_control_mode == VISION_ONLY_CONTROL_GIMBAL)
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 0;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 1;

        vision_control.shoot_vision_control.shoot_command = SHOOT_READY_ATTACK;
    }
    else if (vision_control.vision_control_mode == VISION_CONTROL_FIRE)
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 1;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 1;

        vision_control.shoot_vision_control.shoot_command = SHOOT_ATTACK;
    }
    else
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 0;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 0;

        vision_control.shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }
    //--------------------------

}

void FYT_Vision_Receive_Packet(FYT_vision_t *vision,uint8_t *buf, uint32_t len)
{

    FYT_vision_receive_packet_t *tmp_buff = (FYT_vision_receive_packet_t *)buf;

    vision->Rx_data.header = tmp_buff->header;
    vision->Rx_data.fire = tmp_buff->fire;
    vision->Rx_data.pitch = tmp_buff->pitch;
    vision->Rx_data.yaw = tmp_buff->yaw;
    vision->Rx_data.distance = tmp_buff->distance;

}
//------------------------------------------------------------




//SP自瞄任务---------------------------------------------------------------
static void SP_Vision_Send_Packet_Update(SP_vision_t* vision)
{
    float initial_speed=0.0f;

    if (class_referee.Referee_Rx_Data.Robot_Booster.initial_speed != 0.0f)
    {
        initial_speed = class_referee.Referee_Rx_Data.Robot_Booster.initial_speed;
    }
    else
    {
        initial_speed = 11.5f;
    }

    // 更新发送上位机数据
    vision->vision_send_packet.head[0] = 'S';
    vision->vision_send_packet.head[1] = 'P';
    vision->vision_send_packet.mode = 1 ;
    memcpy(vision->vision_send_packet.q, vision_control.vision_INS_point->q, 16);
    vision->vision_send_packet.yaw = vision_control.vision_INS_point->Yaw;
    vision->vision_send_packet.yaw_omega = vision_control.vision_INS_point->Gyro[2];
    vision->vision_send_packet.pitch = vision_control.vision_INS_point->Pitch;
    vision->vision_send_packet.pitch_omega = vision_control.vision_INS_point->Gyro[1];
    vision->vision_send_packet.bullet_speed = initial_speed;
    vision->vision_send_packet.enemy_color= vision_control.detect_armor_color;
    vision->vision_send_packet.bullet_count = 1;
    vision->vision_send_packet.crc16 = Vision_Verify_CRC_16((uint8_t *) &vision->vision_send_packet, sizeof(SP_vision_send_packet_t)-2);
}

static void SP_Vision_Send_Packet(SP_vision_t* vision)
{
    SP_vision_send_packet_t* tmp_buff = (SP_vision_send_packet_t*)vision->tx_buf;

    memcpy(tmp_buff->head, vision->vision_send_packet.head, 2);
    tmp_buff->mode = vision->vision_send_packet.mode;
    memcpy(tmp_buff->q, vision->vision_send_packet.q, 16);
    tmp_buff->yaw = vision->vision_send_packet.yaw;
    tmp_buff->yaw_omega = vision->vision_send_packet.yaw_omega;
    tmp_buff->pitch = vision->vision_send_packet.pitch;
    tmp_buff->pitch_omega = vision->vision_send_packet.pitch_omega;
    tmp_buff->bullet_speed = vision->vision_send_packet.bullet_speed;
    tmp_buff->enemy_color = vision->vision_send_packet.enemy_color;
    tmp_buff->bullet_count = vision->vision_send_packet.bullet_count;
    tmp_buff->crc16 = vision->vision_send_packet.crc16;

    CDC_Transmit_HS(vision->tx_buf, sizeof(SP_vision_send_packet_t));
}

static void SP_Vision_Data_Update(SP_vision_t* vision)
{
    vision_control.vision_control_mode = vision->Rx_data.mode;
    // vision_control.gimbal_vision_control.gimbal_yaw = vision->Rx_data.yaw;
    // vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_omega = vision->Rx_data.yaw_omega;
    // vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_accel = vision->Rx_data.yaw_accel;
    // vision_control.gimbal_vision_control.gimbal_pitch = -vision->Rx_data.pitch;
    // vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_omega = vision->Rx_data.pitch_omega;
    // vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_accel = vision->Rx_data.pitch_accel;

    if (vision_control.vision_control_mode== VISION_NO_CONTROL )
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 0;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 0;

        vision_control.shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }
    else if (vision_control.vision_control_mode== VISION_ONLY_CONTROL_GIMBAL)
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 0;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 1;

        vision_control.shoot_vision_control.shoot_command = SHOOT_READY_ATTACK;
    }
    else if (vision_control.vision_control_mode == VISION_CONTROL_FIRE)
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 1;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 1;

        vision_control.shoot_vision_control.shoot_command = SHOOT_ATTACK;
    }
    else
    {
        vision_control.shoot_vision_control.vision_control_fire_flag = 0;
        vision_control.gimbal_vision_control.vision_control_gimbal_flag = 0;

        vision_control.shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

    static float last_yaw = 0;
    static float last_yaw_forward_omega = 0;
    static float last_yaw_forward_accel = 0;
    static float last_pitch = 0;
    static float last_pitch_forward_omega = 0;
    static float last_pitch_forward_accel = 0;

    //数据更新----------
    if (vision->Rx_data.distance == -1)
    {
        vision_control.gimbal_vision_control.gimbal_yaw = last_yaw;
        vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_omega = last_yaw_forward_omega;
        vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_accel = last_yaw_forward_accel;

        vision_control.gimbal_vision_control.gimbal_pitch = last_pitch;
        vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_omega = last_pitch_forward_omega;
        vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_accel = last_pitch_forward_accel;
    }
    else
    {
        vision_control.gimbal_vision_control.gimbal_yaw = vision->Rx_data.yaw;
        vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_omega = vision->Rx_data.yaw_omega;
        vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_accel = vision->Rx_data.yaw_accel;
        vision_control.gimbal_vision_control.gimbal_pitch = -vision->Rx_data.pitch;
        vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_omega = vision->Rx_data.pitch_omega;
        vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_accel = vision->Rx_data.pitch_accel;
    }

    if (vision->Rx_data.distance != -1)
    {
        last_yaw = vision->Rx_data.yaw;
        last_yaw_forward_omega = vision->Rx_data.yaw_omega;
        last_yaw_forward_accel = vision->Rx_data.yaw_accel;
        last_pitch = -vision->Rx_data.pitch;
        last_pitch_forward_omega = vision->Rx_data.pitch_omega;
        last_pitch_forward_accel = vision->Rx_data.pitch_accel;
    }

    if ( isnan(vision_control.gimbal_vision_control.gimbal_pitch) !=0  )
    {
        vision_control.gimbal_vision_control.gimbal_yaw = 0;
        vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_omega = 0;
        vision_control.gimbal_vision_control.gimbal_feed_forward_yaw_accel = 0;
        vision_control.gimbal_vision_control.gimbal_pitch = 0;
        vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_omega= 0;
        vision_control.gimbal_vision_control.gimbal_feed_forward_pitch_accel= 0;

        vision_control.gimbal_vision_control.vision_control_gimbal_flag=0;
        vision_control.shoot_vision_control.vision_control_fire_flag=0;
    }
}

void SP_Vision_Receive_Packet(SP_vision_t *vision,uint8_t *buf, uint32_t len)
{
    SP_vision_receive_packet_t* tmp_buff = (SP_vision_receive_packet_t*)buf;

    // if (Verify_CRC_16((uint8_t *) tmp_buff, sizeof(vision_receive_SP_packet_t) - 2) != tmp_buff->crc16)
    //     {
    //         return;
    //     }

    vision->Rx_data.head[0] = tmp_buff->head[0];
    vision->Rx_data.head[1] = tmp_buff->head[1];
    vision->Rx_data.mode = tmp_buff->mode;
    vision->Rx_data.yaw = tmp_buff->yaw;
    vision->Rx_data.yaw_omega = tmp_buff->yaw_omega;
    vision->Rx_data.yaw_accel = tmp_buff->yaw_accel;
    vision->Rx_data.pitch = tmp_buff->pitch;
    vision->Rx_data.pitch_omega = tmp_buff->pitch_omega;
    vision->Rx_data.pitch_accel = tmp_buff->pitch_accel;
    vision->Rx_data.distance = tmp_buff->distance;


    vision->Rx_data_Origin.head[0] = tmp_buff->head[0];
    vision->Rx_data_Origin.head[1] = tmp_buff->head[1];
    vision->Rx_data_Origin.mode = tmp_buff->mode;
    vision->Rx_data_Origin.yaw = tmp_buff->yaw;
    vision->Rx_data_Origin.yaw_omega = tmp_buff->yaw_omega;
    vision->Rx_data_Origin.yaw_accel = tmp_buff->yaw_accel;
    vision->Rx_data_Origin.pitch = tmp_buff->pitch;
    vision->Rx_data_Origin.pitch_omega = tmp_buff->pitch_omega;
    vision->Rx_data_Origin.pitch_accel = tmp_buff->pitch_accel;
    vision->Rx_data_Origin.distance = tmp_buff->distance;
    vision->Rx_data_Origin.crc16 = tmp_buff->crc16;
}

//------------------------------------------------------------------------



//对USB数据进行收取，不同自瞄接口不一样
void receive_decode(uint8_t *buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }

    //FYT_Vision_Receive_Packet(&FYT_vision,buf, len);

    SP_Vision_Receive_Packet(&SP_vision,buf, len);
}



// CRC16校验码表
static const uint16_t vision_crc_16_table[256] = {
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf, 0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e, 0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd, 0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c, 0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb, 0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a, 0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9, 0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738, 0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7, 0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036, 0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5, 0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134, 0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3, 0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232, 0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1, 0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330, 0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t Vision_Verify_CRC_16(uint8_t *Message, uint32_t Length)
{
    uint8_t index;
    uint16_t check = 0xffff;

    if (Message == NULL)
    {
        return check;
    }

    while (Length--)
    {
        index = *Message;
        Message++;
        check = ((uint16_t)(check) >> 8) ^ vision_crc_16_table[((uint16_t)(check) ^ (uint16_t)(index)) & 0xff];
    }
    return check;
}



// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}


// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}

