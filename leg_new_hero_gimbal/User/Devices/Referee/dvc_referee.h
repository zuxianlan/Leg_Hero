#ifndef CTRLBOARD_H7_IMU_DVC_REFEREE_H
#define CTRLBOARD_H7_IMU_DVC_REFEREE_H

/**
 * @file dvc_referee.h
 * @author yssickjgd (1345578933@qq.com)
 * @brief PM01裁判系统
 * @version 0.1
 * @date 2023-08-29 0.1 23赛季定稿
 * @date 2024-01-30 1.1 适配1.6.1通信协议
 *
 * @copyright USTC-RoboWalker (c) 2023-2024
 *
 */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <string.h>
#include "bsp_usart.h"
/* Exported macros -----------------------------------------------------------*/
#define REFEREE_RXBUF_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 裁判系统状态
 */
typedef enum
{
    Referee_Status_DISABLE = 0,
    Referee_Status_ENABLE,
} Enum_Referee_Status;

/**
 * @brief 裁判系统数据可用状态
 */
typedef enum
{
    Referee_Data_Status_DISABLE = 0,
    Referee_Data_Status_ENABLE,
} Enum_Referee_Data_Status;

/**
 * @brief 图形操作交互信息
 */
typedef enum
{
    Referee_Data_Interaction_Layer_Delete_Operation_NULL = 0,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ONE,
    Referee_Data_Interaction_Layer_Delete_Operation_CLEAR_ALL,
} Enum_Referee_Data_Interaction_Layer_Delete_Operation;

typedef enum
{
    Referee_Interaction_Command_ID_UI_LAYER_DELETE = 0x0100,
    Referee_Interaction_Command_ID_UI_GRAPHIC_1,
    Referee_Interaction_Command_ID_UI_GRAPHIC_2,
    Referee_Interaction_Command_ID_UI_GRAPHIC_5,
    Referee_Interaction_Command_ID_UI_GRAPHIC_7,
    Referee_Interaction_Command_ID_UI_GRAPHIC_STRING = 0x0110,
    Referee_Interaction_Command_ID_SENTRY = 0x0120,
    Referee_Interaction_Command_ID_RADAR = 0x0121,
}Enum_Referee_Interaction_Command_ID;

/**
 * @brief 图形操作
 */
typedef enum
{
    Referee_Data_Interaction_Graphic_Operation_NULL = 0,
    Referee_Data_Interaction_Graphic_Operation_ADD,
    Referee_Data_Interaction_Graphic_Operation_CHANGE,
    Referee_Data_Interaction_Graphic_Operation_DELETE,
} Enum_Referee_Data_Interaction_Graphic_Operation;

/**
 * @brief 图形类型
 */
typedef enum
{
    Referee_Data_Interaction_Graphic_Type_LINE = 0,
    Referee_Data_Interaction_Graphic_Type_RECTANGLE,
    Referee_Data_Interaction_Graphic_Type_CIRCLE,
    Referee_Data_Interaction_Graphic_Type_OVAL,
    Referee_Data_Interaction_Graphic_Type_ARC,
    Referee_Data_Interaction_Graphic_Type_FLOAT,
    Referee_Data_Interaction_Graphic_Type_INTEGER,
    Referee_Data_Interaction_Graphic_Type_STRING,
} Enum_Referee_Data_Interaction_Graphic_Type;

/**
 * @brief 图形颜色
 */
typedef enum
{
    Referee_Data_Interaction_Graphic_Color_MAIN = 0,
    Referee_Data_Interaction_Graphic_Color_YELLOW,
    Referee_Data_Interaction_Graphic_Color_GREEN,
    Referee_Data_Interaction_Graphic_Color_ORANGE,
    Referee_Data_Interaction_Graphic_Color_PURPLE,
    Referee_Data_Interaction_Graphic_Color_PINK,
    Referee_Data_Interaction_Graphic_Color_CYAN,
    Referee_Data_Interaction_Graphic_Color_BLACK,
    Referee_Data_Interaction_Graphic_Color_WHITE,
} Enum_Referee_Data_Interaction_Graphic_Color;

/**
 * @brief 图形操作交互信息
 */
typedef enum
{
    Referee_Data_Interaction_Semiautomatic_Command_ATTACK = 1,
    Referee_Data_Interaction_Semiautomatic_Command_DEFENCE,
    Referee_Data_Interaction_Semiautomatic_Command_MOVE,
} Enum_Referee_Data_Interaction_Semiautomatic_Command;

// 命令码枚举定义
typedef enum
{
    Referee_Command_ID_GAME_STATUS = 0x0001,                 // 0x0001 比赛状态数据
    Referee_Command_ID_GAME_RESULT = 0x0002,                 // 0x0002 比赛结果数据
    Referee_Command_ID_GAME_ROBOT_HP = 0x0003,               // 0x0003 机器人血量数据
    Referee_Command_ID_EVENT_SELF_DATA = 0x0101,             // 0x0101 场地事件数据
    Referee_Command_ID_EVENT_SELF_REFEREE_WARNING = 0x0104,  // 0x0104 裁判警告数据
    Referee_Command_ID_EVENT_SELF_DART_STATUS = 0x0105,      // 2025新增 飞镖发射数据
    Referee_Command_ID_ROBOT_STATUS = 0x0201,                // 0x0201 机器人性能数据
    Referee_Command_ID_ROBOT_POWER_HEAT = 0x0202,            // 0x0202 底盘能量数据
    Referee_Command_ID_ROBOT_POSITION = 0x0203,              // 0x0203 机器人位置数据
    Referee_Command_ID_ROBOT_BUFF = 0x0204,                  // 0x0204 增益状态数据
    Referee_Command_ID_ROBOT_DAMAGE = 0x0206,                // 0x0206 伤害数据
    Referee_Command_ID_ROBOT_BOOSTER = 0x0207,               // 0x0207 实时射击数据
    Referee_Command_ID_ROBOT_REMAINING_AMMO = 0x0208,        // 0x0208 允许发弹量
    Referee_Command_ID_ROBOT_RFID = 0x0209,                  // 0x0209 RFID状态数据
    Referee_Command_ID_ROBOT_DART_COMMAND = 0x020A,          // 0x020A 飞镖选手端指令
    Referee_Command_ID_ROBOT_SENTRY_LOCATION = 0x020B,       // 0x020B 地面机器人位置
    Referee_Command_ID_ROBOT_RADAR_MARK = 0x020C,            // 0x020C 雷达标记进度
    Referee_Command_ID_ROBOT_SENTRY_DECISION = 0x020D,       // 0x020D 哨兵自主决策
    Referee_Command_ID_ROBOT_RADAR_DECISION = 0x020E,        // 0x020E 雷达自主决策
    Referee_Command_ID_INTERACTION = 0x0301,                 // 0x0301 机器人交互数据
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CUSTOM_CONTROLLER = 0x0302,             // 0x0302 自定义控制器交互
    Referee_Command_ID_INTERACTION_ROBOT_RECEIVE_CLIENT_MINIMAP = 0x0303,                // 0x0303 小地图交互
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_RADAR = 0x0305,                        // 0x0305 雷达数据接收
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_CUSTOM_CONTROLLER = 0x0306,            // 0x0306 自定义控制器接收
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_SENTRY_SEMIAUTOMATIC_MINIMAP = 0x0307, // 0x0307 哨兵半自动小地图
    Referee_Command_ID_INTERACTION_CLIENT_RECEIVE_ROBOT_MINIMAP = 0x0308,                // 0x0308 机器人小地图
} Enum_Referee_Command_ID;

/**
 * @brief 通用双方机器人ID
 *
 */
typedef enum
{
    Referee_Data_Robots_ID_NO = 0,
    Referee_Data_Robots_ID_RED_HERO_1,
    Referee_Data_Robots_ID_RED_ENGINEER_2,
    Referee_Data_Robots_ID_RED_INFANTRY_3,
    Referee_Data_Robots_ID_RED_INFANTRY_4,
    Referee_Data_Robots_ID_RED_INFANTRY_5,
    Referee_Data_Robots_ID_RED_AERIAL_6,
    Referee_Data_Robots_ID_RED_SENTRY_7,
    Referee_Data_Robots_ID_RED_DART_8,
    Referee_Data_Robots_ID_RED_RADAR_9,
    Referee_Data_Robots_ID_RED_BASE_10,
    Referee_Data_Robots_ID_RED_OUTPOST_11,
    Referee_Data_Robots_ID_BLUE_HERO_1 = 101,
    Referee_Data_Robots_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_ID_BLUE_AERIAL_6,
    Referee_Data_Robots_ID_BLUE_SENTRY_7,
    Referee_Data_Robots_ID_BLUE_DART_8,
    Referee_Data_Robots_ID_BLUE_RADAR_9,
    Referee_Data_Robots_ID_BLUE_BASE_10,
    Referee_Data_Robots_ID_BLUE_OUTPOST_11,
}Enum_Referee_Data_Robots_ID;
/**
 * @brief 通用双方选手端ID
 *
 */
typedef enum
{
    Referee_Data_Robots_Client_ID_NO = 0,
    Referee_Data_Robots_Client_ID_RED_HERO_1 = 0x0101,
    Referee_Data_Robots_Client_ID_RED_ENGINEER_2,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_3,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_4,
    Referee_Data_Robots_Client_ID_RED_INFANTRY_5,
    Referee_Data_Robots_Client_ID_RED_AERIAL_6,
    Referee_Data_Robots_Client_ID_BLUE_HERO_1 = 0x0165,
    Referee_Data_Robots_Client_ID_BLUE_ENGINEER_2,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_3,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_4,
    Referee_Data_Robots_Client_ID_BLUE_INFANTRY_5,
    Referee_Data_Robots_Client_ID_BLUE_AERIAL_6,
    Referee_Data_Robots_Server = 0x8080,
}Enum_Referee_Data_Robots_Client_ID;



/* 结构体定义 ---------------------------------------------------------------*/
//裁判系统串口接受基础结构体
typedef struct __attribute__((packed))
{
    uint8_t Frame_Header;
    uint16_t Data_Length;
    uint8_t Sequence;
    uint8_t CRC_8;
    Enum_Referee_Command_ID Referee_Command_ID;
    uint8_t Data[121];
}Struct_Referee_UART_Data;

/**
 * @brief 裁判系统经过处理的数据, 0x0001比赛状态, 3Hz发送
 */
typedef struct
{
    uint8_t game_type : 4;         // 比赛类型（位0-3）
    // 1: 超级对抗赛 2: 高校单项赛 3: AI挑战赛 4: 3V3对抗 5: 步兵对抗
    uint8_t game_progress : 4;     // 比赛阶段（位4-7）
    // 0: 未开始 1: 准备 2: 15秒自检 3: 5秒倒计时 4: 比赛 5: 结算
    uint16_t stage_remain_time;    // 当前阶段剩余时间（秒）
    uint64_t sync_timestamp;       // NTP同步时间戳（UNIX时间）

} __attribute__((packed)) Struct_Referee_Rx_Data_Game_Status;

/**
 * @brief 裁判系统经过处理的数据, 0x0002比赛结果, 比赛结束后发送
 */
typedef struct
{
    uint8_t winner;                // 胜利方标识
    // 0: 平局 1: 红方胜 2: 蓝方胜

} __attribute__((packed)) Struct_Referee_Rx_Data_Game_Result;

/**
 * @brief 裁判系统经过处理的数据, 0x0003机器人血量, 1Hz
 */
typedef struct
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;

} __attribute__((packed)) Struct_Referee_Rx_Data_Game_Robot_HP;

/**
 * @brief 裁判系统经过处理的数据, 0x0101场地事件, 1Hz发送
 */
typedef struct
{
    uint32_t event_data;           // 事件状态位
    // bit0-2: 补给区占领状态（0:未占领 1:已占领）
    // bit3-5: 能量机关激活状态（0:未激活 1:已激活）
    // bit6-7: 中央高地占领状态（1:己方 2:敌方）
    // bit21-22: 中心增益点占领状态（0:未占领 1:己方 2:敌方 3:双方）
} __attribute__((packed)) Struct_Referee_Rx_Data_Event_Self_Data;

/**
 * @brief 裁判系统经过处理的数据, 0x0104裁判警告信息, 判罚发生后发送
 */
typedef struct
{
    uint8_t level;                 // 判罚等级（1:黄牌 2:红牌 3:判负）
    uint8_t offending_robot_id;    // 违规机器人ID
    uint8_t count;                 // 违规次数统计
} __attribute__((packed)) Struct_Referee_Rx_Data_Event_Referee_Warning;

/**
 * @brief 裁判系统经过处理的数据, 0x0105飞镖15s倒计时, 1Hz发送
 */
typedef struct
{
    uint8_t dart_remaining_time;   // 剩余发射时间（秒）
    uint16_t dart_info;            // 发射状态信息
    // bit0-2: 最近击中目标（0:未命中 1:前哨站 2:基地固定目标...）
    // bit6-7: 当前选定目标（0:前哨站 1:基地固定目标...）
} __attribute__((packed)) Struct_Referee_Rx_Data_Event_Dart_Status;

/**
 * @brief 裁判系统经过处理的数据, 0x0201机器人状态, 10Hz发送
 */
typedef struct
{
    uint8_t robot_id;              // 机器人唯一ID
    uint8_t robot_level;           // 机器人等级（1-3级）
    uint16_t current_HP;           // 当前血量
    uint16_t maximum_HP;           // 最大血量
    uint16_t shooter_cooling_value;// 射击冷却值
    uint16_t shooter_heat_limit;   // 射击热量上限
    uint16_t chassis_power_limit;  // 底盘功率上限
    uint8_t gimbal_output : 1;     // 云台供电状态（0:关闭 1:24V）
    uint8_t chassis_output : 1;    // 底盘供电状态
    uint8_t shooter_output : 1;    // 发射机构供电状态
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Status;

/**
 * @brief 裁判系统经过处理的数据, 0x0202当前机器人实时功率热量, 50Hz发送
 */
typedef struct
{
    uint16_t reserved;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;        // 底盘缓冲能量（焦耳）
    uint16_t shooter_17mm_heat;    // 17mm发射机构热量
    uint16_t shooter_42mm_heat;    // 42mm发射机构热量
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Power_Heat;

/**
 * @brief 裁判系统经过处理的数据, 0x0203当前机器人实时位置, 10Hz发送
 */
typedef struct
{
    float x;                     // X坐标（米）
    float y;                     // Y坐标（米）
    float angle;                 // 朝向角度（正北0度）
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Position;

/**
 * @brief 裁判系统经过处理的数据, 0x0204当前机器人增益, 1Hz发送
 */
typedef struct
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t remaining_energy;
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Buff;

/**
 * @brief 裁判系统经过处理的数据, 0x0206伤害情况, 伤害发生后发送
 */
typedef struct
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;

} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Damage;

/**
 * @brief 裁判系统经过处理的数据, 0x0207子弹信息, 射击发生后发送
 */
typedef struct
{
    uint8_t bullet_type;           // 弹丸类型（1:17mm 2:42mm）
    uint8_t shooter_id;            // 发射机构ID（1-3）
    uint8_t fire_rate;             // 射击频率（Hz）
    float initial_speed;           // 初始速度（m/s）
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Booster;

/**
 * @brief 裁判系统经过处理的数据, 0x0208子弹剩余信息, 10Hz发送
 */
typedef struct
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Remaining_Ammo;

/**
 * @brief 裁判系统经过处理的数据, 0x0209RFID状态信息, 1Hz发送
 */
typedef struct
{
    uint32_t rfid_status;
    uint8_t rfid_status_2;
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_RFID;

/**
 * @brief 裁判系统经过处理的数据, 0x020a飞镖状态, 10Hz发送
 */
typedef struct
{
    uint8_t launch_status;         // 发射状态（0:关闭 1:开启中 2:已开启）
    uint8_t reserved;
    uint16_t target_change_time;   // 目标切换剩余时间（秒）
    uint16_t latest_launch_time;   // 最近发射指令时间（秒）
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Dart_Command;

/**
 * @brief 裁判系统经过处理的数据, 0x020b哨兵获取己方位置信息, 1Hz发送
 */
typedef struct
{
    float hero_x;                // 英雄机器人X坐标
    float hero_y;                // 英雄机器人Y坐标
    float engineer_x;            // 工程机器人X坐标
    float engineer_y;            // 工程机器人Y坐标
    float standard_3_x;          // 3号步兵机器人X坐标
    float standard_3_y;          // 3号步兵机器人Y坐标
    float standard_4_x;          // 4号步兵机器人X坐标
    float standard_4_y;          // 4号步兵机器人Y坐标
    float reserved;           // 保留字段
    float reserved_2;
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Sentry_Location;

/**
 * @brief 裁判系统经过处理的数据, 0x020c雷达标记进度, 1Hz发送
 */
typedef struct
{
    uint16_t mark_progress;

} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Radar_Mark;

/**
 * @brief 裁判系统经过处理的数据, 0x020d哨兵决策信息, 1Hz发送
 */
typedef struct
{
    uint32_t sentry_info;          // 自主决策信息
    // bit0-1: 双倍易伤机会（0-2）
    // bit2: 是否触发双倍易伤
    uint16_t sentry_info_2;        // 扩展信息
    // bit0: 脱战状态
    // bit1-11: 17mm弹药剩余兑换数
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Sentry_Decision;

/**
 * @brief 裁判系统经过处理的数据, 0x020e雷达决策信息, 1Hz发送
 */
typedef struct
{
    uint8_t radar_info;            // 雷达决策信息
    // bit0-1: 双倍易伤机会（0-2）
    // bit2: 是否触发双倍易伤
} __attribute__((packed)) Struct_Referee_Rx_Data_Robot_Radar_Decision;

//裁判系统接收数据总结构体
typedef struct
{
    Struct_Referee_Rx_Data_Game_Status Game_Status;
    Struct_Referee_Rx_Data_Game_Result Game_Result;
    Struct_Referee_Rx_Data_Game_Robot_HP Game_Robot_HP;
    Struct_Referee_Rx_Data_Event_Self_Data Event_Self_Data;
    Struct_Referee_Rx_Data_Event_Referee_Warning Event_Referee_Warning;
    Struct_Referee_Rx_Data_Event_Dart_Status Event_Dart_Status;
    Struct_Referee_Rx_Data_Robot_Status Robot_Status;
    Struct_Referee_Rx_Data_Robot_Power_Heat Robot_Power_Heat;
    Struct_Referee_Rx_Data_Robot_Position Robot_Position;
    Struct_Referee_Rx_Data_Robot_Buff Robot_Buff;
    Struct_Referee_Rx_Data_Robot_Damage Robot_Damage;
    Struct_Referee_Rx_Data_Robot_Booster Robot_Booster;
    Struct_Referee_Rx_Data_Robot_Remaining_Ammo Robot_Remaining_Ammo;
    Struct_Referee_Rx_Data_Robot_RFID Robot_RFID;
    Struct_Referee_Rx_Data_Robot_Dart_Command Robot_Dart_Command;
    Struct_Referee_Rx_Data_Robot_Sentry_Location Robot_Sentry_Location;
    Struct_Referee_Rx_Data_Robot_Radar_Mark Robot_Radar_Mark;
    Struct_Referee_Rx_Data_Robot_Sentry_Decision Robot_Sentry_Decision;
    Struct_Referee_Rx_Data_Robot_Radar_Decision Robot_Radar_Decision;

} __attribute__((packed)) Struct_Referee_Rx_Data;


/* 发送数据结构体 ------------------------------------------------------------*/
/**
 * @brief 图形配置结构体
 */
typedef struct
{
    uint8_t Index[3];
    uint32_t Operation_Enum : 3;
    uint32_t Type_Enum : 3;
    uint32_t Layer_Num : 4;
    uint32_t Color_Enum : 4;
    uint32_t Details_A : 9;
    uint32_t Details_B : 9;
    uint32_t Line_Width : 10;
    uint32_t Start_X : 11;
    uint32_t Start_Y : 11;
    uint32_t Details_C : 10;
    uint32_t Details_D : 11;
    uint32_t Details_E : 11;
} __attribute__((packed)) Struct_Referee_Data_Interaction_Graphic_Config;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Enum_Referee_Data_Interaction_Layer_Delete_Operation Operation;
    uint8_t Delete_Serial;
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Layer_Delete;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Struct_Referee_Data_Interaction_Graphic_Config Graphic[1];
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Graphic_1;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Struct_Referee_Data_Interaction_Graphic_Config Graphic[2];
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Graphic_2;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Struct_Referee_Data_Interaction_Graphic_Config Graphic[5];
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Graphic_5;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Struct_Referee_Data_Interaction_Graphic_Config Graphic[7];
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Graphic_7;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Robots_ID Sender;
    uint8_t Reserved;
    Enum_Referee_Data_Robots_Client_ID Receiver;
    Struct_Referee_Data_Interaction_Graphic_Config Graphic_String;
    uint8_t String[30];
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Graphic_String;

typedef struct
{
    uint16_t Header;
    uint32_t Confirm_Respawn_Status_Enum : 1;
    uint32_t Confirm_Exchange_Respawn_Status_Enum : 1;
    uint32_t Request_Exchange_Ammo_Number : 11;
    uint32_t Request_Exchange_Ammo_Time : 4;
    uint32_t Request_Exchange_HP_Time : 4;
    uint32_t Reserved : 11;
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Sentry;

typedef struct
{
    uint16_t Header;
    Enum_Referee_Data_Status Request_Double_Damage;
    uint16_t CRC_16;
} __attribute__((packed)) Struct_Referee_Tx_Data_Interaction_Radar;

/* 其他交互数据结构体略，根据需要可添加，这里仅包含发送UI所需的 */

/* 裁判系统主结构体 ---------------------------------------------------------*/
typedef struct
{
    // 绑定的UART
    Struct_UART_Manage_Object *UART_Manage_Object;
    // 数据包头标
    uint8_t Frame_Header;

    // 内部变量
    uint32_t Flag;
    uint32_t Pre_Flag;
    uint8_t  Sequence;

    // UI是否是初次绘制
    uint8_t UI_Change_Flag[10][10];

    // 裁判系统状态
    Enum_Referee_Status Referee_Status;

    // 接收数据缓冲区
    Struct_Referee_Rx_Data Referee_Rx_Data;

    // 发送图形配置缓存
    Struct_Referee_Data_Interaction_Graphic_Config Graphic_Config[10][10];

    // 读写变量
    Enum_Referee_Data_Status Referee_Trust_Status;
} Class_Referee;

/* Exported function declarations --------------------------------------------*/
/* 获取裁判系统状态 */


/* 图形配置生成函数 */
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Clear(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Line(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Rectangle(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t End_X, uint32_t End_Y);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Circle(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Radius);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Oval(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Length_X, uint32_t Length_Y);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Arc(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Center_X, uint32_t Center_Y, uint32_t Angle_Start, uint32_t Angle_End, uint32_t Length_X, uint32_t Length_Y);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Float(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, float Float);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_Integer(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, int32_t Integer);
Struct_Referee_Data_Interaction_Graphic_Config *Class_Referee_Set_Referee_UI_String(Class_Referee *self, uint8_t Layer_Num, uint8_t Graphic_Num, Enum_Referee_Data_Interaction_Graphic_Color Color, uint32_t Line_Width, uint32_t Start_X, uint32_t Start_Y, uint32_t Font_Width, uint32_t String_Length);

/* 发送函数 */
void Class_Referee_UART_Send_Interaction_UI_Layer_Delete(Class_Referee *self, Enum_Referee_Data_Interaction_Layer_Delete_Operation Layer_Delete_Operation, uint8_t Layer);
void Class_Referee_UART_Send_Interaction_UI_Graphic_1(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1);
void Class_Referee_UART_Send_Interaction_UI_Graphic_2(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2);
void Class_Referee_UART_Send_Interaction_UI_Graphic_5(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_3, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_4, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_5);
void Class_Referee_UART_Send_Interaction_UI_Graphic_7(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_1, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_2, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_3, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_4, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_5, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_6, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_7);
void Class_Referee_UART_Send_Interaction_UI_Graphic_String(Class_Referee *self, Struct_Referee_Data_Interaction_Graphic_Config *Graphic_String, const char *String_Content);

/* 回调函数 */
void Class_Referee_UART_RxCpltCallback(Class_Referee *self,uint8_t *Rx_Data, uint16_t Length);

void Class_Referee_TIM_1000ms_Alive_PeriodElapsedCallback(Class_Referee *self);

void Class_Referee_Init(Class_Referee *self, UART_HandleTypeDef *huart, uint8_t __Frame_Header);

extern Class_Referee class_referee;

#endif //CTRLBOARD_H7_IMU_DVC_REFEREE_H