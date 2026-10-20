/**
  ******************************************************************************
  * @file           : INS.cpp
  * @author         : Chen Haoran
  * @brief          : None
  * @attention      : None
  * @date           : 2025/11/4
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "INS.h"
#include "task_and_callback.h"
/* Define --------------------------------------------------------------------*/

/* Enum ----------------------------------------------------------------------*/

/* Variable && Struct --------------------------------------------------------*/

/* Function Declaration ------------------------------------------------------*/

/* Function ------------------------------------------------------------------*/

// void class_INS::INS_Init()
// {
//     QuaternionEKF_Init(0.996, 0.001);
// }


// void class_INS::INS_Update()
// {
//     if (attitude_flag == 2) //ekf����̬����
//     {
//         gyro[0] -= gyro_correct[0]; //��ȥ������0Ʈ
//         gyro[1] -= gyro_correct[1];
//         gyro[2] -= gyro_correct[2];

// #if cheat              //���� ������yaw���ȶ� ȥ���Ƚ�С��ֵ
//         if (fabsf(gyro[2]) < 0.003f)
//             gyro[2] = 0;
// #endif

//         //ekf��̬���㲿��
//         Quaternion_EKF_Calculate(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0.001);

//         // ekf��ȡ��̬�ǶȺ���
//         pitch = Get_Roll();
//         roll = Get_Pitch();
//         yaw = Get_Yaw();
//         total_yaw = Get_Total_Yaw();

//         memcpy(accel, Get_Accel(), sizeof(accel));


//         //��������
//         if (fabsf(accel[0]) < 0.03f)
//         {
//             accel[0] = 0.0f; //x��
//         }
//         if (fabsf(accel[1]) < 0.03f)
//         {
//             accel[1] = 0.0f; //y��
//         }
//         if (fabsf(accel[2]) < 0.04f)
//         {
//             accel[2] = 0.0f; //z��
//         }

//         ins_flag = 1;
//     }
//     else if (attitude_flag == 1) //״̬1 ��ʼ1000�ε�������0Ʈ��ʼ��
//     {
//         //gyro correct
//         gyro_correct[0] += gyro[0];
//         gyro_correct[1] += gyro[1];
//         gyro_correct[2] += gyro[2];
//         correct_times++;
//         if (correct_times >= correct_Time_define)
//         {
//             gyro_correct[0] /= correct_Time_define;
//             gyro_correct[1] /= correct_Time_define;
//             gyro_correct[2] /= correct_Time_define;
//             attitude_flag = 2; //go to 2 state
//         }
//     }
// }




const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;

// Fusion_AHRS
TacticalSystem sys;

void class_INS::INS_Init()
{

    QuaternionEKF_Init(0.996, 0.001);
    // ������������ (Hz), ������ͨ��ֹƵ�� (Hz)
    // ������ֹƵ�ʣ����� 0.05~0.1Hz������ 0.5~1.0Hz������Ҫ������ 0
    Tactical_Init(&sys, 1000.0f, 0.8f);
}

void class_INS::INS_Update()
{
    static uint32_t count = 0;
    const float gravity[3] = {0, 0, 9.81f};

    // ins update
    if ((count % 1) == 0)
    {
        // ���ĺ���,EKF������Ԫ��
        Quaternion_EKF_Calculate(gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], 0.001);

        memcpy(q, Get_q(), sizeof(q));

        // ����ϵ������ת������������ϵ������ѡȡ����ϵΪ����ϵ
        BodyFrameToEarthFrame(xb, xn, q);
        BodyFrameToEarthFrame(yb, yn, q);
        BodyFrameToEarthFrame(zb, zn, q);

        // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
        // float gravity_b[3];
        // EarthFrameToBodyFrame(gravity, gravity_b, q);
        // for (uint8_t i = 0; i < 3; i++) // ͬ����һ����ͨ�˲�
        // {
        //     MotionAccel_b[i] = (Accel[i] - gravity_b[i]) * dt / (AccelLPF + dt) + MotionAccel_b[i] * AccelLPF / (AccelLPF + dt);
        // }
        // BodyFrameToEarthFrame(MotionAccel_b, MotionAccel_n, q); // ת���ص���ϵn

        // ��ȡ��������
        // pitch = Get_Roll();
        // roll = Get_Pitch();
        // yaw = Get_Yaw();

        // pitch = -get_mahony_pitch();
        // yaw = get_mahony_yaw();
    }



    if ((count % 1000) == 0)
    {
        // 200hz
    }

    count++;





    // FusionVector gyro_calc = {.axis = {(float)(gyro[0] * 180.0/M_PI), (float)(gyro[1] * 180.0/M_PI), (float)(gyro[2] * 180.0/M_PI)}};   // ��λ����/s
    // FusionVector acc_calc  = {.axis = {accel[0], accel[1], accel[2]}};    // ��λ��g (1g = 9.8m/s?)

    // // ���� EKF
    // Tactical_Update(&sys, gyro_calc, acc_calc);

    // // ��ȡ��̬ (��Ԫ�� �� ŷ����)
    // FusionEuler euler = FusionQuaternionToEuler(sys.quaternion);
    // pitch  = euler.angle.roll * M_PI/180.0f;   // ����� (��)
    // roll = euler.angle.pitch * M_PI/180.0f;  // ������ (��)
    // yaw   = euler.angle.yaw * M_PI/180.0f;    // ƫ���� (��) �� 6���޾��Ժ��򣬻�Ư��

    // // ��ȡ������Ϣ
    // float heavePos = sys.heavePosition;  // ����λ�� (m)
    // float heaveVel = sys.heaveVelocity;  // �����ٶ� (m/s)

}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void class_INS::BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void class_INS::EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


