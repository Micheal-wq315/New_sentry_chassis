#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
static const float LQR_K[2] = {0.045f, 0.001f}; // 角度增益K1、角速度增益K2
 
#include "math.h"
#include "stdio.h"

static const float LQR_KI = 0.0025f;          // 积分增益
static const float CTRL_DT = 0.002f;          // 任务周期 2ms
static const float INTEGRAL_MAX = 100.0f;     // 积分限幅
static const float THETA_DEAD_BAND = 0.05f;   // 角度误差死区(度)
static const float OMEGA_DEAD_BAND = 0.05f;   // 角速度死区(度/s)
static const float OUTPUT_DEAD_BAND = 0.06f;  // 输出死区
static const float OUTPUT_RAMP_STEP = 0.35f;  // 每周期最大输出变化
static const float REF_RAMP_STEP = 0.60f;     // 每周期参考值最大变化
static const float OMEGA_LPF_ALPHA = 0.85f;   // 角速度低通滤波系数
static const float STATIC_FRICTION_COMP = 0.08f; // 静摩擦补偿
static const float INTEGRAL_SEP_THRESH = 8.0f;   // 积分分离阈值(大误差不积分)

static float dm_integral[DM_MOTOR_CNT] = {0};
static float dm_last_theta_err[DM_MOTOR_CNT] = {0};
static float dm_last_output[DM_MOTOR_CNT] = {0};
static float dm_filt_omega[DM_MOTOR_CNT] = {0};
static float dm_filt_ref[DM_MOTOR_CNT] = {0};

static float clampf(float x, float min_val, float max_val)
{
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

static float apply_deadband(float x, float deadband)
{
    if (fabsf(x) < deadband)
        return 0.0f;
    return x;
}

static float ramp_limit(float target, float current, float step)
{
    float diff = target - current;
    if (diff > step) diff = step;
    if (diff < -step) diff = -step;
    return current + diff;
}

static float signf_fast(float x)
{
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

static uint8_t DMMotorGetIndex(DMMotorInstance *motor)
{
    for (uint8_t i = 0; i < idx; i++)
    {
        if (dm_motor_instance[i] == motor)
            return i;
    }
    return 0;
}

/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
    memset(motor->motor_can_instace->tx_buff, 0, 8);    //清空
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}
void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
    DWT_Delay(0.1);
}
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));
    
    motor->motor_settings = config->controller_setting_init_config;
    PIDInit(&motor->current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&motor->speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->angle_PID, &config->controller_param_init_config.angle_PID);
    motor->other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;

    motor->speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;
    motor->current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;

    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
    DWT_Delay(0.1);
    DMMotorCaliEncoder(motor);
    DWT_Delay(0.1);
    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorStop(DMMotorInstance *motor)//不使用使能模式是因为需要收到反馈
{
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}

void DMMotorChangeFeed(DMMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if(loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    if(loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
}

//使用lqr控制器替换原有pid控制器,保持接口不变
void DMMotorTask(void const *argument)
{
    float pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    static uint8_t *vbuf;

    uint8_t motor_idx = DMMotorGetIndex(motor);

    while (1)
    {
        pid_ref = motor->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        // 1. 参考值斜坡限制，防止设定值突变
        dm_filt_ref[motor_idx] = ramp_limit(pid_ref, dm_filt_ref[motor_idx], REF_RAMP_STEP);

        // 2. 获取角度/角速度反馈
        float theta = 0.0f;
        float omega = 0.0f;

        if (motor->other_angle_feedback_ptr != NULL)
        {
            theta = *motor->other_angle_feedback_ptr;
            motor->measure.other_angle = theta;
        }

        if (motor->other_speed_feedback_ptr != NULL)
        {
            omega = *motor->other_speed_feedback_ptr * 57.2958f; // rad/s -> deg/s
        }

        // 3. 角速度低通滤波
        dm_filt_omega[motor_idx] =
            OMEGA_LPF_ALPHA * dm_filt_omega[motor_idx] +
            (1.0f - OMEGA_LPF_ALPHA) * omega;
        omega = dm_filt_omega[motor_idx];

        // 4. 误差计算       
        float theta_err = dm_filt_ref[motor_idx] - theta;
        float omega_err = 0.0f - omega;

        // 5. 误差死区
        theta_err = apply_deadband(theta_err, THETA_DEAD_BAND);
        omega_err = apply_deadband(omega_err, OMEGA_DEAD_BAND);

        // 6. 梯形积分 + 积分分离 + 积分限幅
        float integral_candidate = dm_integral[motor_idx];

        if (fabsf(theta_err) < INTEGRAL_SEP_THRESH)
        {
            integral_candidate += 0.5f * (theta_err + dm_last_theta_err[motor_idx]) * CTRL_DT;
            integral_candidate = clampf(integral_candidate, -INTEGRAL_MAX, INTEGRAL_MAX);
        }

        // 7. LQR + I补偿
        float raw_output = LQR_K[0] * theta_err
                         + LQR_K[1] * omega_err
                         + LQR_KI * integral_candidate;

        // 8. 抗积分饱和
        // 若输出已饱和且误差继续推动饱和，则冻结积分
        if (!((raw_output > DM_PID_MAX && theta_err > 0.0f) ||
              (raw_output < -DM_PID_MAX && theta_err < 0.0f)))
        {
            dm_integral[motor_idx] = integral_candidate;
        }

        // 9. 输出死区 + 静摩擦补偿
        if (fabsf(raw_output) < OUTPUT_DEAD_BAND)
        {
            raw_output = 0.0f;
        }
        else
        {
            raw_output += signf_fast(raw_output) * STATIC_FRICTION_COMP;
        }
        // 10. 输出限幅
        raw_output = clampf(raw_output, -DM_PID_MAX, DM_PID_MAX);

        // 11. 输出斜坡限制
        set = ramp_limit(raw_output, dm_last_output[motor_idx], OUTPUT_RAMP_STEP);
        dm_last_output[motor_idx] = set;

        // 12. 停止模式处理
        if (motor->stop_flag == MOTOR_STOP)
        {
            set = 0.0f;
            dm_integral[motor_idx] = 0.0f;
            dm_last_output[motor_idx] = 0.0f;
        }

        dm_last_theta_err[motor_idx] = theta_err;

        // 保持原有方向逻辑
        set = -set;

        vbuf = (uint8_t *)&set;
        motor->motor_can_instace->tx_buff[0] = *vbuf;
        motor->motor_can_instace->tx_buff[1] = *(vbuf + 1);
        motor->motor_can_instace->tx_buff[2] = *(vbuf + 2);
        motor->motor_can_instace->tx_buff[3] = *(vbuf + 3);
        CANTransmit(motor->motor_can_instace, 1);

        osDelay(2);
    }
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
// void DMMotorTask(void const *argument)
// {
//     float  pid_measure, pid_ref, set;
//     DMMotorInstance *motor = (DMMotorInstance *)argument;
//    //DM_Motor_Measure_s *measure = &motor->measure;
//     Motor_Control_Setting_s *setting = &motor->motor_settings;
//     //CANInstance *motor_can = motor->motor_can_instace;
//     //uint16_t tmp;
//     DMMotor_Send_s motor_send_mailbox;
//     while (1)
//     {   
//         pid_ref = motor->pid_ref;

//         if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
//             pid_ref *= -1;
       
//         LIMIT_MIN_MAX(set, DM_V_MIN, DM_V_MAX);
//         // motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
//         // // motor_send_mailbox.velocity_des = float_to_uint(pid_ref, DM_V_MIN, DM_V_MAX, 16);
//         // motor_send_mailbox.velocity_des = 0;
//         // motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);
//         // motor_send_mailbox.Kp = 0;
//         // motor_send_mailbox.Kd = 0;

//             // motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 16);

//         /* MIT模式控制帧 */
//         // motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
//         // motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
//         // motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
//         // motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
//         // motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
//         // motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
//         // motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
//         // motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

//         /*位置环计算*/
//         if((setting->close_loop_type & ANGLE_LOOP) && (setting->outer_loop_type & ANGLE_LOOP))
//         {
//             if(setting->angle_feedback_source == OTHER_FEED)
//             {
//                 pid_measure = *motor->other_angle_feedback_ptr;
//                 motor->measure.other_angle = pid_measure;
//             }
//             else
//                 pid_measure = motor->measure.total_angle;

//             pid_ref = PIDCalculate(&motor->angle_PID, pid_measure, pid_ref);
            
//         }

//     // 速度前馈（启用且指针有效时）
//     if ((setting->feedforward_flag & SPEED_FEEDFORWARD) && (motor->speed_feedforward_ptr != NULL)) {
//         float speed_feed = *motor->speed_feedforward_ptr;
    
//         // 电机反转时前馈同步反转
//         if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE) {
//             speed_feed *= -1;
//          }

//         // 前馈滤波
//         static float speed_feed_filtered = 0.0f;
//         speed_feed_filtered = 0.95f * speed_feed_filtered + 0.05f * speed_feed; // 一阶低通滤波，可调整系数
    
//          // 叠加滤波后的前馈
//         pid_ref -= speed_feed_filtered; 
//     }

//         set = pid_ref*(-1); // 方向反转
//         if(abs(set) > DM_PID_MAX)
//             set = DM_PID_MAX * (set/abs(set));
        
//         if(motor->stop_flag == MOTOR_STOP)
//             set = 0;
 
//         /* 速度模式控制帧 */
//         static uint8_t *vbuf;
//         vbuf = (uint8_t *)&set;

//         motor->motor_can_instace->tx_buff[0] = *vbuf;
//         motor->motor_can_instace->tx_buff[1] = *(vbuf + 1);
//         motor->motor_can_instace->tx_buff[2] = *(vbuf + 2);
//         motor->motor_can_instace->tx_buff[3] = *(vbuf + 3);

//         CANTransmit(motor->motor_can_instace, 1);

//         osDelay(2);
//     }
// }

void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}
