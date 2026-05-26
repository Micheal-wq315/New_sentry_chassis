# 半舵轮+半全向轮对角线构型底盘运动学解算

<p align='right'>HY-2025 SteeringSentry Team</p>

## 📋 目录

- [1. 构型概述](#1-构型概述)
- [2. 坐标系定义](#2-坐标系定义)
- [3. 运动学原理](#3-运动学原理)
- [4. 详细解算过程](#4-详细解算过程)
- [5. 代码实现](#5-代码实现)
- [6. 调试与优化](#6-调试与优化)
- [7. 常见问题](#7-常见问题)

---

## 1. 构型概述

### 1.1 底盘布局

本底盘采用**半舵轮 + 半全向轮对角线构型**，具体布局如下：

```
        前方 (X+)
        
    LF(舵轮) ────────── RF(全向轮竖直)
       │                      │
       │      底盘中心         │
       │                      │
    LB(全向轮竖直) ─────── RB(舵轮)
    
        后方 (X-)
```

**轮子配置：**
- **LF (Left-Front, 左前)**：舵轮 - 可360°转向 + 驱动
- **RF (Right-Front, 右前)**：全向轮 - **竖直方向（90°）**，仅沿Y轴滚动
- **LB (Left-Back, 左后)**：全向轮 - **竖直方向（90°）**，仅沿Y轴滚动
- **RB (Right-Back, 右后)**：舵轮 - 可360°转向 + 驱动

### 1.2 构型优势

| 特性 | 纯舵轮底盘 | 半舵轮+半全向轮 | 纯麦轮底盘 |
|------|-----------|----------------|-----------|
| 电机数量 | 8个 | **6个** | 4个 |
| 控制复杂度 | 高（4角度环） | **中（2角度环）** | 低（无角度环） |
| 机动性 | 最优 | **较好** | 好 |
| 成本 | 高 | **中等** | 低 |
| 调试难度 | 难 | **较易** | 易 |
| 功率效率 | 高 | **较高** | 中 |

**核心优势：**
- ✅ 减少2个舵机电机，降低成本和重量
- ✅ 保留全向移动能力，机动性优于纯麦轮
- ✅ 控制复杂度适中，易于调试和维护
- ✅ 对角线舵轮布局保证旋转稳定性

---

## 2. 坐标系定义

### 2.1 右手坐标系

底盘采用**右手坐标系**（符合RoboMaster标准）：

- **X轴**：前进方向为正（前方）
- **Y轴**：右侧为正（右方）
- **Z轴**：垂直向上为正（遵循右手定则）

```
        Y+ (右)
        ↑
        │
        │
   X+ ←─┼──→ (前)
   (前)  │
        │
        ↓
```

### 2.2 轮子位置坐标

以底盘几何中心为原点，各轮坐标为：

| 轮子 | X坐标 | Y坐标 | 说明 |
|------|-------|-------|------|
| LF | `-TRACK_WIDTH/2` | `+WHEEL_BASE/2` | 左前 |
| RF | `+TRACK_WIDTH/2` | `+WHEEL_BASE/2` | 右前 |
| LB | `-TRACK_WIDTH/2` | `-WHEEL_BASE/2` | 左后 |
| RB | `+TRACK_WIDTH/2` | `-WHEEL_BASE/2` | 右后 |

其中：
- `TRACK_WIDTH`：轮距（左右轮中心距离）
- `WHEEL_BASE`：轴距（前后轮中心距离）

### 2.3 到旋转中心的距离

所有轮到底盘中心的距离相同：

```
r = √[(TRACK_WIDTH/2)² + (WHEEL_BASE/2)²]
```

---

## 3. 运动学原理

### 3.1 运动分解

任何底盘运动都可以分解为三个自由度的组合：

1. **Vx**：沿X轴的平移速度（mm/s），向前为正
2. **Vy**：沿Y轴的平移速度（mm/s），向右为正
3. **Wz**：绕Z轴的旋转角速度（°/s），逆时针为正

### 3.2 速度合成原理

对于任意一个轮子，其速度由两部分组成：

```
V_wheel = V_translation + V_rotation
```

#### **平移分量**
所有轮子的平移速度相同，等于底盘的(Vx, Vy)。

#### **旋转分量**
旋转产生的速度与轮子位置有关：

```
V_rot_x = -Wz × y_i
V_rot_y = Wz × x_i
```

其中 `(x_i, y_i)` 是第 i 个轮子的坐标。

### 3.3 各轮速度矢量

综合平移和旋转，每个轮子的速度分量为：

```
Vx_i = Vx - Wz × y_i
Vy_i = Vy + Wz × x_i
```

---

## 4. 详细解算过程

### 4.1 计算各轮速度分量

首先将角速度从度/秒转换为弧度/秒：

```c
wz_rad = Wz × (π / 180)  // 单位：rad/s
```

定义中间变量：

```c
half_track = TRACK_WIDTH / 2.0f;
half_wheel_base = WHEEL_BASE / 2.0f;
```

#### **LF (左前舵轮)**

```
x_LF = -half_track
y_LF = +half_wheel_base

Vx_LF = Vx - wz_rad × half_wheel_base
Vy_LF = Vy + wz_rad × (-half_track) = Vy - wz_rad × half_track
```

#### **RF (右前全向轮)**

```
x_RF = +half_track
y_RF = +half_wheel_base

Vx_RF = Vx - wz_rad × half_wheel_base
Vy_RF = Vy + wz_rad × half_track
```

#### **LB (左后全向轮)**

```
x_LB = -half_track
y_LB = -half_wheel_base

Vx_LB = Vx - wz_rad × (-half_wheel_base) = Vx + wz_rad × half_wheel_base
Vy_LB = Vy + wz_rad × (-half_track) = Vy - wz_rad × half_track
```

#### **RB (右后舵轮)**

```
x_RB = +half_track
y_RB = -half_wheel_base

Vx_RB = Vx - wz_rad × (-half_wheel_base) = Vx + wz_rad × half_wheel_base
Vy_RB = Vy + wz_rad × half_track
```

### 4.2 舵轮解算（LF 和 RB）

舵轮可以自由转向，因此需要计算**速度大小**和**目标角度**。

#### **速度大小计算**

```
Speed_LF = √(Vx_LF² + Vy_LF²)
Speed_RB = √(Vx_RB² + Vy_RB²)
```

#### **目标角度计算**

使用 `atan2` 函数计算角度（返回弧度值，范围 [-π, π]）：

```
Angle_LF = atan2(Vy_LF, Vx_LF)
Angle_RB = atan2(Vy_RB, Vx_RB)
```

转换为角度制：

```
Angle_LF_deg = Angle_LF × (180 / π)
Angle_RB_deg = Angle_RB × (180 / π)
```

#### **角度翻转策略**

为了避免舵轮旋转过大（超过180°），采用**最短路径策略**：

**判断条件：**
```c
angle_diff = current_angle - last_angle

// 归一化角度差到 [-π, π]
while(angle_diff > π) angle_diff -= 2π;
while(angle_diff < -π) angle_diff += 2π;

if(|angle_diff| > π/2) {  // 角度差超过90°
    // 翻转180°
    new_angle = current_angle ± π;
    // 同时反转速度方向
    new_speed = -old_speed;
}
```

**物理意义：**
- 如果目标角度与当前角度相差超过90°，选择反向转动更短
- 例如：从 80° 转到 100°，直接转 20° 比转 340° 更快
- 但需要从 80° 转到 -80°（即 280°），翻转后变为从 80° 转到 100°（20°）

### 4.3 全向轮解算（RF 和 LB）

全向轮**竖直放置**，固定方向为 **90°**（沿Y轴正方向）。

#### **投影原理**

全向轮只能沿固定方向滚动，需要将速度矢量**投影**到该方向。

对于固定角度 θ 的全向轮：

```
Speed = Vx × cos(θ) + Vy × sin(θ)
```

#### **RF 全向轮（θ = 90°）**

```
cos(90°) = 0
sin(90°) = 1

Speed_RF = Vx_RF × 0 + Vy_RF × 1 = Vy_RF
```

**结论：RF轮的速度直接等于 Vy_RF！**

#### **LB 全向轮（θ = 90°）**

同样：

```
Speed_LB = Vy_LB
```

#### **全向轮角度**

全向轮方向固定，无需计算角度：

```
Angle_RF = 90.0°
Angle_LB = 90.0°
```

### 4.4 单位转换

将速度从 mm/s 转换为电机 RPM（转/分钟）。

#### **转换公式**

```
RPM = Speed(mm/s) / (2π × R_wheel) × 60 × Reduction_Ratio
```

其中：
- `R_wheel`：轮子半径（mm）
- `Reduction_Ratio`：减速比（M3508 为 19:1）

#### **简化计算**

预计算转换系数：

```c
rpm_ratio = 60.0 × REDUCTION_RATIO_WHEEL / (2π × RADIUS_WHEEL)
```

最终输出：

```c
RPM_LF = Speed_LF × rpm_ratio
RPM_RF = Speed_RF × rpm_ratio
RPM_LB = Speed_LB × rpm_ratio
RPM_RB = Speed_RB × rpm_ratio
```

---

## 5. 代码实现

### 5.1 核心解算函数

```c
/**
 * @brief 半舵轮+半全向轮对角线构型速度解算
 *        LF(舵轮), RF(全向轮90°), LB(全向轮90°), RB(舵轮)
 * 
 * @param chassis_handle 底盘控制句柄
 * @param chassis_vx     X轴速度 (mm/s)
 * @param chassis_vy     Y轴速度 (mm/s)
 * @param chassis_wz     旋转角速度 (°/s)
 */
static void Steer_Speed_Calcu(ChassisHandle_t *chassis_handle, 
                              float chassis_vx, float chassis_vy, float chassis_wz)
{
    // ========== 1. 预计算中间变量 ==========
    
    float half_track = TRACK_WIDTH / 2.0f;
    float half_wheel_base = WHEEL_BASE / 2.0f;
    float wz_rad = chassis_wz * DEGREE_2_RAD;  // 转换为 rad/s
    
    // ========== 2. 计算各轮速度分量 ==========
    
    // LF (舵轮)
    float vx_lf = chassis_vx - wz_rad * half_wheel_base;
    float vy_lf = chassis_vy - wz_rad * half_track;
    
    // RF (全向轮竖直)
    float vx_rf = chassis_vx - wz_rad * half_wheel_base;
    float vy_rf = chassis_vy + wz_rad * half_track;
    
    // LB (全向轮竖直)
    float vx_lb = chassis_vx + wz_rad * half_wheel_base;
    float vy_lb = chassis_vy - wz_rad * half_track;
    
    // RB (舵轮)
    float vx_rb = chassis_vx + wz_rad * half_wheel_base;
    float vy_rb = chassis_vy + wz_rad * half_track;
    
    // ========== 3. 舵轮解算 ==========
    
    // LF 舵轮
    float speed_lf = sqrtf(vx_lf*vx_lf + vy_lf*vy_lf);
    float angle_lf = atan2f(vy_lf, vx_lf);  // 弧度
    
    // RB 舵轮
    float speed_rb = sqrtf(vx_rb*vx_rb + vy_rb*vy_rb);
    float angle_rb = atan2f(vy_rb, vx_rb);  // 弧度
    
    // ========== 4. 全向轮解算（简化！）==========
    
    float speed_rf = vy_rf;  // 直接取Vy分量
    float speed_lb = vy_lb;  // 直接取Vy分量
    
    // ========== 5. 转换为RPM ==========
    
    float rpm_ratio = 60.0f * REDUCTION_RATIO_WHEEL / (2.0f * PI * RADIUS_WHEEL);
    
    chassis_handle->chassis_motor_speed[0] = speed_lf * rpm_ratio;  // LF
    chassis_handle->chassis_motor_speed[1] = speed_rf * rpm_ratio;  // RF
    chassis_handle->chassis_motor_speed[2] = speed_lb * rpm_ratio;  // LB
    chassis_handle->chassis_motor_speed[3] = speed_rb * rpm_ratio;  // RB
    
    // ========== 6. 舵轮角度处理与翻转策略 ==========
    
    // 处理 LF 舵轮（索引0）
    ProcessSteerAngle(chassis_handle, 0, angle_lf, speed_lf);
    
    // RF 全向轮（索引1）- 固定90°
    chassis_handle->motor_set_steer[1] = 90.0f;
    chassis_handle->TurnFlag[1] = 0;
    
    // LB 全向轮（索引2）- 固定90°
    chassis_handle->motor_set_steer[2] = 90.0f;
    chassis_handle->TurnFlag[2] = 0;
    
    // 处理 RB 舵轮（索引3）
    ProcessSteerAngle(chassis_handle, 3, angle_rb, speed_rb);
    
    // ========== 7. 复制到最终输出 ==========
    
    for(int i = 0; i < 4; i++)
    {
        chassis_handle->motor_set_speed[i] = chassis_handle->chassis_motor_speed[i];
    }
}

/**
 * @brief 处理舵轮角度翻转策略
 * 
 * @param chassis_handle 底盘控制句柄
 * @param index          轮子索引 (0或3)
 * @param target_angle   目标角度（弧度）
 * @param speed          速度大小
 */
static void ProcessSteerAngle(ChassisHandle_t *chassis_handle, int index, 
                              float target_angle, float speed)
{
    // 角度归一化到 [-π, π]
    while(target_angle > PI) target_angle -= 2*PI;
    while(target_angle < -PI) target_angle += 2*PI;
    
    // 转换为角度制
    float target_angle_deg = target_angle * RADIAN_TO_ANGLE;
    
    // 计算角度差
    float last_angle_rad = chassis_handle->last_steer_target_angle[index] * DEGREE_2_RAD;
    float angle_diff = target_angle - last_angle_rad;
    
    // 归一化角度差
    while(angle_diff > PI) angle_diff -= 2*PI;
    while(angle_diff < -PI) angle_diff += 2*PI;
    
    // 判断是否需要翻转
    if(fabsf(angle_diff) > PI/2)
    {
        // 翻转180°
        target_angle += (target_angle > 0) ? -PI : PI;
        
        // 归一化
        while(target_angle > PI) target_angle -= 2*PI;
        while(target_angle < -PI) target_angle += 2*PI;
        
        // 标记需要反转速度
        chassis_handle->TurnFlag[index] = 1;
        chassis_handle->chassis_motor_speed[index] = -speed;
    }
    else
    {
        chassis_handle->TurnFlag[index] = 0;
    }
    
    // 保存结果
    chassis_handle->motor_set_steer[index] = target_angle * RADIAN_TO_ANGLE;
    chassis_handle->last_steer_target_angle[index] = chassis_handle->motor_set_steer[index];
}
```

### 5.2 特殊工况处理

#### **静止状态处理**

当 Vx、Vy、Wz 都接近零时，保持上一状态以避免舵轮抖动：

```c
#define DEAD_ZONE 0.5f  // 死区阈值

if(fabsf(chassis_vx) < DEAD_ZONE && 
   fabsf(chassis_vy) < DEAD_ZONE && 
   fabsf(chassis_wz) < DEAD_ZONE)
{
    // 保持上一状态
    for(int i = 0; i < 4; i++)
    {
        chassis_handle->motor_set_speed[i] = 0;
        if(i == 0 || i == 3)  // 只处理舵轮
        {
            chassis_handle->motor_set_steer[i] = chassis_handle->last_steer_target_angle[i];
        }
    }
    return;
}
```

#### **最大功率限制**

根据裁判系统反馈动态限制输出功率：

```c
// 获取裁判系统功率限制
float power_limit = referee_data->GameRobotState.chassis_power_limit;
float current_power = referee_data->PowerHeatData.chassis_power;

// 计算限幅系数
float limit_coeff = 1.0f;
if(current_power > power_limit * 0.9f)
{
    limit_coeff = (power_limit - current_power) / (power_limit * 0.1f);
    limit_coeff = fmaxf(0.0f, fminf(1.0f, limit_coeff));
}

// 应用限幅
for(int i = 0; i < 4; i++)
{
    chassis_handle->motor_set_speed[i] *= limit_coeff;
}
```

---

## 6. 调试与优化

### 6.1 PID 参数整定

#### **舵轮角度环（GM6020）**

推荐初始参数：

```c
.angle_PID = {
    .Kp = 10.0f,      // 比例增益：8~15
    .Ki = 0.0f,       // 积分增益：通常为0
    .Kd = 0.0f,       // 微分增益：通常为0
    .MaxOut = 500.0f, // 最大输出
    .IntegralLimit = 100.0f,
    .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit
}
```

**调参步骤：**
1. 先将 Ki、Kd 设为 0
2. 逐步增大 Kp，直到出现轻微振荡
3. 回退 Kp 到稳定值的 70%~80%
4. 如有稳态误差，适当增加 Ki（通常不需要）

#### **舵轮速度环（M3508）**

推荐初始参数：

```c
.speed_PID = {
    .Kp = 3.5f,       // 比例增益：3~5
    .Ki = 1.0f,       // 积分增益：0.5~1.5
    .Kd = 0.0f,       // 微分增益：通常为0
    .MaxOut = 20000.0f,
    .IntegralLimit = 3000.0f,
    .Improve = PID_Integral_Limit | PID_DerivativeFilter
}
```

#### **全向轮速度环（M3508）**

与舵轮速度环相同。

### 6.2 安装校准

#### **舵轮零点校准**

1. 手动将所有舵轮转到机械零点（通常指向正前方）
2. 读取编码器值 `ecd_zero`
3. 在初始化时设置偏移量：

```c
chassis_handle.chassis_steer_motor[0].offset_ecd = ecd_zero_LF;
chassis_handle.chassis_steer_motor[3].offset_ecd = ecd_zero_RB;
```

#### **全向轮方向校准**

1. 确保 RF 和 LB 全向轮严格竖直安装（90°）
2. 测试方法：
   - 给定纯 Vy 运动（Vy=1000, Vx=0, Wz=0）
   - 观察 RF 和 LB 是否正常滚动
   - 如果有横向滑动，说明安装角度有偏差
3. 微调偏移量补偿安装误差

### 6.3 测试流程

#### **阶段1：单轮测试**

逐个测试每个电机：
- 检查电机旋转方向是否正确
- 验证编码器反馈是否正常
- 确认 CAN 通信无误

#### **阶段2：纯平移测试**

分别测试三个方向的纯平移：

```c
// 纯X方向
Vx = 1000, Vy = 0, Wz = 0

// 纯Y方向
Vx = 0, Vy = 1000, Wz = 0

// 纯旋转
Vx = 0, Vy = 0, Wz = 180
```

**预期现象：**
- X方向：所有轮子同速同向
- Y方向：左侧两轮与右侧两轮反向
- 旋转：对角线轮子同向，相邻轮子反向

#### **阶段3：复合运动测试**

测试斜向移动+旋转的组合：

```c
Vx = 500, Vy = 500, Wz = 90
```

观察底盘是否平滑运动，无明显抖动。

#### **阶段4：极限测试**

- 最大速度下的稳定性
- 快速启停响应
- 长时间运行温升

### 6.4 性能优化

#### **斜坡加减速**

避免突变导致的冲击：

```c
// 斜坡滤波器
#define ACCEL_LIMIT 5000.0f  // 最大加速度 mm/s²

float delta_v = target_v - current_v;
float max_delta = ACCEL_LIMIT * dt;

if(delta_v > max_delta) delta_v = max_delta;
if(delta_v < -max_delta) delta_v = -max_delta;

current_v += delta_v;
```

#### **前馈补偿**

加入摩擦力前馈提高响应：

```c
// 速度环前馈
float feedforward = sign(speed_ref) * FRICTION_COMPENSATION;
pid_output += feedforward;
```

---

## 7. 常见问题

### 7.1 舵轮抖动

**现象：** 舵轮在静止或小幅度运动时频繁摆动

**原因：**
- PID 参数过大
- 编码器噪声
- 机械间隙

**解决方案：**
1. 降低角度环 Kp
2. 增加死区处理
3. 检查机械安装是否牢固
4. 添加低通滤波

```c
// 低通滤波
#define FILTER_ALPHA 0.1f
angle_filtered = FILTER_ALPHA * angle_new + (1 - FILTER_ALPHA) * angle_filtered;
```

### 7.2 旋转不流畅

**现象：** 原地旋转时底盘抖动或漂移

**原因：**
- Wz 角速度单位转换错误
- IMU 数据不准确
- 各轮速度不对称

**解决方案：**
1. 检查 `DEGREE_2_RAD` 常量是否正确
2. 校准 IMU 零偏
3. 验证各轮速度计算是否对称

### 7.3 功率不均

**现象：** 某些方向运动时功率消耗异常大

**原因：**
- 对角线构型在某些方向需要更大扭矩
- 未做功率均衡

**解决方案：**
1. 实施动态功率限制
2. 监控各电机电流
3. 优化运动轨迹规划

### 7.4 漂移问题

**现象：** 底盘无法精确保持位置，缓慢漂移

**原因：**
- 速度环积分饱和
- 地面摩擦不均
- IMU 漂移

**解决方案：**
1. 限制积分项上限
2. 定期校准 IMU
3. 考虑加入视觉或里程计辅助定位

### 7.5 全向轮打滑

**现象：** 全向轮在横向受力时打滑

**原因：**
- 安装角度不准确
- 轮子磨损
- 地面摩擦系数低

**解决方案：**
1. 重新校准安装角度
2. 更换全向轮
3. 改善地面条件或使用防滑垫

---

## 附录 A：数学公式汇总

### A.1 基本公式

**轮子位置：**
```
x_i = ±TRACK_WIDTH/2
y_i = ±WHEEL_BASE/2
```

**速度分量：**
```
Vx_i = Vx - Wz × y_i
Vy_i = Vy + Wz × x_i
```

**舵轮速度：**
```
Speed = √(Vx² + Vy²)
Angle = atan2(Vy, Vx)
```

**全向轮速度（竖直方向）：**
```
Speed = Vy
```

**单位转换：**
```
RPM = Speed × 60 × Reduction_Ratio / (2π × R_wheel)
```

### A.2 常量定义

```c
#define PI              3.14159265358979f
#define DEGREE_2_RAD    (PI / 180.0f)
#define RAD_2_DEGREE    (180.0f / PI)
```

---

## 附录 B：参考文献

1. RoboMaster 官方技术文档
2. 《机器人学导论》- John J. Craig
3. 华南理工大学 RoboMaster 战队技术分享
4. CSDN: RoboMaster舵轮底盘代码分享

---

## 版本历史

| 版本 | 日期 | 作者 | 说明 |
|------|------|------|------|
| v1.0 | 2025-01 | HY Team | 初始版本，半舵轮+半全向轮对角线构型 |

---

**文档维护：** HY-2025 SteeringSentry 电控组  
**联系方式：** neozng1@hnu.edu.cn
