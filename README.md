# New_sentry_chassis

> 参考跃鹿战队 RoboMaster 哨兵机器人运动控制代码仓库  
> **战队**：南昌航空大学洪鹰战队
> **赛季**：2025 – 2026  
> **硬件平台**：STM32H7 系列 + STM32f407系列 + FreeRTOS  
> **通信**：CAN / USB / 航模遥控器 (DT7)

---

## 📁 项目概况

本仓库包含两代哨兵底盘的核心嵌入式软件：

| 子项目 | 说明 | 适用赛季 |
|--------|------|----------|
| `hy_sentry-main` | 基于舵轮底盘的独立控制方案，具备航模遥控手动操控功能 | 2026 |
| `hy_sentry_-gimbal-main` | 底盘‑云台一体化方案，运行于达妙 MC02 (STM32H723)，集成单 Yaw 电机控制 | 2026 |

两套代码均基于 **跃鹿开源框架**，使用 **FreeRTOS** 作为实时操作系统，强调底层实时性与可靠性。

---

## 🧱 目录结构

```plaintext
New_sentry_chassis/
├── hy_sentry-main/                # 2025 赛季舵轮哨兵
│   ├── application/               # 顶层应用逻辑
│   ├── Drivers/                   # 外设驱动 (CAN, 编码器, 电机等)
│   ├── Inc/ & Src/                # 核心头文件与源文件
│   ├── Middlewares/               # FreeRTOS 等中间件
│   └── README.md                  # 原项目说明
│
├── hy_sentry_-gimbal-main/        # 2026 赛季云台集成底盘
│   ├── Core/                      # 核心 MCU 依赖与启动文件
│   ├── application/               # 应用层任务 (底盘、云台、通信)
│   ├── USB_DEVICE/                # USB 虚拟串口 / 通信模块
│   └── README.md                  
│
└── README.md                      # 本文件 
