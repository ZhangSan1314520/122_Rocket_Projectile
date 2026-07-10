# 122_Rocket_Projectile 项目长期记忆

## 硬件设计

### 电源架构（2026-07-01）
- **VM 母线来源**：外部电源输入连接器 **P2**，板边丝印标注为 **"VIN 12-28V"**。
- **VM 去向**：
  - TPS54302 降压电路输入（VDD_5V → VDD_3V3）
  - 4 路 EG2226 栅极驱动 + H 桥 MOSFET 功率级
  - INA139 电流采样分流电阻
  - 经分压后送入 MCU PA1 做 ADC 母线电压监测
- **VM 范围**：按丝印和 TPS54302 额定，标称 12-28V，但需注意过压保护设计。

## 代码约定
- VM 母线 ADC 句柄在兄弟项目 `ClosedLoop_StepperMotor` 中定义为 `&hadc1`；本项目中 `bsp.hpp` 尚未定义 `VM_hadc`。

## 调试环境
- **ST-Link 只认特定 USB 口**：其他口报"未知 USB 设备 描述符请求失败"，固定插能识别的那个口（疑似 USB2.0 直连主板）。识别后 BUSID 形如 `4-4`(VID:PID `0483:3752`)，用 `usbipd attach --wsl --busid 4-4` 挂到 WSL 发行版 `dev`（需管理员；attach 前关闭占用 COM5 的 Windows 程序，否则设备不进 WSL）。WSL 内 OpenOCD 已装，烧录命令见 2026-07-07 日志。
