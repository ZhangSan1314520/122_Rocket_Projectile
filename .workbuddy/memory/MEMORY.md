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
