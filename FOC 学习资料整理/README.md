# FOC 学习资料导航

> 本文档用于整理当前目录中的 FOC、关节驱动和机器人控制资料，并给出一条从基础原理到系统实践的学习路线。  
> 外部链接核对日期：2026-09-18。仓库内容和许可可能继续变化，请以项目当前页面为准。

## 1. 快速入口

- 本地资料包：[木子哥的FOC.zip](./木子哥的FOC.zip)
- FOC 基础：[《自制 FOC 驱动器：深入浅出讲解 FOC 算法与 SVPWM 技术》](https://zhuanlan.zhihu.com/p/147659820)
- 入门实现：[灯哥开源 FOC 双路无刷电机控制器](https://github.com/ToanTech/Deng-s-foc-controller)
- 关节驱动：资料包内的 RdriveC1、Drivex2 和 moteus 源码
- 机器人系统：[Dummy-Robot 六轴机械臂](https://github.com/peng-zhihui/Dummy-Robot)
- 动力学进阶：[《工业机器人重力-摩擦力补偿辨识》](https://zhuanlan.zhihu.com/p/604255934)

建议先阅读第 4 节的学习路线，再按第 2 节中的路径查找本地文件。

## 2. 本地资料：木子哥的 FOC

### 2.1 资料包结构

压缩包约 95.6 MiB。内部顶层目录名称为 `acuator-release/`（原文件如此拼写），主要结构如下：

```text
木子哥的FOC.zip
└── acuator-release/
    ├── model/
    │   └── actuator-x2.step
    ├── Altium_design_files/
    │   ├── RdriveC1_Sch.pdf
    │   ├── RdriveC1.SchDoc
    │   ├── RdriveC1.PcbDoc
    │   ├── RdriveC1_bom.xlsx
    │   └── RdriveC1_Gerber.zip
    ├── source_files/
    │   ├── README_INSTALL_CN.md
    │   └── dummyx2_rdrivec1.tar.gz
    ├── USB2CANFD/
    │   ├── USB2CANFD_sch.pdf
    │   ├── Main.SchDoc
    │   ├── USB2CANFD.PcbDoc
    │   └── USB2CANFD_bom.xlsx
    └── Dummyx2 – 使用许可说明
```

`__MACOSX/`、`.DS_Store`、`__pycache__/`、日志和编译产物属于系统或构建生成文件，不作为学习资料入口。

### 2.2 机械模型

| 文件 | 格式 | 用途 | 建议关注 |
| --- | --- | --- | --- |
| `acuator-release/model/actuator-x2.step` | STEP 三维模型 | 查看执行器外形、接口和装配空间，可导入常见 CAD 软件 | 安装孔、轴线、外形尺寸、与机械臂连杆的干涉关系 |

该模型适合用于整机布局和结构适配，不应仅凭模型推断材料、加工公差或承载能力。

### 2.3 RdriveC1 控制器硬件

| 文件 | 格式 | 用途 | 建议阅读入口 |
| --- | --- | --- | --- |
| `acuator-release/Altium_design_files/RdriveC1_Sch.pdf` | 两页 PDF | 第一页为电气原理图，第二页为 PCB 元件布局图 | 先看 `POWER`、`CAN BUS`、MCU/编码器接口和 `BLDC Driver` 功能分区 |
| `acuator-release/Altium_design_files/RdriveC1.SchDoc` | Altium 原理图源文件 | 查看网络连接、器件属性并进行二次设计 | 与 PDF 对照阅读，修改前先确认器件库和 Altium 版本 |
| `acuator-release/Altium_design_files/RdriveC1.PcbDoc` | Altium PCB 源文件 | 查看布局、布线、铜皮、层叠和设计规则 | 重点观察功率回路、电流采样、栅极驱动、CAN 和编码器信号 |
| `acuator-release/Altium_design_files/RdriveC1_bom.xlsx` | BOM 表 | 采购与器件识别 | 字段包含数量、位号、描述、参数、封装和所在层；主要器件包括 STM32G474、DRV8323、TCAN1057A 和 AS5047P |
| `acuator-release/Altium_design_files/RdriveC1_Gerber.zip` | Gerber 制造文件 | PCB 加工输出 | 生产前必须重新核对层叠、钻孔、板框、版本和制造规则 |

RdriveC1 展示了一套完整的无刷伺服控制器硬件链路：电源变换、STM32G4 控制、三相栅极驱动及 MOS 功率级、电流/电压采样、磁编码器和 CAN-FD 通信。适合在掌握 FOC 基础后结合固件中的 `moteus/fw/foc.cc`、`moteus/fw/bldc_servo.cc` 等模块对照学习。

### 2.4 USB2CANFD 适配器硬件

| 文件 | 格式 | 用途 | 建议阅读入口 |
| --- | --- | --- | --- |
| `acuator-release/USB2CANFD/USB2CANFD_sch.pdf` | 两页 PDF | 第一页为适配器原理图，第二页为 PCB 布局图 | 按 `LCD`、`CAN`、`MCU`、`CAN BUS`、`CORE-REF`、`USB-PROG`、`POWER` 和 `IOs` 分区阅读 |
| `acuator-release/USB2CANFD/Main.SchDoc` | Altium 原理图源文件 | 查看和修改适配器电路 | 对照接口定义、收发器、电源和 USB 电路 |
| `acuator-release/USB2CANFD/USB2CANFD.PcbDoc` | Altium PCB 源文件 | 查看布局布线 | 关注 CAN 差分走线、保护器件、电源回路和接口方向 |
| `acuator-release/USB2CANFD/USB2CANFD_bom.xlsx` | BOM 表 | 采购与器件识别 | 字段包含参数、描述、位号、封装和数量；主要器件包括 STM32G474、TCAN1057A、INA226、OLED 模块和 USB 接口 |

该板用于把上位机与 CAN-FD 总线连接起来，是后续设备发现、配置、标定、遥测和固件刷写的基础工具。实际使用前需要确认固件、USB 权限、CAN 位速率、数据位速率以及总线终端电阻。

### 2.5 源码、工具链与使用文档

首先阅读：

1. `acuator-release/source_files/README_INSTALL_CN.md`：Ubuntu 环境、Python 工具、Bazel 编译、CAN-FD/SWD 刷写和常见问题。
2. `acuator-release/source_files/dummyx2_rdrivec1.tar.gz`：完整源码包。解包后源码根目录为 `moteus/`。
3. `moteus/acuator/readme.md`：Drivex2 双编码器关节电机手册。`acuator` 是包内原始目录名。

源码包中的重点模块如下：

| 路径 | 内容 | 学习重点 |
| --- | --- | --- |
| `moteus/fw/` | STM32G4 无刷伺服主固件、CAN bootloader 和刷写脚本 | FOC、电流环、位置/速度控制、编码器、故障保护、CAN-FD |
| `moteus/hw/` | moteus 多版本控制器硬件设计 | 对比 C1、N1、X1 等控制器的功率级和接口设计 |
| `moteus/lib/python/` | Python 客户端库和示例 | 查询状态、位置/速度/力矩命令、多电机同步控制 |
| `moteus/utils/` | 配置、标定、遥测、补偿和测试工具 | `moteus_tool`、编码器标定、CAN ID 修改、齿槽/编码器补偿 |
| `moteus/docs/` | 快速入门、协议、配置和硬件参考 | 先读 `quick-start.md`、`reference.md` 和 `protocol/` |
| `moteus/fdcanusb/` | USB 转 CAN-FD 固件、规则与刷写脚本 | 独立 Bazel 工程、udev 权限、适配器固件刷写 |
| `moteus/configs/` | 控制器配置示例 | 参数命名、控制限制与配置备份 |
| `moteus/webgui_c1/`、`moteus/webgui.py` | Web 图形化控制界面 | 多关节配置、状态显示和交互控制 |
| `moteus/ros2/` | ROS 2 相关资料和源码包 | 将关节控制接入机器人软件栈 |
| `moteus/dynamics/` | 基于 Pinocchio 的逆动力学补偿示例 | URDF、重力补偿、全动力学补偿和 CAN 控制 |
| `moteus/acuator/` | Drivex2 关节说明 | 双编码器、减速器、MIT 风格阻抗控制、前馈力矩与 API 示例 |

Drivex2 手册中的示例覆盖：扫描 CAN ID、读取位置/速度/扭矩/温度/电流、位置运动、速度和加速度限制、MIT 风格 `Kp/Kd` 缩放、前馈扭矩、纯力矩模式、配置写入和编码器校准。首次实验时应从低电流、低速度和低扭矩开始。

### 2.6 本地资料之间的关系

| 层次 | 对应资料 | 作用 |
| --- | --- | --- |
| 机械层 | `acuator-release/model/actuator-x2.step` | 定义执行器外形和机械接口 |
| 功率与控制硬件 | RdriveC1 原理图、PCB、BOM、Gerber | 驱动无刷电机并采集电流、位置等反馈 |
| 嵌入式控制 | `moteus/fw/` | 执行 FOC、伺服环、保护与通信 |
| 总线与调试 | USB2CANFD、`moteus/fdcanusb/`、Python 工具 | 连接、配置、标定、遥测和刷写控制器 |
| 关节接口 | `moteus/acuator/`、Web GUI、ROS 2 | 向上层提供位置、速度、力矩和阻抗控制 |
| 机器人动力学 | `moteus/dynamics/` | 根据模型计算重力和动力学前馈扭矩 |

## 3. 外部参考资料

### 3.1 原理与算法

#### [自制 FOC 驱动器：深入浅出讲解 FOC 算法与 SVPWM 技术](https://zhuanlan.zhihu.com/p/147659820)

适合作为第一篇系统性入门资料。文章从无刷电机和普通电调的区别讲起，逐步介绍：

- BLDC/PMSM 与三相逆变桥；
- Clarke/Park 变换及其逆变换；
- 将三相耦合量变换为便于控制的 `Id/Iq` 分量；
- 电流、速度、位置控制环；
- 空间电压矢量、扇区判断和七段式 SVPWM。

阅读目标不是背公式，而是能画出“传感器反馈 → 坐标变换 → 控制器 → 逆变换 → SVPWM → 功率级”的完整信号链。

#### [工业机器人重力-摩擦力补偿辨识](https://zhuanlan.zhihu.com/p/604255934)

属于机器人关节控制进阶资料，重点包括：

- 机器人动力学模型中的惯性、科里奥利、重力、摩擦和外力项；
- 库仑-粘滞摩擦模型；
- 使用正反向恒速轨迹分离摩擦力和重力影响；
- 使用数据拟合与最小二乘法辨识摩擦、重力参数；
- 将辨识结果作为前馈补偿，提高低速运动和力控表现。

可与本地 `moteus/dynamics/` 的 Pinocchio 逆动力学补偿以及 Drivex2 的 `feedforward_torque` 接口对照学习。

### 3.2 开源项目

| 项目 | 技术定位 | 适合学习的内容 | 与本地资料的关系 |
| --- | --- | --- | --- |
| [unlir/XDrive](https://github.com/unlir/XDrive) | 带多功能接口和闭环控制的步进电机 | 主应用固件、USB bootloader、硬件、外壳和使用说明 | 可与无刷 FOC 关节比较：电机类型、换相方式、位置反馈和控制目标均不同 |
| [ToanTech/Deng-s-foc-controller](https://github.com/ToanTech/Deng-s-foc-controller) | 基于 ESP32 与 SimpleFOC 的双路无刷驱动板 | 低成本硬件、开闭环位置/速度/力矩控制、编码器接入和大量入门例程 | 适合在阅读 RdriveC1/moteus 之前建立可运行的 FOC 直觉 |
| [peng-zhihui/Dummy-Robot](https://github.com/peng-zhihui/Dummy-Robot) | 超迷你六轴机械臂完整项目 | 机械模型、PCB、STM32/FreeRTOS 固件、CAN 关节网络、运动学、上位机和无线示教器 | 提供从单关节驱动扩展到整机协调控制的系统级参考 |

### 3.3 本地资料涉及的上游与配套入口

- [mjbots/moteus 源码](https://github.com/mjbots/moteus)：本地固件和工具链的主要上游项目。
- [moteus 官方文档](https://mjbots.github.io/moteus/)：快速入门、寄存器、协议、配置与硬件参考。
- [SwitchPi RdriveC1](https://gitee.com/switchpi/rdrive-controller-c1)：本地 Drivex2 手册中注明的控制器项目入口。
- [SwitchPi DummyX2](https://gitee.com/switchpi/dummyx2)：本地手册给出的 DummyX2/Drivex2 相关源码入口。
- [SimpleFOC 文档](https://docs.simplefoc.com/)：灯哥 FOC 项目所使用控制库的官方资料。

## 4. 推荐学习路线

| 阶段 | 学习目标 | 建议资料 | 完成标志 |
| --- | --- | --- | --- |
| 1. FOC/SVPWM 原理 | 理解三相电机、坐标变换、`Id/Iq` 解耦和空间矢量调制 | FOC/SVPWM 知乎文章 | 能独立画出有感 FOC 控制框图，并解释每次变换的输入输出 |
| 2. 简化实现 | 用较低门槛的平台观察开环与闭环效果 | DengFOC、SimpleFOC 文档 | 能区分位置、速度、力矩模式，理解编码器方向和极对数配置 |
| 3. 控制器硬件 | 理解从 MCU 到三相功率级的电路链路 | RdriveC1 PDF、SchDoc、PcbDoc、BOM | 能在原理图中定位电源、栅极驱动、MOS、电流采样、编码器和 CAN |
| 4. 固件与工具链 | 建立、编译、配置和刷写控制器 | `README_INSTALL_CN.md`、`moteus/fw/`、`moteus/utils/` | 能说明 CAN-FD 与 SWD 的使用边界，并找到固件、bootloader 和配置入口 |
| 5. 单关节控制 | 掌握状态查询、运动限制、力矩和阻抗接口 | `moteus/acuator/readme.md`、Python 示例、Web GUI | 能在安全限幅下完成单关节查询、缓慢运动和停止 |
| 6. 多关节与机器人 | 理解总线组网、同步、运动学和上位机分层 | Dummy-Robot、本地 ROS 2/Web GUI | 能说明上位机、主控、CAN 总线和各关节控制器的职责 |
| 7. 动力学补偿 | 理解重力/摩擦辨识和前馈补偿 | 补偿辨识文章、`moteus/dynamics/` | 能解释补偿扭矩的来源、辨识数据需求和安全验证方法 |

也可把 XDrive 插入第 2～3 阶段，作为闭环步进电机与无刷 FOC 伺服的对照案例。

## 5. 实验安全与调试建议

> **注意：资料涉及 24 V 级电源、三相功率电路和可产生较大力矩的机器人关节。错误接线、失控或短路可能损坏设备并造成人身伤害。**

1. 首次上电使用带限流功能的实验电源，先核对极性、额定电压、绝缘和短路情况。
2. 空载或卸下连杆调试单个关节；固定电机本体，避免输出轴、线缆或松动物体突然旋转。
3. 从低电流、低扭矩、低速度和低增益开始，确认电机相序、编码器方向和零位后再逐步增加限制。
4. 保留可靠的断电或急停方式。运行程序前确认机械运动范围内没有人员、工具和碰撞物。
5. CAN/CAN-FD 总线应核对 `CAN_H`、`CAN_L`、公共地、位速率、数据位速率和两端终端电阻；多个节点上线前逐一设置唯一 ID。
6. 刷写固件时保持电源和连接稳定。只有在 CAN bootloader 不可用或需要完整恢复时再使用 SWD。
7. 不要把 Gerber 或 BOM 直接视为可投产版本；生产前应完成 ERC/DRC、器件可得性、封装、散热、电流能力和板厂规则复核。

## 6. 许可与使用范围

- 压缩包中的 `Dummyx2 – 使用许可说明` 表明：DummyX2 面向开发者和个人用户，但禁止未经授权的商业用途；商业集成或企业内部商业化系统需联系 `xin.li@switchpi.com` 获取单独授权。该限制不同于常见的 OSI 开源许可证。
- 本地源码包中的上游 moteus 文件默认采用 Apache License 2.0，但个别文件或第三方依赖可能另有声明，应以相应目录中的许可证为准。
- XDrive 和 DengFOC 仓库页面标注 GPL-3.0，复制、修改和分发时应遵守相应许可证义务。
- 截至链接核对日期，Dummy-Robot 仓库根目录未见明确的通用许可证文件。不要仅因源码公开就默认可以复制、商用或再分发；使用前应向作者核实授权范围。
- 将多个项目的代码、硬件文件或文档组合使用时，需要分别检查每一部分的许可兼容性。本文仅作资料导航，不构成法律意见。

## 7. 术语速查

| 缩写 | 含义 |
| --- | --- |
| FOC | Field-Oriented Control，磁场定向/矢量控制 |
| SVPWM | Space Vector Pulse Width Modulation，空间矢量脉宽调制 |
| BLDC | Brushless DC Motor，无刷直流电机 |
| PMSM | Permanent Magnet Synchronous Motor，永磁同步电机 |
| CAN-FD | 支持更高数据速率和更长数据段的 CAN 协议扩展 |
| SWD | Serial Wire Debug，ARM 常用调试与烧录接口 |
| BOM | Bill of Materials，物料清单 |
| Gerber | PCB 制造数据格式 |
| MIT 模式 | 常指位置、速度、`Kp/Kd` 和前馈力矩组合的关节阻抗控制接口 |

## 8. 学习进度清单

- [ ] 读完 FOC/SVPWM 原理文章并画出控制框图
- [ ] 跑通一个 SimpleFOC/DengFOC 基础例程
- [ ] 完成 RdriveC1 原理图功能分区标注
- [ ] 阅读 moteus 安装、编译和刷写说明
- [ ] 理解 USB2CANFD 与 CAN 总线接线和配置
- [ ] 在安全限幅下完成单关节查询、运动和停止
- [ ] 阅读 Dummy-Robot 的系统架构与 CAN 多关节控制方式
- [ ] 对照文章和 `moteus/dynamics/` 理解重力、摩擦与动力学补偿
