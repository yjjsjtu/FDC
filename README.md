# Dummyx2 / RDriveC1 开发资料

本仓库用于在多台主机之间同步 Dummyx2 / RDriveC1 的硬件设计、机械模型、嵌入式控制源码、ROS 2 工程和三环控制仿真环境。

## 目录

- `Altium_design_files/`：RDriveC1 的 Altium 设计与生产资料。
- `model/`：执行器机械模型。
- `source_files/dummyx2_rdrivec1/moteus/`：moteus 固件、工具、fdcanusb 与 ROS 2 相关源码，包含本地定制修改。
- `three_loop_sim/`：电流环、速度环、位置环仿真及测试、结果图。
- `USB2CANFD/`：USB 转 CAN-FD 相关资料。

## 在另一台主机上使用

```bash
git clone <本仓库的 GitHub 地址>
cd acuator-release
```

三环仿真的运行和测试方法见 `three_loop_sim/README.md`。

## 版本控制说明

仓库保留源码、设计文件、模型和已有仿真结果；不跟踪可重新生成的 ROS 2 构建目录、Python 缓存、编辑器状态、macOS 元数据以及已解压内容对应的原始 `dummyx2_rdrivec1.tar.gz`。因此克隆后无需 Git LFS，也不会依赖嵌套 Git 子模块。

## 许可

请先阅读根目录的 `Dummyx2 – 使用许可说明`。该许可禁止商业用途；`moteus` 目录内的上游代码另附 Apache-2.0 许可。若需商业使用，请按许可文件中的联系方式获取授权。
