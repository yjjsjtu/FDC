# FDC

本仓库用于整理 FOC 学习资料、DengFOC 资料、离线 FOC 仿真代码，以及后续真实硬件接入相关脚本。

## 目录说明

- `Deng-s-foc-controller-master/`：灯哥开源 FOC 控制器资料和说明。
- `FOC 学习资料整理/`：FOC、关节驱动、moteus/Drivex2 相关学习资料。
- `FOC 学习资料整理/foc_simulation/`：Python 离线仿真项目，包含三相 FOC、SVPWM、速度环、电流环、位置三环、参数辨识和补偿验证。
- `Arduino_Linux_环境/*.sh`：本机 Arduino/SimpleFOC 相关启动和编译脚本。

## 没有上传的内容

为了让 GitHub 仓库适合协作开发，本仓库不会提交完整 Arduino 安装目录、工具链、下载缓存、压缩包和仿真生成 CSV。

这些内容体积很大，且很多单文件超过 GitHub 普通仓库限制。需要时请按文档重新下载或在本机重新运行脚本生成。

## 快速运行仿真

```bash
cd "FOC 学习资料整理/foc_simulation"
python3 examples/run_svpwm_demo.py
python3 examples/run_velocity_foc.py
python3 examples/run_control_optimization_compare.py
python3 examples/run_position_three_loop.py
python3 -m unittest discover -s tests
```

更多说明见：

- `FOC 学习资料整理/README.md`
- `FOC 学习资料整理/foc_simulation/README.md`
- `FOC 学习资料整理/foc_simulation/docs/位置三环控制说明.md`
