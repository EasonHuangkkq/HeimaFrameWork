Copyright 2025 人形机器人（上海）有限公司, https://www.openloong.net/

# 简介

OpenLoong控制框架仿真组件

# 使用方法

所有操作均在tools内完成

* 编译：./make.sh（第一次编译需安装sudo apt install libgl-dev)
* 运行：./run_mujoco_loco.sh [延时ms数]

固定基座可注释掉xml中 `<freejoint/>`一行
辅助悬挂与否f键，高度g-、h+


## 务必区分算法仿真和全链仿真！

算法仿真：直接调用loco/mani算法库，按键在mujoco界面内定义
全链仿真：配合loong_base，完全模拟实机操作流程链，发布于loong_sim_sdk_release

预编译版仅在ubuntu22测试，其余平台需自行编译，有报错可及时反馈
