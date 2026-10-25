# K7 ROS 2 工作区

当前仅有 7 个自有包：`k7_msgs`、`k7_bringup`、`k7_camera`、`k7_control`、`k7_tasks`、`k7_experiment`、`k7_description`。

统一启动：`src/k7_bringup/launch/robot.launch.py`；统一配置：`src/k7_bringup/config/robot.yaml`。任务用 `profile:=manual / trajectory / formation` 选择，启动后默认 IDLE。`test_mode:=no_perception` 可选用于单车路径或编队测试，默认关闭；跳过相机与双目节点，使用 Stanley-only，guard 限速 0.05 m/s，不提供避障保护。旧算法包、启动脚本和红外接口已删除。

在 K7 上执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
ros2 launch k7_bringup robot.launch.py namespace:=car1 profile:=trajectory
```

部署已有工作区时，先按[升级清理说明](../../docs/K7_上手操作指南.md#2-k7-软件部署)移除旧构建产物；目录合并复制会保留已删除的旧源码，应替换板端 `src/`。

项目事实见 [CLAUDE.md](../../CLAUDE.md)，完整操作入口见[文档目录](../../docs/README.md)。当前尚未完成目标机编译、固件刷写与实车验收。
