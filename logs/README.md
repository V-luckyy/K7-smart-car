# 实验记录

记录器节点随车辆 launch 启动，但运行时数据默认关闭。配置入口为 `ROS2_src/K7_ros2/src/k7_bringup/config/robot.yaml`；`record:=true` 可令车辆启动后立即记录，否则通过 `/car1/experiment/record` 的 `std_srvs/srv/SetBool` 服务开始或停止记录。默认空闲时不写盘。

每次启用建立 `时间-车辆命名空间-随机ID/`：
- `manifest.json`：任务、算法、记录字段、配置摘要、结束原因。
- `config.yaml`：本次 launch 使用的配置原文，节点启动时缓存并核对 SHA-256；之后改文件不会冒充本次生效配置。
- `metrics.csv`：所选标量，过期量留空，不冒充新测量。
- `bag/`：仅记录选定话题；`rosbag.log` 保存 recorder 错误。

数据不进入 Git。默认 5 Hz、30 分钟上限、512 MB 剩余空间保护。
节点未记录时不订阅数据；禁用日志包启动可消除剩余计时器开销。
修改参数后先停车，再重启整个 launch 和记录，确保所有节点使用一致配置。`config.yaml` 不复制左右标定文件，也不自动记录固件哈希；验收时另行保存它们的版本。

`output_dir` 默认相对启动命令的工作目录，不一定是仓库根目录的 logs；推荐车辆配置使用绝对路径。`logs/` 是实验目录，colcon 的 `log/` 是构建目录，两者不同。

| 配置 | 含义 |
|---|---|
| variables | CSV 可选 x、y、v、w、v_cmd、w_cmd、cte、heading_error、w_stanley、w_apf、solve_ms |
| topics | rosbag 显式话题列表，相对名加本车 namespace，绝对名保持不变；空列表不录 bag |
| rate | CSV 采样频率，默认 5 Hz；不是 bag 消息节流器 |
| max_seconds / min_free_mb | 默认 1800 s / 512 MB，每秒检查，达到限制停止记录，不停止车辆任务 |

要记录控制误差，需同时设置 `controller_node.debug: true`、对应 `debug_fields` 和 recorder 的 `variables`；否则字段留空。未记录时移除数据订阅，开启后也只订阅所选标量对应的话题。默认不录图像、深度或点云。CSV 接收超过 0.5 s 未更新的量留空；接收时效不等于传感器采集时延，请结合 bag 原始时间戳分析。

关闭记录后检查 manifest 的 status/reason 和 rosbag.log。断电或强制终止可能留下未关闭的 manifest、CSV 尾部或不完整 bag，不当作完整实验。数据目录已被 Git 忽略，勿强制提交大包。启动和开关示例见 [命令速查](../docs/ROS2_命令速查.md)。
