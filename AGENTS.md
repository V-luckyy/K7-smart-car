# 项目指导文件

本项目的唯一事实来源为根目录下的 `CLAUDE.md`（K7 智能小车 — RK3576 项目文档）。

**每次开始工作前，必须先完整阅读 `CLAUDE.md`**，并严格遵循其中的约定。如需了解项目完整目录结构，也请查看 `CLAUDE.md` 第 6 节。

## 关键规则

1. 每次对该项目有新的了解、决策或变更，**必须更新 `CLAUDE.md`**
2. 更新项目目录结构时，必须同步更新 `README.md`、`AGENTS.md`、`CLAUDE.md` 及 `docs/` 下所有 `.md` 文件中的目录路径
3. 大文件/第三方代码需通过 `.gitignore` 排除，添加新文件前确认不会被误提交
4. Commit 格式：`<类型>: <简述>`（docs / feat / fix / refactor / chore）

## 当前入口

- 活动 ROS 工作区：`ROS2_src/K7_ros2/`；统一启动 `src/k7_bringup/launch/robot.launch.py`，统一参数 `src/k7_bringup/config/robot.yaml`。
- `src/` 只保留当前 7 个自有包；旧算法包、旧 launch 和兼容消息已删除，不在活动工作区归档副本。
- 固件：`STM32F407VET6_src/USER/WHEELTEC.uvprojx`，当前只验收 `Diff_Car` / ROS-only。
- 当前手册与架构图放 `docs/`；实验数据放 `logs/`，生成数据不提交；回归放 `tests/`。
- `CLAUDE.md` 只维护现状和索引，修改对应条目，不追加流水日志；历史资料不作为当前行为依据。
- 必须区分源码实现、本机检查、目标机编译、刷写和实车验收，未执行不得宣称完成。
