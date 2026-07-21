# 项目指导文件

本项目的唯一事实来源为根目录下的 `CLAUDE.md`（K7 智能小车 — RK3576 项目文档）。

**每次开始工作前，必须先完整阅读 `CLAUDE.md`**，并严格遵循其中的约定。如需了解项目完整目录结构，也请查看 `CLAUDE.md` 第 6 节。

## 关键规则

1. 每次对该项目有新的了解、决策或变更，**必须更新 `CLAUDE.md`**
2. 更新项目目录结构时，必须同步更新 `README.md`、`AGENTS.md`、`CLAUDE.md` 及 `docs/` 下所有 `.md` 文件中的目录路径
3. 大文件/第三方代码需通过 `.gitignore` 排除，添加新文件前确认不会被误提交
4. Commit 格式：`<类型>: <简述>`（docs / feat / fix / refactor / chore）
