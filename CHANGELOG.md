# 更新记录

## 2026-10-29

- ROS 2 CI 增加临时 Bake 文件读取兼容设置，关闭未配置的容器镜像推送，更新 checkout 运行时，并让构建失败立即终止测试命令。
- 补充 ROS 2 devcontainer 构建与收尾失败的排障说明。设备代码与现场配置保持原样。

## 2026-10-21

- 配置加载保留 Python 3.8–3.10 兼容（条件依赖 tomli），新增只读配置回归测试。

- 新增 `config/default.toml`、`xcore_gello_software/configuration.py`：
  `read` 的串口、频率、采样数和夹爪端点默认值来自 TOML。
- 支持 `XCORE_GELLO_CONFIG` 覆盖默认文件。
- 新增 `docs/index.md`、`docs/installation.md`、
  `docs/reference/{cli,config,errors}.md`、`CONTRIBUTING.md` 和本文件。

## 早期

- 从上游 GELLO 同步代码，新增 CR7 相关适配和统一命令入口。
  上游历史见 `docs/UPSTREAM.md`。
