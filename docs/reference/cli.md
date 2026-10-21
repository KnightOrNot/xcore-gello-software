# CLI 参考

```bash
xcore-gello-software COMMAND [OPTIONS]
```

| 命令 | 行为 |
| --- | --- |
| `read` | 只读主臂关节和夹爪状态 |
| `launch-nodes` | 启动上游节点 |
| `run-env` | 运行上游实验环境 |
| `launch-yaml` | 使用 YAML 配置启动 |
| `quick-run` | 上游快速实验入口 |
| `camera-server` / `camera-client` | 上游相机节点 |

## read

| 参数 | 默认值 / 用途 |
| --- | --- |
| `--gello-port PATH` | `[read].gello_port` |
| `--hz HZ` | `[read].hz`，默认 10 |
| `--samples N` | `[read].samples`，0 表示持续读取 |
| `--gripper-open-deg` | 夹爪开端点 |
| `--gripper-close-deg` | 夹爪闭端点 |

未提供的参数使用 `config/default.toml` 的 `[read]`。`read` 只打开主臂串口，
不连接 CR7，也不发送运动命令。

## 上游命令

`launch-nodes`、`run-env` 等继承上游 GELLO 的参数和风险；它们可能启动仿真、
相机或机器人控制。使用前阅读 `docs/UPSTREAM.md` 和对应 `--help`，
不要在真实实验进行时启动第二个 GELLO 进程。
