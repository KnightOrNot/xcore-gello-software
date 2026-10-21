# 安装与环境

## 要求

- Python 3.11。
- GELLO 主臂通过 USB 串口连接（默认 57600 baud）。
- 用户属于 `dialout`。
- DynamixelSDK 位于 `third_party/DynamixelSDK`（由顶层 setup 或
  `git submodule update --init` 初始化）。

## 安装

```bash
uv venv --python 3.11
uv pip install -r requirements.txt dynamixel-sdk -e .
```

顶层 `./scripts/setup.sh` 会自动完成这些步骤。

## 验证

```bash
xcore-gello-software --help
xcore-gello-software read --help
xcore-gello-software read --samples 5
```

最后一条会打开真实串口并读取主臂；确认主臂已连接且串口未被其他进程占用。
不连接硬件时只运行 `--help`。

## 配置

默认配置位于 `config/default.toml`，`read` 的参数优先于文件。
可用 `XCORE_GELLO_CONFIG=/path/to.toml` 选择另一份文件。
