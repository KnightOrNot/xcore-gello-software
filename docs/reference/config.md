# 配置参考

默认文件：[config/default.toml](../../config/default.toml)。优先级：

```text
read 命令行参数 > XCORE_GELLO_CONFIG 指定的文件 > config/default.toml > 内置默认值
```

## read

```toml
[read]
gello_port = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB4C7PQ-if00-port0"
hz = 10.0
samples = 0
gripper_open_deg = 0.0
gripper_close_deg = 0.0
gripper_use_library_endpoints = true
```

- `gello_port` 必须是稳定的 by-id 路径，不能用会变化的 `/dev/ttyUSB0`。
- `hz` 必须为正；`samples=0` 表示读到 Ctrl+C。
- 默认使用库自己的夹爪端点；只有 `gripper_use_library_endpoints=false` 时，
  文件中的端点才生效，且两个端点必须不同。

## 与 controller 的区别

controller 的 GELLO 串口、方向映射和 CR7 标定在
`xcore-sdk-python/config/default.toml` 与现场 JSON 中，不在本文件。
本文件只影响 `xcore-gello-software read`。

## 覆盖

```bash
XCORE_GELLO_CONFIG=config/experiment.toml xcore-gello-software read --samples 1
```

未知表或字段会被拒绝。
