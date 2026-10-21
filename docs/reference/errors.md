# 故障排查

## 串口不存在或无权限

检查 `[read].gello_port`、`/dev/serial/by-id/`、供电和 `dialout` 权限。
加入组后需要重新登录。同一串口只允许一个进程。

## Dynamixel 持续 -3001

通常表示串口被其他程序占用或舵机未响应。确认没有同时运行
`read`、上游实验和 SDK 跟随；检查波特率和主臂供电。

## 夹爪端点配置无效

端点相同或端点方向不符合实际安装时，`load_config` 会拒绝启动。
先使用库默认端点，观察原始角度后再配置。

## tyro 参数解析错误

`read` 只暴露 `--gello-port`、`--hz`、`--samples` 和两个夹爪端点。
上游命令有各自的参数；用 `COMMAND --help` 查看，不要混用。

## 找不到 DynamixelSDK

初始化嵌套 submodule：

```bash
git submodule update --init third_party/DynamixelSDK
```

然后按 `docs/installation.md` 重新安装本包。
