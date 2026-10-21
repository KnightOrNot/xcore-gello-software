# 贡献与开发检查

## 规则

- 本仓库是 GELLO 上游的本地适配层；优先保持上游接口兼容。
- 新增默认值写入 `config/default.toml`；现场标定不要提交。
- `read` 必须只读，不得在导入或 `--help` 时打开串口。
- 不提交 `third_party/`、`.venv`、模型、图像和日志。

## 检查

```bash
./.venv/bin/python -m compileall xcore_gello_software
./.venv/bin/xcore-gello-software --help
./.venv/bin/xcore-gello-software read --help
```

涉及真实主臂的检查需要现场确认；不要在跟随会话运行时打开同一串口。
