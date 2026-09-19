# RS485 Python 示例

## 概述

基于 Python SDK 的 RS485 串口通信示例，支持 **电机运动测试** 和 **传感器数据读取** 两种模式。

## 环境依赖

```bash
cd LHandProLib_RS485_Test_python/
pip install -r requirements.txt
```

## 运行

```bash
python main.py
```

## 操作说明

1. 扫描并选择串口
2. 选择测试模式：
   - `0` — 电机运动测试：使能 → 回零 → 逐轴循环运动
   - `1` — 传感器数据测试：直接读取传感器数据（不使能电机）
3. 按 `Ctrl+C` 退出程序

## 驱动说明

RS485 使用标准串口通信，无需额外驱动或 DLL。

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.py` | 示例主程序入口 |
| `serial_port.py` | 串口通信封装 |
| `requirements.txt` | Python 依赖（pyserial） |
| `lhandprolib_python_sdk/` | Python SDK 封装库（详见该目录下 README） |
