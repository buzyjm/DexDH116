#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Python 串口封装类
用于 RS485 通信

发送/接收模型：
  发送：write() 同步写入串口，不排队、不等待响应
  接收：后台线程持续读取串口，将原始字节片段回调给 SDK
"""

import sys
import glob
import threading
import time
from typing import Optional, List, Callable


class SerialPort:
    """串口封装类"""

    def __init__(self):
        self.serial = None
        self.is_open = False
        self._read_thread: Optional[threading.Thread] = None
        self._running = False
        self.read_callback: Optional[Callable[[bytes], None]] = None
        self._write_lock = threading.Lock()

        # 尝试导入 pyserial
        try:
            import serial
            import serial.tools.list_ports
            self.serial_module = serial
        except ImportError:
            raise ImportError("需要安装 pyserial 库: pip install pyserial")

    def scan_available_ports(self) -> List[str]:
        """扫描可用串口"""
        ports = []
        try:
            for port in self.serial_module.tools.list_ports.comports():
                device = port.device
                # Linux 下只显示 USB 串口设备（过滤掉板载 UART 如 ttyAMA0）
                if sys.platform.startswith('linux'):
                    usb_serial_prefixes = ('ttyUSB', 'ttyACM', 'ttyXRUSB')
                    if not any(prefix in device for prefix in usb_serial_prefixes):
                        continue
                ports.append(device)
        except Exception as e:
            print(f"扫描串口失败: {e}")

        # ttyXRUSB 设备使用私有驱动，不在 pyserial 的 comports() 列表中
        # 直接用 glob 补充扫描
        if sys.platform.startswith('linux'):
            for device in sorted(glob.glob('/dev/ttyXRUSB*')):
                if device not in ports:
                    ports.append(device)

        return ports

    def open(self, port_name: str, baud_rate: int = 500000,
             bytesize: int = 8, parity: str = 'N',
             stopbits: int = 1, timeout: float = 0.001) -> bool:
        """打开串口"""
        if self.is_open:
            return False

        try:
            self.serial = self.serial_module.Serial(
                port=port_name,
                baudrate=baud_rate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout
            )
            self.is_open = True
            self._running = True
            self._read_thread = threading.Thread(
                target=self._read_loop, daemon=True
            )
            self._read_thread.start()
            return True
        except Exception as e:
            print(f"打开串口失败: {e}")
            return False

    def close(self):
        """关闭串口"""
        self._running = False
        if self._read_thread and self._read_thread.is_alive():
            self._read_thread.join(timeout=2.0)
        if self.serial and self.is_open:
            self.serial.close()
            self.serial = None
        self.is_open = False

    def write(self, data: bytes) -> int:
        """同步发送数据"""
        if not self.is_open or not self.serial:
            return 0
        try:
            with self._write_lock:
                written = self.serial.write(data)
                self.serial.flush()
            return int(written)
        except Exception as e:
            print(f"串口发送失败: {e}")
            return 0

    def set_read_callback(self, callback: Callable[[bytes], None]):
        """设置接收回调"""
        self.read_callback = callback

    def _read_loop(self):
        """后台接收线程：读取串口数据，将原始片段交给 SDK。"""
        while self._running:
            if not self.is_open or not self.serial:
                time.sleep(0.001)
                continue

            try:
                chunk = self.serial.read(self.serial.in_waiting or 1)
            except Exception as e:
                print(f"串口读取失败: {e}")
                break

            if not chunk:
                continue

            if self.read_callback:
                try:
                    self.read_callback(bytes(chunk))
                except Exception as e:
                    print(f"读取回调执行失败: {e}")

        if self.serial and self.is_open:
            self.serial.close()
            self.serial = None
        self.is_open = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False
