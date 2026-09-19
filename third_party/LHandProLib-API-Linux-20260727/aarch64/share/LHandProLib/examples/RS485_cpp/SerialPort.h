#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

// RS485 serial port wrapper.
// Send model: write() synchronously writes bytes to the port.
// Receive model: the reader thread forwards raw serial chunks to the SDK. The
// SDK owns RS485 framing, CRC validation, and decode dispatch.
class SerialPort {
 public:
  using ReadCallback = std::function<void(const uint8_t* data, size_t size)>;

  SerialPort() = default;
  ~SerialPort() { close(); }

  // Open port at the given baud rate (default 500000), start reader thread.
  bool open(const std::string& port_name, uint32_t baud_rate = 500000);

  // Close port, wait for reader thread to exit.
  void close();

  bool isOpen() const { return is_open_.load(); }

  // Synchronously write data to the serial port.
  bool write(const uint8_t* data, size_t size);

  void setReadCallback(ReadCallback callback) {
    read_callback_ = std::move(callback);
  }

  // Scan available serial ports.
  static std::vector<std::string> scanAvailablePorts();

 private:
  void readLoop();
  bool platformWrite(const uint8_t* data, size_t size);
  int platformRead(uint8_t* buf, int buf_size);

#ifdef _WIN32
  HANDLE hCom_ = INVALID_HANDLE_VALUE;
#else
  int fd_ = -1;
#endif

  std::atomic<bool> is_open_{false};
  std::thread read_thread_;
  std::atomic<bool> running_{false};
  ReadCallback read_callback_;

  std::mutex write_mutex_;
};
