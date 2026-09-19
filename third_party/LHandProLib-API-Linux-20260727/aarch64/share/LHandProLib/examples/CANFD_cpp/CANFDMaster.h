#ifndef CANFDMASTER_H
#define CANFDMASTER_H

// Linux 后端选择（优先级：高 → 低）:
// 1. 取消下行注释以启用 libcanbus
// 2. CMake 选项 -DLHANDPRO_USE_LIBCANBUS=ON
//
// 默认: SocketCAN
// #define USE_LIBCANBUS

#ifndef USE_LIBCANBUS
#ifdef LHANDPRO_USE_LIBCANBUS
#define USE_LIBCANBUS
#endif
#endif

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

using ReceiveCallback = std::function<void(
    uint32_t id, const std::vector<uint8_t>& data, uint64_t timestamp)>;

class CANFDMaster {
 public:
  CANFDMaster();
  ~CANFDMaster();

  std::vector<std::string> scanDevices();

  bool connect(int deviceIndex, int nomBaud = 1000000,
               int dataBaud = 5000000);
  bool disconnect();
  bool attemptReconnect();

  bool sendData(unsigned int id, const unsigned char* data, unsigned int size,
                int externflag = 0);
  bool sendData(uint32_t id, const std::vector<uint8_t>& data,
                int externflag = 0);

  void setReceiveCallback(ReceiveCallback callback);

  bool isConnected() const;
  int getCurrentDeviceIndex() const;
  uint64_t getLostFrames() const;
  void resetLostFrames();

 private:
#ifndef _WIN32
#ifndef USE_LIBCANBUS
  bool setupCANInterface(const std::string& ifname, int nomBaud, int dataBaud);
#endif
#endif

  void receiveThreadFunc();

 private:
  int dev_index_;
  int nom_baud_;
  int data_baud_;
  std::atomic<bool> is_connected_;
  std::atomic<bool> receive_thread_running_;
  std::thread receive_thread_;
  ReceiveCallback receive_callback_;
  uint64_t lost_frames_{0};

#ifndef _WIN32
#ifdef USE_LIBCANBUS
  void* libcan_handle_;
  void* libusb_handle_;
  int (*fn_CAN_ScanDevice_)();
  int (*fn_CAN_OpenDevice_)(int, int);
  int (*fn_CAN_CloseDevice_)(int, int);
  int (*fn_CANFD_Init_)(int, int, void*);
  int (*fn_CANFD_Transmit_)(int, int, void*, int, int);
  int (*fn_CANFD_Receive_)(int, int, void*, int, int);

  bool loadLibCanBus();
  void unloadLibCanBus();
  bool libcanbusConnect(int deviceIndex, int nomBaud, int dataBaud);
  bool libcanbusDisconnect();
  bool libcanbusSendData(uint32_t id, const std::vector<uint8_t>& data,
                         int externflag);
  void libcanbusReceiveLoop();
#else
  int socket_fd_;
  std::string current_interface_;
#endif
#endif
};

#endif  // CANFDMASTER_H
