#include "CANFDMaster.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>

#ifdef _WIN32

// Windows backend: HCanbus SDK.
#include <HCanbus.h>

namespace {

constexpr char kDlcToLength[] = {0, 1, 2, 3, 4, 5, 6, 7,
                                 8, 12, 16, 20, 24, 32, 48, 64};

char lengthToDlc(char length) {
  if (length <= 8) return length;
  if (length <= 12) return 9;
  if (length <= 16) return 10;
  if (length <= 20) return 11;
  if (length <= 24) return 12;
  if (length <= 32) return 13;
  if (length <= 48) return 14;
  return 15;
}

}  // namespace

CANFDMaster::CANFDMaster()
    : dev_index_(-1),
      nom_baud_(0),
      data_baud_(0),
      is_connected_(false),
      receive_thread_running_(false) {}

CANFDMaster::~CANFDMaster() {
  if (is_connected_) {
    disconnect();
  }
}

std::vector<std::string> CANFDMaster::scanDevices() {
  std::vector<std::string> devices;

  const int dev_count = CAN_ScanDevice();
  for (int i = 0; i < dev_count; ++i) {
    Dev_Info info;
    if (CAN_ReadDevInfo(i, &info) == 0) {
      devices.push_back(std::to_string(i) + " " + std::string(info.HW_Type));
    } else {
      devices.push_back(std::to_string(i));
    }
  }

  return devices;
}

bool CANFDMaster::connect(int deviceIndex, int nomBaud, int dataBaud) {
  if (is_connected_) {
    disconnect();
  }

  if (CAN_OpenDevice(deviceIndex) != 0) {
    return false;
  }

  CanFD_Config cancfg;
  memset(&cancfg, 0, sizeof(cancfg));
  cancfg.Model = 0;
  cancfg.NomBaud = nomBaud;
  cancfg.DatBaud = dataBaud;
  cancfg.Config = 0x0001 | 0x0002 | 0x0004;
  cancfg.Cantype = 1;

  cancfg.NomPre = 2;
  cancfg.NomTseg1 = 31;
  cancfg.NomTseg2 = 8;
  cancfg.NomSJW = 5;

  cancfg.DatPre = 1;
  cancfg.DatTseg1 = 11;
  cancfg.DatTseg2 = 4;
  cancfg.DatSJW = 2;

  if (CANFD_Init(deviceIndex, &cancfg) != 0) {
    CAN_CloseDevice(deviceIndex);
    return false;
  }

  dev_index_ = deviceIndex;
  nom_baud_ = nomBaud;
  data_baud_ = dataBaud;
  is_connected_ = true;

  receive_thread_running_ = true;
  receive_thread_ = std::thread(&CANFDMaster::receiveThreadFunc, this);

  resetLostFrames();
  return true;
}

bool CANFDMaster::disconnect() {
  if (!is_connected_) {
    return true;
  }

  receive_thread_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  const int device_index = dev_index_;
  const int ret = CAN_CloseDevice(device_index);
  if (ret == 0) {
    dev_index_ = -1;
    is_connected_ = false;
    resetLostFrames();
    return true;
  }

  return false;
}

bool CANFDMaster::attemptReconnect() {
  const int device_index = dev_index_;
  const int nom_baud = nom_baud_;
  const int data_baud = data_baud_;
  disconnect();
  return connect(device_index, nom_baud, data_baud);
}

bool CANFDMaster::sendData(unsigned int id, const unsigned char* data,
                           unsigned int size, int externflag) {
  return sendData(static_cast<uint32_t>(id),
                  std::vector<uint8_t>(data, data + size), externflag);
}

bool CANFDMaster::sendData(uint32_t id, const std::vector<uint8_t>& data,
                           int externflag) {
  if (!is_connected_) {
    return false;
  }

  CanFD_Msg msg;
  memset(&msg, 0, sizeof(msg));
  msg.DLC = lengthToDlc(64);
  msg.ExternFlag = externflag;
  msg.RemoteFlag = 0;
  msg.ID = id;
  msg.FrameType = 0x04;

  const size_t copy_size = std::min(data.size(), size_t(64));
  memcpy(msg.Data, data.data(), copy_size);

  const int ret = CANFD_Transmit(dev_index_, &msg, 1, 100);
  const bool success = (ret == 1);
  if (!success) {
    ++lost_frames_;
  }
  return success;
}

void CANFDMaster::setReceiveCallback(ReceiveCallback callback) {
  receive_callback_ = callback;
}

void CANFDMaster::receiveThreadFunc() {
  while (receive_thread_running_) {
    if (!is_connected_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    CanFD_Msg msgs[500];
    const int count = CANFD_Receive(dev_index_, msgs, 500, 50);

    if (count > 0 && receive_callback_) {
      for (int i = 0; i < count; ++i) {
        const CanFD_Msg& msg = msgs[i];
        const int data_length = msg.DLC < 16 ? kDlcToLength[msg.DLC] : 64;

        std::vector<uint8_t> rxdata(data_length);
        memcpy(rxdata.data(), msg.Data, data_length);

        if (msg.ErrSatus != 0) {
          ++lost_frames_;
        }

        receive_callback_(msg.ID, rxdata, msg.TimeStamp);
      }
    } else if (count < 0) {
      ++lost_frames_;
    }

    if (lost_frames_ > (1 << 20)) {
      lost_frames_ = 0;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

bool CANFDMaster::isConnected() const { return is_connected_; }

int CANFDMaster::getCurrentDeviceIndex() const { return dev_index_; }

uint64_t CANFDMaster::getLostFrames() const { return lost_frames_; }

void CANFDMaster::resetLostFrames() { lost_frames_ = 0; }

#else

// Linux backend: SocketCAN by default, or libcanbus when USE_LIBCANBUS is set.
#include <dirent.h>
#ifdef USE_LIBCANBUS
#include <dlfcn.h>
#endif
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#ifdef USE_LIBCANBUS

#pragma pack(push, 1)
struct LibCanFD_Config {
  unsigned int NomBaud;
  unsigned int DatBaud;
  unsigned short NomPre;
  unsigned char NomTseg1;
  unsigned char NomTseg2;
  unsigned char NomSJW;
  unsigned char DatPre;
  unsigned char DatTseg1;
  unsigned char DatTseg2;
  unsigned char DatSJW;
  unsigned char Config;
  unsigned char Model;
  unsigned char Cantype;
};

struct LibCanFD_Msg {
  unsigned int ID;
  unsigned int TimeStamp;
  unsigned char FrameType;
  unsigned char DLC;
  unsigned char ExternFlag;
  unsigned char RemoteFlag;
  unsigned char BusSatus;
  unsigned char ErrSatus;
  unsigned char TECounter;
  unsigned char RECounter;
  unsigned char Data[64];
};
#pragma pack(pop)

#endif  // USE_LIBCANBUS

CANFDMaster::CANFDMaster()
    : dev_index_(-1),
      nom_baud_(0),
      data_baud_(0),
      is_connected_(false),
      receive_thread_running_(false),
#ifdef USE_LIBCANBUS
      libcan_handle_(nullptr),
      libusb_handle_(nullptr),
      fn_CAN_ScanDevice_(nullptr),
      fn_CAN_OpenDevice_(nullptr),
      fn_CAN_CloseDevice_(nullptr),
      fn_CANFD_Init_(nullptr),
      fn_CANFD_Transmit_(nullptr),
      fn_CANFD_Receive_(nullptr)
#else
      socket_fd_(-1)
#endif
{
}

CANFDMaster::~CANFDMaster() {
  if (is_connected_) {
    disconnect();
  }
#ifdef USE_LIBCANBUS
  unloadLibCanBus();
#endif
}

#ifndef USE_LIBCANBUS
bool CANFDMaster::setupCANInterface(const std::string& ifname, int nomBaud,
                                    int dataBaud) {
  std::string cmd = "ip link set " + ifname + " down";
  system(cmd.c_str());

  cmd = "ip link set " + ifname + " type can bitrate " +
        std::to_string(nomBaud) + " dbitrate " + std::to_string(dataBaud) +
        " fd on loopback off listen-only off";
  system(cmd.c_str());

  cmd = "ip link set " + ifname + " up";
  system(cmd.c_str());

  return true;
}
#endif

#ifdef USE_LIBCANBUS
bool CANFDMaster::loadLibCanBus() {
  if (libcan_handle_) {
    return true;
  }

  libusb_handle_ =
      dlopen("/usr/local/lib/libusb-1.0.so", RTLD_NOW | RTLD_GLOBAL);
  if (!libusb_handle_) {
    std::cerr << "Failed to load libusb-1.0.so: " << dlerror() << std::endl;
    return false;
  }

  libcan_handle_ = dlopen("/usr/local/lib/libcanbus.so", RTLD_NOW);
  if (!libcan_handle_) {
    std::cerr << "Failed to load libcanbus.so: " << dlerror() << std::endl;
    dlclose(libusb_handle_);
    libusb_handle_ = nullptr;
    return false;
  }

  fn_CAN_ScanDevice_ =
      reinterpret_cast<int (*)()>(dlsym(libcan_handle_, "CAN_ScanDevice"));
  fn_CAN_OpenDevice_ = reinterpret_cast<int (*)(int, int)>(
      dlsym(libcan_handle_, "CAN_OpenDevice"));
  fn_CAN_CloseDevice_ = reinterpret_cast<int (*)(int, int)>(
      dlsym(libcan_handle_, "CAN_CloseDevice"));
  fn_CANFD_Init_ = reinterpret_cast<int (*)(int, int, void*)>(
      dlsym(libcan_handle_, "CANFD_Init"));
  fn_CANFD_Transmit_ = reinterpret_cast<int (*)(int, int, void*, int, int)>(
      dlsym(libcan_handle_, "CANFD_Transmit"));
  fn_CANFD_Receive_ = reinterpret_cast<int (*)(int, int, void*, int, int)>(
      dlsym(libcan_handle_, "CANFD_Receive"));

  if (!fn_CAN_ScanDevice_ || !fn_CAN_OpenDevice_ || !fn_CAN_CloseDevice_ ||
      !fn_CANFD_Init_ || !fn_CANFD_Transmit_ || !fn_CANFD_Receive_) {
    std::cerr << "Failed to resolve libcanbus symbols" << std::endl;
    unloadLibCanBus();
    return false;
  }

  return true;
}

void CANFDMaster::unloadLibCanBus() {
  fn_CAN_ScanDevice_ = nullptr;
  fn_CAN_OpenDevice_ = nullptr;
  fn_CAN_CloseDevice_ = nullptr;
  fn_CANFD_Init_ = nullptr;
  fn_CANFD_Transmit_ = nullptr;
  fn_CANFD_Receive_ = nullptr;

  if (libcan_handle_) {
    dlclose(libcan_handle_);
    libcan_handle_ = nullptr;
  }
  if (libusb_handle_) {
    dlclose(libusb_handle_);
    libusb_handle_ = nullptr;
  }
}
#endif  // USE_LIBCANBUS

std::vector<std::string> CANFDMaster::scanDevices() {
  std::vector<std::string> devices;

#ifdef USE_LIBCANBUS
  if (!loadLibCanBus()) {
    std::cerr << "Failed to load libcanbus library" << std::endl;
    return devices;
  }

  const int count = fn_CAN_ScanDevice_();
  for (int i = 0; i < count; ++i) {
    devices.push_back("libcanbus device " + std::to_string(i));
  }
#else
  DIR* dir = opendir("/sys/class/net");
  if (dir == nullptr) {
    return devices;
  }

  struct dirent* ent = nullptr;
  while ((ent = readdir(dir)) != nullptr) {
    const std::string ifname = ent->d_name;
    if (ifname.rfind("can", 0) == 0) {
      devices.push_back(ifname);
    }
  }
  closedir(dir);
#endif

  return devices;
}

bool CANFDMaster::connect(int deviceIndex, int nomBaud, int dataBaud) {
  if (is_connected_) {
    disconnect();
  }

  dev_index_ = deviceIndex;
  nom_baud_ = nomBaud;
  data_baud_ = dataBaud;

#ifdef USE_LIBCANBUS
  if (!libcanbusConnect(deviceIndex, nomBaud, dataBaud)) {
    dev_index_ = -1;
    return false;
  }
#else
  std::vector<std::string> devices = scanDevices();
  if (deviceIndex < 0 || deviceIndex >= static_cast<int>(devices.size())) {
    dev_index_ = -1;
    return false;
  }

  const std::string ifname = devices[deviceIndex];
  current_interface_ = ifname;

  if (!setupCANInterface(ifname, nomBaud, dataBaud)) {
    std::cerr << "Failed to setup CAN interface" << std::endl;
    dev_index_ = -1;
    current_interface_.clear();
    return false;
  }

  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    perror("socket");
    dev_index_ = -1;
    current_interface_.clear();
    return false;
  }

  int enable_canfd = 1;
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd,
                 sizeof(enable_canfd)) < 0) {
    perror("setsockopt CAN_RAW_FD_FRAMES");
    close(socket_fd_);
    socket_fd_ = -1;
    dev_index_ = -1;
    current_interface_.clear();
    return false;
  }

  struct ifreq ifr;
  memset(&ifr, 0, sizeof(ifr));
  strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    perror("ioctl SIOCGIFINDEX");
    close(socket_fd_);
    socket_fd_ = -1;
    dev_index_ = -1;
    current_interface_.clear();
    return false;
  }

  struct sockaddr_can addr;
  memset(&addr, 0, sizeof(addr));
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(socket_fd_, reinterpret_cast<struct sockaddr*>(&addr),
           sizeof(addr)) < 0) {
    perror("bind");
    close(socket_fd_);
    socket_fd_ = -1;
    dev_index_ = -1;
    current_interface_.clear();
    return false;
  }
#endif

  is_connected_ = true;
  receive_thread_running_ = true;
  receive_thread_ = std::thread(&CANFDMaster::receiveThreadFunc, this);

  resetLostFrames();
  return true;
}

#ifdef USE_LIBCANBUS
bool CANFDMaster::libcanbusConnect(int deviceIndex, int nomBaud,
                                   int dataBaud) {
  if (!loadLibCanBus()) {
    std::cerr << "Failed to load libcanbus library" << std::endl;
    return false;
  }

  int ret = fn_CAN_OpenDevice_(deviceIndex, 0);
  if (ret != 0) {
    std::cerr << "CAN_OpenDevice failed, error: " << ret << std::endl;
    return false;
  }

  LibCanFD_Config cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.NomBaud = nomBaud;
  cfg.DatBaud = dataBaud;
  cfg.Config = 0x01 | 0x02 | 0x04;
  cfg.Cantype = 1;
  cfg.Model = 0;

  cfg.NomPre = 2;
  cfg.NomTseg1 = 31;
  cfg.NomTseg2 = 8;
  cfg.NomSJW = 5;

  cfg.DatPre = 1;
  cfg.DatTseg1 = 11;
  cfg.DatTseg2 = 4;
  cfg.DatSJW = 2;

  ret = fn_CANFD_Init_(deviceIndex, 0, &cfg);
  if (ret != 0) {
    std::cerr << "CANFD_Init failed, error: " << ret << std::endl;
    fn_CAN_CloseDevice_(deviceIndex, 0);
    return false;
  }

  return true;
}

bool CANFDMaster::libcanbusDisconnect() {
  if (dev_index_ >= 0 && fn_CAN_CloseDevice_) {
    const int ret = fn_CAN_CloseDevice_(dev_index_, 0);
    if (ret != 0) {
      std::cerr << "CAN_CloseDevice failed, error: " << ret << std::endl;
      return false;
    }
  }
  return true;
}
#endif  // USE_LIBCANBUS

bool CANFDMaster::disconnect() {
  if (!is_connected_) {
    return true;
  }

  receive_thread_running_ = false;
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

#ifdef USE_LIBCANBUS
  libcanbusDisconnect();
#else
  if (socket_fd_ >= 0) {
    close(socket_fd_);
    socket_fd_ = -1;
  }
  current_interface_.clear();
#endif

  dev_index_ = -1;
  is_connected_ = false;
  resetLostFrames();
  return true;
}

bool CANFDMaster::attemptReconnect() {
  const int device_index = dev_index_;
  const int nom_baud = nom_baud_;
  const int data_baud = data_baud_;
  disconnect();
  return connect(device_index, nom_baud, data_baud);
}

bool CANFDMaster::sendData(unsigned int id, const unsigned char* data,
                           unsigned int size, int externflag) {
  return sendData(static_cast<uint32_t>(id),
                  std::vector<uint8_t>(data, data + size), externflag);
}

bool CANFDMaster::sendData(uint32_t id, const std::vector<uint8_t>& data,
                           int externflag) {
  if (!is_connected_) {
    return false;
  }

#ifdef USE_LIBCANBUS
  return libcanbusSendData(id, data, externflag);
#else
  if (socket_fd_ < 0) {
    return false;
  }

  struct canfd_frame frame;
  memset(&frame, 0, sizeof(frame));

  frame.can_id = externflag ? (id | CAN_EFF_FLAG) : id;
  frame.len = 64;
  frame.flags = CANFD_BRS;

  const size_t copy_size = std::min(data.size(), size_t(64));
  memcpy(frame.data, data.data(), copy_size);

  const ssize_t ret = write(socket_fd_, &frame, sizeof(frame));
  const bool success = (ret == static_cast<ssize_t>(sizeof(frame)));
  if (!success) {
    perror("write");
    ++lost_frames_;
  }
  return success;
#endif
}

#ifdef USE_LIBCANBUS
bool CANFDMaster::libcanbusSendData(uint32_t id,
                                    const std::vector<uint8_t>& data,
                                    int externflag) {
  LibCanFD_Msg msg;
  memset(&msg, 0, sizeof(msg));

  msg.ID = id;
  msg.FrameType = 0x04;
  msg.DLC = 15;
  msg.ExternFlag = externflag;
  msg.RemoteFlag = 0;

  const size_t copy_size = std::min(data.size(), size_t(64));
  memcpy(msg.Data, data.data(), copy_size);

  const int ret = fn_CANFD_Transmit_(dev_index_, 0, &msg, 1, 100);
  const bool success = (ret == 1);
  if (!success) {
    std::cerr << "CANFD_Transmit failed, ret: " << ret << std::endl;
    ++lost_frames_;
  }
  return success;
}
#endif  // USE_LIBCANBUS

void CANFDMaster::setReceiveCallback(ReceiveCallback callback) {
  receive_callback_ = callback;
}

void CANFDMaster::receiveThreadFunc() {
#ifdef USE_LIBCANBUS
  libcanbusReceiveLoop();
#else
  while (receive_thread_running_) {
    if (!is_connected_ || socket_fd_ < 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    struct canfd_frame frame;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 50000;

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(socket_fd_, &readfds);

    const int ret =
        select(socket_fd_ + 1, &readfds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(socket_fd_, &readfds)) {
      const ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
      if (nbytes == static_cast<ssize_t>(sizeof(frame))) {
        if (receive_callback_) {
          uint32_t id = frame.can_id & CAN_EFF_MASK;
          if (!(frame.can_id & CAN_EFF_FLAG)) {
            id = frame.can_id & CAN_SFF_MASK;
          }

          const int data_length = frame.len;
          std::vector<uint8_t> rxdata(data_length);
          memcpy(rxdata.data(), frame.data, data_length);

          const uint64_t timestamp =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  std::chrono::system_clock::now().time_since_epoch())
                  .count();

          receive_callback_(id, rxdata, timestamp);
        }
      } else {
        ++lost_frames_;
      }
    } else if (ret < 0) {
      ++lost_frames_;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
#endif
}

#ifdef USE_LIBCANBUS
void CANFDMaster::libcanbusReceiveLoop() {
  static constexpr int kDlcToLength[] = {0, 1, 2, 3, 4, 5, 6, 7,
                                         8, 12, 16, 20, 24, 32, 48, 64};
  static constexpr int kRecvBufSize = 500;
  static constexpr int kRecvTimeoutMs = 50;

  LibCanFD_Msg msgs[kRecvBufSize];

  while (receive_thread_running_) {
    if (!is_connected_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    const int count =
        fn_CANFD_Receive_(dev_index_, 0, msgs, kRecvBufSize, kRecvTimeoutMs);

    if (count > 0 && receive_callback_) {
      for (int i = 0; i < count; ++i) {
        const LibCanFD_Msg& msg = msgs[i];
        const int data_length = msg.DLC < 16 ? kDlcToLength[msg.DLC] : 64;

        std::vector<uint8_t> rxdata(data_length);
        memcpy(rxdata.data(), msg.Data, data_length);

        if (msg.ErrSatus != 0) {
          ++lost_frames_;
        }

        receive_callback_(msg.ID, rxdata, msg.TimeStamp);
      }
    } else if (count < 0) {
      ++lost_frames_;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
#endif  // USE_LIBCANBUS

bool CANFDMaster::isConnected() const { return is_connected_; }

int CANFDMaster::getCurrentDeviceIndex() const { return dev_index_; }

uint64_t CANFDMaster::getLostFrames() const { return lost_frames_; }

void CANFDMaster::resetLostFrames() { lost_frames_ = 0; }

#endif  // _WIN32
