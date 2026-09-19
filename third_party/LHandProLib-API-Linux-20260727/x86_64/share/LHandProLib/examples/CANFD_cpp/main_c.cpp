// ============================================================================
// 文件说明:
// File overview:
// 该示例演示如何通过 C 语言封装接口在 CANFD 主站下连接、初始化并控制单个机械手节点。
// This sample shows how to use the C wrapper API to connect, initialize, and
// control one hand node through a CANFD master.
//
// 主要流程包括:
// Main flow includes:
// 1. 扫描并选择 CANFD 通道
// 1. Scan and select a CANFD channel
// 2. 创建 C 封装句柄并绑定 CANFD 发送/接收回调
// 2. Create the C wrapper handle and bind CANFD send/receive callbacks
// 3. 初始化节点，读取自由度，并完成使能与回零
// 3. Initialize the node, read DOF information, then enable and home it
// 4. 执行循环运动示例
// 4. Run the cyclic motion example
// ============================================================================

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "CANFDMaster.h"
#ifndef SDK_C_HEADER
#define SDK_C_HEADER "LHandProLib.h"
#endif
#include SDK_C_HEADER

#ifdef _WIN32
#include <conio.h>
#else
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {

// 当前示例默认连接的 CANFD 节点号。
// Default CANFD node id used by this sample.
constexpr int kNodeId = 1;

// 使能后等待设备进入稳定状态的时间，单位为毫秒。
// Time to wait after enable so the device can reach a stable state, in ms.
constexpr int kEnableWaitMs = 1000;

// 回零命令下发后等待回零完成的时间，单位为毫秒。
// Time to wait after issuing homing, in ms.
constexpr int kHomeWaitMs = 5000;

// 每组测试位姿之间的停顿时间，单位为毫秒。
// Delay between motion steps in the test sequence, in ms.
constexpr int kMotionStepDelayMs = 1000;

// 位置模式示例中使用的默认目标速度。
// Default position velocity used in the motion example.
constexpr int kPositionVelocity = 20000;

// 位置模式示例中使用的默认最大电流限制。
// Default max current limit used in the motion example.
constexpr int kMaxCurrent = 1000;

// 全局 CANFD 主站指针，供 C 回调函数访问当前主站对象。
// Global CANFD master pointer used by the C callback to access the master.
std::shared_ptr<CANFDMaster> g_canfd_master = nullptr;

// 仅在 Windows 控制台中检测 Esc，便于快速停止示例循环。
// Only check Esc on Windows console so the sample can be stopped quickly.
bool is_escape_pressed() {
#ifdef _WIN32
  if (_kbhit()) {
    return _getch() == 27;
  }
#endif
  return false;
}

// 退出前等待用户确认，便于查看终端输出信息。
// Wait before exit so the user can read the terminal output.
void wait_before_exit() {
#ifdef _WIN32
  system("pause");
#else
  std::cin.get();
#endif
}

// 读取一个指定范围内的数字输入。
// Read a numeric input within the specified range.
int select_number_in_range(const std::string& prompt, int min_value,
                           int max_value) {
  while (true) {
    std::cout << prompt << " [" << min_value << " - " << max_value << "]"
              << std::endl;

    int value = 0;
    std::cin >> value;
    if (value < min_value || value > max_value) {
      std::cout << "Input out of range, please try again." << std::endl;
      continue;
    }

    return value;
  }
}

// 打印一组目标位置，便于观察当前步的控制目标。
// Print one position set so the current command step is easy to inspect.
void print_positions(int line_index, const std::vector<int>& positions,
                     int return_code) {
  std::cout << "line: " << line_index << " positions: ";
  for (int value : positions) {
    std::cout << value << " ";
  }
  std::cout << "retn: " << return_code << std::endl;
}

// 为当前节点批量下发本步运动参数，包括目标位置、速度和电流限制。
// Apply one motion step to the current node, including position, velocity,
// and current.
void configure_motion(SDK_C_API(_handle) handle,
                      const std::vector<int>& positions, int dof_active) {
  for (int axis = 0; axis < dof_active; ++axis) {
    const int joint_id = axis + 1;
    SDK_C_API(_set_target_position)(handle, joint_id, positions[axis]);
    SDK_C_API(_set_position_velocity)(handle, joint_id, kPositionVelocity);
    SDK_C_API(_set_max_current)(handle, joint_id, kMaxCurrent);
  }
}

// CANFD 发送回调，将 C SDK 发出的数据转发到当前 CANFD 通道。
// CANFD send callback that forwards SDK output data to the current channel.
bool canfd_send_callback(unsigned int id, const unsigned char* data,
                         unsigned int size, int is_extended) {
  if (g_canfd_master) {
    return g_canfd_master->sendData(id, data, size, is_extended);
  }

  std::cerr << "CANFD master not initialized!" << std::endl;
  return false;
}

}  // namespace

int main() {
  SDK_C_API(_handle) sdk_handle = SDK_C_API(_create)();
  if (!sdk_handle) {
    std::cerr << "Failed to create C wrapper." << std::endl;
    return -1;
  }

  g_canfd_master = std::make_shared<CANFDMaster>();
  int return_code = 0;

  // 运动测试使用的目标位置表。
  // Motion table used by the single-node motion example.
  const std::vector<std::vector<int>> positions = {
      {10000, 0, 0, 0, 0, 0},
      {0, 10000, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0},
      {0, 0, 10000, 0, 0, 0},
      {0, 0, 0, 10000, 0, 0},
      {0, 0, 0, 0, 10000, 0},
      {0, 0, 0, 0, 0, 10000},
  };

  // ---------------------------------------------------------------------------
  // 1. 扫描 CANFD 通道并选择目标通道
  // 1. Scan CANFD channels and choose the target channel.
  // ---------------------------------------------------------------------------
  std::cout << "Using CANFD communication." << std::endl << std::endl;

  const std::vector<std::string> channel_names = g_canfd_master->scanDevices();
  std::cout << "Detected CANFD channels: " << channel_names.size()
            << std::endl
            << std::endl;

  if (channel_names.empty()) {
    wait_before_exit();
    SDK_C_API(_destroy)(sdk_handle);
    return 0;
  }

  for (size_t i = 0; i < channel_names.size(); ++i) {
    std::cout << "Input " << i << " ---- channel " << channel_names[i]
              << std::endl;
  }

  const int channel_index =
      select_number_in_range("\nPlease select the CANFD channel", 0,
                             static_cast<int>(channel_names.size()) - 1);

  if (!g_canfd_master->connect(channel_index)) {
    std::cout << "CANFD connect failed." << std::endl;
    wait_before_exit();
    SDK_C_API(_destroy)(sdk_handle);
    return -1;
  }
  std::cout << "CANFD connect succeeded." << std::endl;

  // ---------------------------------------------------------------------------
  // 2. 绑定 C SDK 与 CANFD 回调
  // 2. Bind the C SDK handle and CANFD callbacks.
  // ---------------------------------------------------------------------------
  SDK_C_API(_set_send_canfd_callback)(sdk_handle, canfd_send_callback);

  g_canfd_master->setReceiveCallback(
      [sdk_handle](uint32_t id, const std::vector<uint8_t>& data,
                   uint64_t timestamp) {
        (void)timestamp;
        SDK_C_API(_set_canfd_data_decode)(sdk_handle, id, data.data(),
                                          data.size());
      });

  int dof_total = 0;
  int dof_active = 0;

  if (SDK_C_API(_initial_ex)(sdk_handle, C_LCN_CANFD, kNodeId) != C_LER_NONE) {
    std::cerr << "LHandProLib C wrapper init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  // ---------------------------------------------------------------------------
  // 3. 读取节点能力，完成使能，然后执行回零
  // 3. Read node capability, enable the device, then home it.
  // ---------------------------------------------------------------------------
  SDK_C_API(_get_dof)(sdk_handle, &dof_total, &dof_active);
  std::cout << "DOF: total " << dof_total << ", active " << dof_active
            << std::endl;

  SDK_C_API(_set_enable)(sdk_handle, 0, 1);

  std::cout << "Waiting for enable..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(kEnableWaitMs));

  std::cout << "Homing node..." << std::endl;
  SDK_C_API(_home_motors)(sdk_handle, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(kHomeWaitMs));

  // ---------------------------------------------------------------------------
  // 4. 运动测试
  // 4. Single-node motion example.
  // ---------------------------------------------------------------------------
  std::cout << "Starting cyclic motion test..." << std::endl;

  while (true) {
    for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
      if (is_escape_pressed()) {
        std::cout << "Esc pressed, exiting..." << std::endl;
        goto Cleanup;
      }

      configure_motion(sdk_handle, positions[i], dof_active);

      const int motion_result = SDK_C_API(_move_motors)(sdk_handle, 0);
      print_positions(i, positions[i], motion_result);

      if (motion_result != C_LER_NONE) {
        return_code = motion_result;
        goto Cleanup;
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMotionStepDelayMs));
    }
  }

Cleanup:
  // 使用统一清理路径，确保 SDK 关闭和 CANFD 断开都能一致执行。
  // Use a single cleanup path so SDK close and CANFD disconnect happen
  // consistently on normal exit and on early error.
  SDK_C_API(_close)(sdk_handle);
  SDK_C_API(_destroy)(sdk_handle);
  if (g_canfd_master) {
    g_canfd_master->disconnect();
    g_canfd_master.reset();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
