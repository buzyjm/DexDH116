// ============================================================================
// 文件说明:
// File overview:
// 该示例演示如何通过 C 语言封装接口，在 EtherCAT 主站下连接、初始化并控制单个从站。
// This sample shows how to use the C wrapper API to connect, initialize, and
// control one EtherCAT slave under a single EtherCAT master.
//
// 主要流程包括:
// Main flow includes:
// 1. 扫描并选择网口
// 1. Scan and select a network adapter
// 2. 扫描并选择要控制的从站
// 2. Scan and select the target slave
// 3. 创建 C 封装句柄并绑定 EtherCAT 发送回调
// 3. Create the C wrapper handle and bind the EtherCAT send callback
// 4. 后台循环接收并解析当前从站的 TPDO 数据
// 4. Receive and decode TPDO data for the selected slave in a background loop
// 5. 执行单从站运动测试
// 5. Run the single-slave motion test
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include "EthercatMaster.h"
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

// 使能后等待设备进入稳定状态的时间，单位为毫秒。
// Time to wait after enable so the device can reach a stable state, in ms.
constexpr int kEnableWaitMs = 1000;

// 回零命令下发后等待回零完成的时间，单位为毫秒。
// Time to wait after issuing homing, in ms.
constexpr int kHomeWaitMs = 5000;

// 监控线程启动后的首次等待时间，给 EtherCAT 循环和设备初始化留出缓冲。
// Initial delay before the monitor thread starts polling TPDO data, in ms.
constexpr int kMonitorStartupDelayMs = 1000;

// 监控线程轮询当前从站 TPDO 数据的周期，单位为毫秒。
// Polling period of the monitor thread when reading TPDO data, in ms.
constexpr int kMonitorLoopDelayMs = 10;

// 每组测试位姿之间的停顿时间，单位为毫秒。
// Delay between motion steps in the test sequence, in ms.
constexpr int kMotionStepDelayMs = 1000;

// 位置模式示例中使用的默认目标速度。
// Default position velocity used in the motion example.
constexpr int kPositionVelocity = 20000;

// 位置模式示例中使用的默认最大电流限制。
// Default max current limit used in the motion example.
constexpr int kMaxCurrent = 1000;

// 全局 EtherCAT 主站指针，供 C 回调函数访问当前主站对象。
// Global EtherCAT master pointer used by the C callback to access the master.
std::shared_ptr<EthercatMaster> g_ec_master = nullptr;

// 当前选中的从站编号，供 C 回调函数写入目标从站输出。
// Currently selected slave id used by the C callback when sending outputs.
int g_slave_id = 1;

// 仅在 Windows 控制台中检测 Esc，便于快速停止循环运动示例。
// Only check Esc on Windows console so the motion loop can be stopped quickly.
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

// 为当前从站批量下发本步运动参数，包括目标位置、速度和电流限制。
// Apply one motion step to the selected slave, including position, velocity,
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

// 从当前从站读取一帧 TPDO 输入数据，并交给 C SDK 进行解析。
// Read one TPDO input frame from the selected slave and decode it in the C SDK.
void refresh_tpdo(EthercatMaster& master, int slave_id,
                  SDK_C_API(_handle) handle) {
  const int input_size = master.getSlaveInputSize(slave_id);
  if (input_size <= 0) {
    return;
  }

  std::vector<uint8_t> input_buffer(input_size);
  if (master.getSlaveInputs(slave_id, input_buffer.data(), input_size)) {
    SDK_C_API(_set_tpdo_data_decode)(handle, input_buffer.data(), input_size);
  }
}

// EtherCAT 发送回调，将 C SDK 发出的 RPDO 数据转发到当前从站。
// EtherCAT send callback that forwards RPDO data from the C SDK to the slave.
bool ec_send_callback(const unsigned char* data, unsigned int size) {
  if (g_ec_master) {
    return g_ec_master->setSlaveOutputs(g_slave_id, data, size);
  }

  std::cerr << "EC master not initialized!" << std::endl;
  return false;
}

}  // namespace

int main() {
  SDK_C_API(_handle) sdk_handle = SDK_C_API(_create)();
  if (!sdk_handle) {
    std::cerr << "Failed to create C wrapper." << std::endl;
    return -1;
  }

  g_ec_master = std::make_shared<EthercatMaster>();

  std::atomic<bool> stop_monitor{false};
  std::thread monitor_thread;
  int return_code = 0;

  // 运动测试使用的目标位置表。
  // Motion table used by the single-slave motion example.
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
  // 1. 初始化 EtherCAT 主站并选择网口
  // 1. Initialize EtherCAT master and let the user choose a network adapter.
  // ---------------------------------------------------------------------------
  std::cout << "Using EtherCAT communication (100M)." << std::endl
            << std::endl;

  const std::vector<std::string> interface_names =
      g_ec_master->scanNetworkInterfaces();
  std::cout << "Detected network adapters: " << interface_names.size()
            << std::endl
            << std::endl;

  if (interface_names.empty()) {
    wait_before_exit();
    SDK_C_API(_destroy)(sdk_handle);
    return 0;
  }

  for (size_t i = 0; i < interface_names.size(); ++i) {
    std::cout << "Input " << i << " ---- adapter " << interface_names[i]
              << std::endl;
  }

  const int channel_index = select_number_in_range(
      "\nPlease select the EtherCAT adapter", 0,
      static_cast<int>(interface_names.size()) - 1);

  if (!g_ec_master->init(channel_index)) {
    std::cout << "EtherCAT init failed." << std::endl;
    wait_before_exit();
    SDK_C_API(_destroy)(sdk_handle);
    return -1;
  }

  std::cout << "EtherCAT init succeeded." << std::endl;

  if (!g_ec_master->start()) {
    std::cout << "Failed to start EtherCAT master." << std::endl;
    SDK_C_API(_destroy)(sdk_handle);
    return -1;
  }

  // ---------------------------------------------------------------------------
  // 2. 扫描从站并选择目标从站
  // 2. Scan slaves and choose the target slave.
  // ---------------------------------------------------------------------------
  const int slave_count = g_ec_master->getSlaveCount();
  std::cout << "\nDetected slave count: " << slave_count << std::endl;

  const auto slave_list = g_ec_master->getSlaveInfoList();
  for (const auto& slave : slave_list) {
    std::cout << "  Slave " << slave.index << " : " << slave.name
              << std::endl;
  }

  if (slave_count < 1) {
    std::cerr << "No EtherCAT slave was found." << std::endl;
    SDK_C_API(_destroy)(sdk_handle);
    return -1;
  }

  g_slave_id = 1;
  if (slave_count > 1) {
    g_slave_id = select_number_in_range("\nPlease select the target slave id",
                                        1, slave_count);
  }
  std::cout << "Selected slave: " << g_slave_id << std::endl;

  // 启动 EtherCAT 后台交换循环，后续 C SDK 的发送和接收都依赖这条循环。
  // Start the EtherCAT background exchange loop. The C SDK send/receive path
  // below depends on this loop to keep PDO data flowing.
  g_ec_master->run();

  // ---------------------------------------------------------------------------
  // 3. 绑定 C SDK 句柄到当前从站
  // 3. Bind the C SDK handle to the selected slave.
  // ---------------------------------------------------------------------------
  SDK_C_API(_set_send_rpdo_callback)(sdk_handle, ec_send_callback);

  // 监控线程持续从当前从站获取 TPDO 数据，并交给 C SDK 解析。
  // This monitor thread continuously fetches TPDO data from the selected slave
  // and forwards it to the C SDK for decoding.
  monitor_thread = std::thread([&stop_monitor, sdk_handle]() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kMonitorStartupDelayMs));

    while (!stop_monitor) {
      refresh_tpdo(*g_ec_master, g_slave_id, sdk_handle);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMonitorLoopDelayMs));
    }
  });

  int dof_total = 0;
  int dof_active = 0;

  if (SDK_C_API(_initial)(sdk_handle, C_LCN_ECAT) != C_LER_NONE) {
    std::cerr << "LHandProLib C wrapper init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  // 从这里开始，sdk_handle 就代表当前选中的 EtherCAT 从站。
  // From here on, sdk_handle represents the selected EtherCAT slave.

  // ---------------------------------------------------------------------------
  // 4. 读取从站能力，完成使能，然后执行回零
  // 4. Read slave capability, enable the device, then home it.
  // ---------------------------------------------------------------------------
  SDK_C_API(_get_dof)(sdk_handle, &dof_total, &dof_active);
  std::cout << "DOF: total " << dof_total << ", active " << dof_active
            << std::endl;

  SDK_C_API(_set_enable)(sdk_handle, 0, 1);

  std::cout << "Waiting for enable..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(kEnableWaitMs));

  std::cout << "Homing slave..." << std::endl;
  SDK_C_API(_home_motors)(sdk_handle, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(kHomeWaitMs));

  // ---------------------------------------------------------------------------
  // 5. 运动测试
  // 5. Single-slave motion example.
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
  // 使用统一清理路径，确保线程退出、SDK 关闭和主站停止都能一致执行。
  // Use a single cleanup path so thread shutdown and cleanup happen
  // consistently on normal exit and on early error.
  stop_monitor = true;
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }

  SDK_C_API(_close)(sdk_handle);
  SDK_C_API(_destroy)(sdk_handle);
  if (g_ec_master) {
    g_ec_master->stop();
    g_ec_master.reset();
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
