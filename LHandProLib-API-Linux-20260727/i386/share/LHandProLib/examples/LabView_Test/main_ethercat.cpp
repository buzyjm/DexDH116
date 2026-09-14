// ============================================================================
// 文件说明:
// File overview:
// 该示例演示如何通过 LHandProLib_LV.dll 的 C 接口连接、初始化并控制单个
// 机械手节点。此 DLL 内部已封装 EtherCAT 通信细节，测试程序无需处理回调。
//
// 主要流程包括:
// Main flow includes:
// 1. 输入 EtherCAT 网卡索引
// 2. 调用 lv_init 初始化（内部完成网卡打开、SDK 创建、PDO 线程启动）
// 3. 读取自由度，完成使能与回零
// 4. 执行循环运动示例
// 5. 调用 lv_close 清理资源
// ============================================================================

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "LHandProLib_LV.h"

#ifdef _WIN32
#include <conio.h>
#else
#include <unistd.h>
#endif

namespace {

// 当前示例默认连接的 EtherCAT 节点号。
constexpr int kNodeId = 1;

// 使能后等待设备进入稳定状态的时间，单位为毫秒。
constexpr int kEnableWaitMs = 1000;

// 回零命令下发后等待回零完成的时间，单位为毫秒。
constexpr int kHomeWaitMs = 5000;

// 每组测试位姿之间的停顿时间，单位为毫秒。
constexpr int kMotionStepDelayMs = 1000;

// 位置模式示例中使用的默认目标速度。
constexpr int kPositionVelocity = 20000;

// 位置模式示例中使用的默认最大电流限制。
constexpr int kMaxCurrent = 1000;

// 仅在 Windows 控制台中检测 Esc，便于快速停止示例循环。
bool is_escape_pressed() {
#ifdef _WIN32
  if (_kbhit()) {
    return _getch() == 27;
  }
#endif
  return false;
}

// 退出前等待用户确认，便于查看终端输出信息。
void wait_before_exit() {
#ifdef _WIN32
  system("pause");
#else
  std::cin.get();
#endif
}

// 读取一个指定范围内的数字输入。
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
void print_positions(int line_index, const std::vector<int>& positions,
                     int return_code) {
  std::cout << "line: " << line_index << " positions: ";
  for (int value : positions) {
    std::cout << value << " ";
  }
  std::cout << "retn: " << return_code << std::endl;
}

// 为当前节点批量下发本步运动参数，包括目标位置、速度和电流限制。
void configure_motion(const std::vector<int>& positions, int dof_active) {
  for (int axis = 0; axis < dof_active; ++axis) {
    const int joint_id = axis + 1;
    lv_set_target_position(joint_id, positions[axis]);
    lv_set_position_velocity(joint_id, kPositionVelocity);
    lv_set_max_current(joint_id, kMaxCurrent);
  }
}

}  // namespace

int main() {
  int return_code = 0;

  // 运动测试使用的目标位置表。
  const std::vector<std::vector<int>> positions = {
      {10000, 0, 0, 0, 0, 0}, {0, 10000, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
      {0, 0, 10000, 0, 0, 0}, {0, 0, 0, 10000, 0, 0}, {0, 0, 0, 0, 10000, 0},
      {0, 0, 0, 0, 0, 10000},
  };

  // ---------------------------------------------------------------------------
  // 1. 扫描并选择 EtherCAT 网卡
  // ---------------------------------------------------------------------------
  std::cout << "Using LHandProLib_LV.dll (EtherCAT communication)."
            << std::endl
            << std::endl;

  std::cout << "Scanning EtherCAT network interfaces..." << std::endl;
  int dev_count = lv_ethercat_scan_device_count();
  if (dev_count <= 0) {
    std::cerr << "No EtherCAT network interface found." << std::endl;
    wait_before_exit();
    return -1;
  }

  std::cout << "Found " << dev_count << " interface(s):" << std::endl;
  for (int i = 0; i < dev_count; ++i) {
    char info[256] = {0};
    if (lv_ethercat_get_device_string(i, info, sizeof(info)) == 0) {
      std::cout << "  [" << i << "] " << info << std::endl;
    }
  }

  const int device_index = select_number_in_range(
      "Please select the EtherCAT interface index", 0, dev_count - 1);

  // ---------------------------------------------------------------------------
  // 2. 调用 lv_init 初始化
  //    内部完成：打开网卡、创建 SDK 实例、注册回调、启动 PDO 线程
  // ---------------------------------------------------------------------------
  std::cout << "Initializing with lv_init(EtherCAT, device=" << device_index
            << ", node=" << kNodeId << ")..." << std::endl;

  int init_ret = lv_init(LV_COMM_ETHERCAT, device_index, kNodeId);
  if (init_ret != 0) {
    std::cerr << "lv_init failed, return code: " << init_ret << std::endl;
    wait_before_exit();
    return init_ret;
  }
  std::cout << "lv_init succeeded." << std::endl;

  // ---------------------------------------------------------------------------
  // 3. 读取节点能力，完成使能，然后执行回零
  // ---------------------------------------------------------------------------
  int dof_total = 0;
  int dof_active = 0;
  lv_get_dof(&dof_total, &dof_active);
  std::cout << "DOF: total " << dof_total << ", active " << dof_active
            << std::endl;

  lv_set_enable(0, 1);

  std::cout << "Waiting for enable..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(kEnableWaitMs));

  std::cout << "Homing node..." << std::endl;
  lv_home_motors(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(kHomeWaitMs));

  // ---------------------------------------------------------------------------
  // 4. 运动测试
  // ---------------------------------------------------------------------------
  std::cout << "Starting cyclic motion test..." << std::endl;
  std::cout << "Press Esc to exit." << std::endl;

  while (true) {
    for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
      if (is_escape_pressed()) {
        std::cout << "Esc pressed, exiting..." << std::endl;
        goto Cleanup;
      }

      configure_motion(positions[i], dof_active);

      const int motion_result = lv_move_motors(0);
      print_positions(i, positions[i], motion_result);

      if (motion_result != 0) {
        return_code = motion_result;
        goto Cleanup;
      }

      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMotionStepDelayMs));
    }
  }

Cleanup:
  // ---------------------------------------------------------------------------
  // 5. 调用 lv_close 清理资源
  // ---------------------------------------------------------------------------
  std::cout << "Calling lv_close..." << std::endl;
  lv_close();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cout << "Test completed." << std::endl;
  wait_before_exit();
  return return_code;
}
