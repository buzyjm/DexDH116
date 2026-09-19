// ============================================================================
// 文件说明:
// 该示例演示如何通过 C++ SDK 在 CANFD 主站下连接、初始化并控制单个机械手节点。
// 连接后选择 node id (默认 1) 和测试模式 (0=电机测试, 1=传感器测试)。
// 传感器测试时不需要使能电机。
// ============================================================================

#include <algorithm>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "CANFDMaster.h"
#ifndef SDK_CPP_HEADER
#define SDK_CPP_HEADER "LHandProLib.hpp"
#endif
#include SDK_CPP_HEADER

#ifndef SDK_NS
#define SDK_NS lhplib
#endif
#ifndef SDK_CLASS
#define SDK_CLASS LHandProLib
#endif
#ifdef _WIN32
#include <conio.h>
#else
#include <sys/select.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {

// 使能后等待设备进入稳定状态的时间，单位为毫秒。
constexpr int kEnableWaitMs = 1000;

// 回零命令下发后等待回零完成的时间，单位为毫秒。
constexpr int kHomeWaitMs = 5000;

// 每组测试位姿之间的停顿时间，单位为毫秒。
constexpr int kMotionStepDelayMs = 1000;

// 传感器轮询示例的循环周期，单位为毫秒。
constexpr int kSensorLoopDelayMs = 50;

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
                           int max_value, int default_value) {
  std::string line;
  while (true) {
    std::cout << prompt << " [" << min_value << " - " << max_value << "]"
              << std::endl;
    if (!std::getline(std::cin, line)) {
      std::cin.clear();
      continue;
    }
    // 空输入 → 默认值
    if (line.empty()) {
      return default_value;
    }
    try {
      int value = std::stoi(line);
      if (value >= min_value && value <= max_value) {
        return value;
      }
    } catch (...) {
    }
    std::cout << "Input out of range, please try again." << std::endl;
  }
}

// 读取一个指定范围内的数字输入（无默认值）。
int select_number_in_range(const std::string& prompt, int min_value,
                           int max_value) {
  return select_number_in_range(prompt, min_value, max_value, min_value - 1);
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
void configure_motion(SDK_NS::SDK_CLASS& lib,
                      const std::vector<int>& positions, int dof_active) {
  for (int axis = 0; axis < dof_active; ++axis) {
    const int joint_id = axis + 1;
    lib.set_target_position(joint_id, positions[axis]);
    lib.set_position_velocity(joint_id, kPositionVelocity);
    lib.set_max_current(joint_id, kMaxCurrent);
  }
}

// 读取 pressure 数组
std::vector<float> read_pressure(SDK_NS::SDK_CLASS& lib, int sensor_id) {
  int count = 0;
  lib.get_finger_pressure(sensor_id, nullptr, &count);
  if (count <= 0)
    return {};
  std::vector<float> data(count, 0.0f);
  lib.get_finger_pressure(sensor_id, data.data(), &count);
  return data;
}

// 读取法向力数组
std::vector<float> read_normal_force(SDK_NS::SDK_CLASS& lib, int sensor_id) {
  int count = 0;
  lib.get_finger_normal_force_ex(sensor_id, nullptr, &count);
  if (count <= 0)
    return {};
  std::vector<float> data(count, 0.0f);
  lib.get_finger_normal_force_ex(sensor_id, data.data(), &count);
  return data;
}

// 读取切向力数组
std::vector<float> read_tangential_force(SDK_NS::SDK_CLASS& lib,
                                         int sensor_id) {
  int count = 0;
  lib.get_finger_tangential_force_ex(sensor_id, nullptr, &count);
  if (count <= 0)
    return {};
  std::vector<float> data(count, 0.0f);
  lib.get_finger_tangential_force_ex(sensor_id, data.data(), &count);
  return data;
}

// 读取方向数组
std::vector<float> read_force_direction(SDK_NS::SDK_CLASS& lib,
                                        int sensor_id) {
  int count = 0;
  lib.get_finger_force_direction_ex(sensor_id, nullptr, &count);
  if (count <= 0)
    return {};
  std::vector<float> data(count, 0.0f);
  lib.get_finger_force_direction_ex(sensor_id, data.data(), &count);
  return data;
}

// 打印单个传感器的所有数据（参照 LHandProLib_EtherCAT_Test_Sensor 格式）
void print_sensor_data(SDK_NS::SDK_CLASS& lib, int sensor_id,
                       bool pressure_only) {
  auto pressure = read_pressure(lib, sensor_id);

  // \033[2K 清除当前行，防止残留字符
  std::cout << "\033[2K  [Sensor " << std::setw(2) << sensor_id << "] ";

  // pressure
  std::cout << "pressure[";
  for (size_t i = 0; i < pressure.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(1) << pressure[i];
  }
  std::cout << "]";

  // DH116S: 只打印 pressure，不打印力字段
  if (pressure_only) {
    std::cout << std::endl;
    return;
  }

  auto nf = read_normal_force(lib, sensor_id);
  auto tf = read_tangential_force(lib, sensor_id);
  auto dir = read_force_direction(lib, sensor_id);

  // normal force
  std::cout << " NF[";
  for (size_t i = 0; i < nf.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(2) << nf[i];
  }
  std::cout << "]";

  // tangential force
  std::cout << " TF[";
  for (size_t i = 0; i < tf.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(2) << tf[i];
  }
  std::cout << "]";

  // force direction
  std::cout << " DIR[";
  for (size_t i = 0; i < dir.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(1) << dir[i];
  }
  std::cout << "]" << std::endl;
}

}  // namespace

int main() {
  auto sdk_handle = std::make_shared<SDK_NS::SDK_CLASS>();
  auto canfd_master = std::make_shared<CANFDMaster>();
  int return_code = 0;

  // 运动测试使用的目标位置表。
  const std::vector<std::vector<int>> positions = {
      {10000, 0, 0, 0, 0, 0}, {0, 10000, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
      {0, 0, 10000, 0, 0, 0}, {0, 0, 0, 10000, 0, 0}, {0, 0, 0, 0, 10000, 0},
      {0, 0, 0, 0, 0, 10000},
  };

  // 传感器示例使用的手指传感器编号列表。
  const std::vector<int> finger_sensor_id_list = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1, SDK_NS::LSS_FINGER_3_1,
      SDK_NS::LSS_FINGER_4_1, SDK_NS::LSS_FINGER_5_1};
  const int palm_sensor_id = SDK_NS::LSS_HAND_PALM;
  // 所有传感器列表（手指 + 掌心）
  const std::vector<int> all_sensor_list = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1, SDK_NS::LSS_FINGER_3_1,
      SDK_NS::LSS_FINGER_4_1, SDK_NS::LSS_FINGER_5_1, SDK_NS::LSS_HAND_PALM};

  // ---------------------------------------------------------------------------
  // 1. 扫描 CANFD 通道并选择目标通道
  // ---------------------------------------------------------------------------
  std::cout << "Using CANFD communication." << std::endl << std::endl;

  const std::vector<std::string> channel_names = canfd_master->scanDevices();
  std::cout << "Detected CANFD channels: " << channel_names.size() << std::endl
            << std::endl;

  if (channel_names.empty()) {
    wait_before_exit();
    return 0;
  }

  for (size_t i = 0; i < channel_names.size(); ++i) {
    std::cout << "Input " << i << " ---- channel " << channel_names[i]
              << std::endl;
  }

  const int channel_index =
      select_number_in_range("\nPlease select the CANFD channel", 0,
                             static_cast<int>(channel_names.size()) - 1);

  if (!canfd_master->connect(channel_index)) {
    std::cout << "CANFD connect failed." << std::endl;
    wait_before_exit();
    return -1;
  }
  std::cout << "CANFD connect succeeded." << std::endl;

  // ---------------------------------------------------------------------------
  // 2. 绑定 SDK 与 CANFD 回调（连接后立即绑定，避免丢帧）
  // ---------------------------------------------------------------------------
  auto send_func = [canfd_master](unsigned int id, const unsigned char* data,
                                  unsigned int size, int is_extended) {
    return canfd_master->sendData(id, data, size, is_extended);
  };
  std::function<bool(unsigned int, const unsigned char*, unsigned int, int)>
      func = send_func;
  sdk_handle->set_send_canfd_callback_ex(&func);

  canfd_master->setReceiveCallback([sdk_handle](uint32_t id,
                                             const std::vector<uint8_t>& data,
                                             uint64_t timestamp) {
    (void)timestamp;
    sdk_handle->set_canfd_data_decode(id, data.data(), data.size());
  });

  // ---------------------------------------------------------------------------
  // 3. 选择 node id（默认 1）和测试模式
  // ---------------------------------------------------------------------------
  const int node_id =
      select_number_in_range("\nPlease enter CANFD node id (default 1)", 1, 255, 1);

  const int sensor_mode =
      select_number_in_range("\nSelect test mode\n"
                             "  0 = Motor motion test\n"
                             "  1 = Sensor data test\n"
                             "(default 0)",
                             0, 1, 0);

  // ---------------------------------------------------------------------------
  // 4. 初始化 SDK
  // ---------------------------------------------------------------------------
  int dof_total = 0;
  int dof_active = 0;

  if (sdk_handle->initial(SDK_NS::LCN_CANFD, node_id) != SDK_NS::LER_NONE) {
    std::cerr << "LHandProLib init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  // ---------------------------------------------------------------------------
  // 5. 主逻辑（块作用域内，避免 goto Cleanup 跳过变量初始化）
  // ---------------------------------------------------------------------------
  {
  int hand_type = SDK_NS::LAC_DOF_6;
  sdk_handle->get_hand_type(&hand_type);

  sdk_handle->get_dof(&dof_total, &dof_active);
  std::cout << "DOF: total " << dof_total << ", active " << dof_active
            << ", hand_type " << hand_type << std::endl;

  if (sensor_mode == 0) {
    sdk_handle->set_enable(0, 1);

    std::cout << "Waiting for enable..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(kEnableWaitMs));

    std::cout << "Homing node..." << std::endl;
    sdk_handle->home_motors(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(kHomeWaitMs));
  } else {
    sdk_handle->set_sensor_enable(true);
  }

  if (sensor_mode == 0) {
    std::cout << "Starting cyclic motion test..." << std::endl;
    std::cout << "Press Esc to exit." << std::endl;

    while (true) {
      for (int i = 0; i < static_cast<int>(positions.size()); ++i) {
        if (is_escape_pressed()) {
          std::cout << "Esc pressed, exiting..." << std::endl;
          goto Cleanup;
        }

        configure_motion(*sdk_handle, positions[i], dof_active);

        const int motion_result = sdk_handle->move_motors();
        print_positions(i, positions[i], motion_result);

        if (motion_result != SDK_NS::LER_NONE) {
          return_code = motion_result;
          goto Cleanup;
        }

        std::this_thread::sleep_for(
            std::chrono::milliseconds(kMotionStepDelayMs));
      }
    }
  } else {
    const bool pressure_only =
        (hand_type == SDK_NS::LAC_DOF_6_S);
    const int active_sensor_count = static_cast<int>(all_sensor_list.size());

    std::cout << "Starting sensor data test..." << std::endl;
    std::cout << "Press Esc to exit." << std::endl;

    const int total_lines = 1 + active_sensor_count;
    bool first_frame = true;
    while (true) {
      if (is_escape_pressed()) {
        std::cout << "\nEsc pressed, exiting..." << std::endl;
        break;
      }

      if (!first_frame) {
        std::cout << "\033[" << total_lines << "A";
      }
      first_frame = false;

      std::cout << "\033[2K----------------------------------------"
                << std::endl;
      for (int i = 0; i < active_sensor_count; ++i) {
        print_sensor_data(*sdk_handle, all_sensor_list[i], pressure_only);
      }
      std::cout << std::flush;

      std::this_thread::sleep_for(
          std::chrono::milliseconds(kSensorLoopDelayMs));
    }
  }
  }

Cleanup:
  // 使用统一清理路径，确保 SDK 关闭和 CANFD 断开都能一致执行。
  sdk_handle->close();
  canfd_master->disconnect();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
