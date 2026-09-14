// ============================================================================
// 文件说明:
// 该示例演示如何通过 C++ SDK 在 RS485 串口下连接、初始化并控制单个机械手节点。
// 连接串口后选择测试模式 (0=电机测试, 1=传感器测试)。
// 传感器测试时不需要使能电机。
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

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
#include "SerialPort.h"

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

// 打印单个传感器的所有数据
void print_sensor_data(SDK_NS::SDK_CLASS& lib, int sensor_id,
                       bool pressure_only) {
  auto pressure = read_pressure(lib, sensor_id);

  std::cout << "\033[2K  [Sensor " << std::setw(2) << sensor_id << "] ";

  // pressure
  std::cout << "pressure[";
  for (size_t i = 0; i < pressure.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(1) << pressure[i];
  }
  std::cout << "]";

  if (pressure_only) {
    std::cout << std::endl;
    return;
  }

  auto nf = read_normal_force(lib, sensor_id);
  auto tf = read_tangential_force(lib, sensor_id);
  auto dir = read_force_direction(lib, sensor_id);

  std::cout << " NF[";
  for (size_t i = 0; i < nf.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(2) << nf[i];
  }
  std::cout << "]";

  std::cout << " TF[";
  for (size_t i = 0; i < tf.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(2) << tf[i];
  }
  std::cout << "]";

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
  auto serial_port = std::make_shared<SerialPort>();

  std::atomic<bool> stop_receive{false};
  int return_code = 0;

  // 运动测试使用的目标位置表。
  const std::vector<std::vector<int>> positions = {
      {10000, 0, 0, 0, 0, 0},
      {0, 10000, 0, 0, 0, 0},
      {0, 0, 0, 0, 0, 0},
      {0, 0, 10000, 0, 0, 0},
      {0, 0, 0, 10000, 0, 0},
      {0, 0, 0, 0, 10000, 0},
      {0, 0, 0, 0, 0, 10000},
  };

  // 所有传感器列表（手指 + 掌心）
  const std::vector<int> all_sensor_list = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1, SDK_NS::LSS_FINGER_3_1,
      SDK_NS::LSS_FINGER_4_1, SDK_NS::LSS_FINGER_5_1, SDK_NS::LSS_HAND_PALM};

  // ---------------------------------------------------------------------------
  // 1. 扫描可用串口并选择目标串口
  // ---------------------------------------------------------------------------
  std::cout << "Using RS485 communication." << std::endl << std::endl;

  const std::vector<std::string> port_names = SerialPort::scanAvailablePorts();
  std::cout << "Detected serial ports: " << port_names.size() << std::endl
            << std::endl;

  if (port_names.empty()) {
    wait_before_exit();
    return 0;
  }

  for (size_t i = 0; i < port_names.size(); ++i) {
    std::cout << "Input " << i << " ---- port " << port_names[i]
              << std::endl;
  }

  const int port_index =
      select_number_in_range("\nPlease select the serial port", 0,
                             static_cast<int>(port_names.size()) - 1);
  const std::string selected_port = port_names[port_index];

  std::cout << "Opening serial port " << selected_port << " ..." << std::endl;

  if (!serial_port->open(selected_port)) {
    std::cout << "Failed to open serial port." << std::endl;
    wait_before_exit();
    return -1;
  }
  std::cout << "Serial port opened successfully." << std::endl;

  // ---------------------------------------------------------------------------
  // 2. 选择测试模式 (默认 0=电机测试)
  // ---------------------------------------------------------------------------
  const int sensor_mode =
      select_number_in_range("\nSelect test mode\n"
                             "  0 = Motor motion test\n"
                             "  1 = Sensor data test\n"
                             "(default 0)",
                             0, 1, 0);

  // ---------------------------------------------------------------------------
  // 3. 绑定 SDK 与 RS485 回调
  // ---------------------------------------------------------------------------
  auto send_func = [serial_port](const unsigned char* data, unsigned int size) {
    return serial_port->write(data, size);
  };
  std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  sdk_handle->set_send_rs485_callback_ex(&func);

  serial_port->setReadCallback(
      [sdk_handle, &stop_receive](const uint8_t* data, size_t size) {
        if (stop_receive) {
          return;
        }
        sdk_handle->set_rs485_data_decode(data, static_cast<int>(size));
      });

  if (sdk_handle->initial(SDK_NS::LCN_RS485) != SDK_NS::LER_NONE) {
    std::cerr << "LHandProLib init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }
  std::cout << "LHandProLib init succeeded." << std::endl;

  // ---------------------------------------------------------------------------
  // 4. 主逻辑（块作用域内，避免 goto Cleanup 跳过变量初始化）
  // ---------------------------------------------------------------------------
  {
  int dof_total = 0;
  int dof_active = 0;
  sdk_handle->get_dof(&dof_total, &dof_active);

  int hand_type = SDK_NS::LAC_DOF_6;
  sdk_handle->get_hand_type(&hand_type);

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
    const bool pressure_only = (hand_type == SDK_NS::LAC_DOF_6_S);
    const int active_sensor_count = static_cast<int>(all_sensor_list.size());
    const int total_lines = 1 + active_sensor_count;

    std::cout << "Starting sensor data test..." << std::endl;
    std::cout << "Press Esc to exit." << std::endl;

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
  stop_receive = true;
  sdk_handle->close();
  serial_port->close();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
