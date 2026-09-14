// ============================================================================
// 文件说明:
// 该示例演示在 EtherCAT 模式下测试传感器数据读取。
// 启动时可选择 format=0（默认）或 format=1（channel，7传感器，顺序 1 3 4 5 7 9 11）。
// 循环打印所有传感器的 pressure、法向力、切向力、方向数据。
// ============================================================================

#include <atomic>
#include <chrono>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>
#include <vector>

#include "EthercatMaster.h"
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
#include <unistd.h>
#endif

namespace {

constexpr int kMonitorStartupDelayMs = 1000;
constexpr int kMonitorLoopDelayMs = 10;
constexpr int kSensorLoopDelayMs = 50;

bool is_escape_pressed() {
#ifdef _WIN32
  if (_kbhit()) {
    return _getch() == 27;
  }
#endif
  return false;
}

void wait_before_exit() {
#ifdef _WIN32
  system("pause");
#else
  std::cin.get();
#endif
}

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

void refresh_tpdo(EthercatMaster& master, int slave_id,
                  SDK_NS::SDK_CLASS& lib) {
  const int input_size = master.getSlaveInputSize(slave_id);
  if (input_size <= 0)
    return;

  std::vector<uint8_t> input_buffer(input_size);
  if (master.getSlaveInputs(slave_id, input_buffer.data(), input_size)) {
    lib.set_tpdo_data_decode(input_buffer.data(), input_size);
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
void print_sensor_data(SDK_NS::SDK_CLASS& lib, int sensor_id) {
  auto pressure = read_pressure(lib, sensor_id);
  auto nf = read_normal_force(lib, sensor_id);
  auto tf = read_tangential_force(lib, sensor_id);
  auto dir = read_force_direction(lib, sensor_id);

  // \033[2K 清除当前行，防止残留字符
  std::cout << "\033[2K  [Sensor " << std::setw(2) << sensor_id << "] ";

  // pressure
  std::cout << "pressure[";
  for (size_t i = 0; i < pressure.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(1) << pressure[i];
  }
  std::cout << "] ";

  // normal force
  std::cout << "NF[";
  for (size_t i = 0; i < nf.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(2) << nf[i];
  }
  std::cout << "] ";

  // tangential force
  std::cout << "TF[";
  for (size_t i = 0; i < tf.size(); ++i) {
    if (i > 0)
      std::cout << ",";
    std::cout << std::fixed << std::setprecision(2) << tf[i];
  }
  std::cout << "] ";

  // force direction
  std::cout << "DIR[";
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
  auto ec_master = std::make_shared<EthercatMaster>();

  std::atomic<bool> stop_monitor{false};
  std::thread monitor_thread;
  int return_code = 0;

  // format=1 时的传感器顺序
  const int sensor_order_fmt1[] = {
    SDK_NS::LSS_FINGER_2_1, SDK_NS::LSS_FINGER_3_1, SDK_NS::LSS_FINGER_2_2,
    SDK_NS::LSS_HAND_PALM, SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_4_1,
    SDK_NS::LSS_FINGER_5_1};
  constexpr int sensor_count_fmt1 = 7;

  // ---------------------------------------------------------------------------
  // 1. 初始化 EtherCAT 主站并选择网口
  // ---------------------------------------------------------------------------
  std::cout << "=== Sensor Test ===" << std::endl << std::endl;

  const std::vector<std::string> interface_names =
      ec_master->scanNetworkInterfaces();
  if (interface_names.empty()) {
    std::cout << "No network adapter found." << std::endl;
    wait_before_exit();
    return 0;
  }

  for (size_t i = 0; i < interface_names.size(); ++i) {
    std::cout << "Input " << i << " ---- adapter " << interface_names[i]
              << std::endl;
  }

  const int channel_index =
      select_number_in_range("\nPlease select the EtherCAT adapter", 0,
                             static_cast<int>(interface_names.size()) - 1);

  if (!ec_master->init(channel_index)) {
    std::cout << "EtherCAT init failed." << std::endl;
    wait_before_exit();
    return -1;
  }

  std::cout << "EtherCAT init succeeded." << std::endl;

  if (!ec_master->start()) {
    std::cout << "Failed to start EtherCAT master." << std::endl;
    return -1;
  }

  // ---------------------------------------------------------------------------
  // 2. 扫描从站并选择目标从站
  // ---------------------------------------------------------------------------
  const int slave_count = ec_master->getSlaveCount();
  std::cout << "\nDetected slave count: " << slave_count << std::endl;

  const auto slave_list = ec_master->getSlaveInfoList();
  for (const auto& slave : slave_list) {
    std::cout << "  Slave " << slave.index << " : " << slave.name << std::endl;
  }

  if (slave_count < 1) {
    std::cerr << "No EtherCAT slave was found." << std::endl;
    return -1;
  }

  int slave_id = 1;
  if (slave_count > 1) {
    slave_id = select_number_in_range("\nPlease select the target slave id", 1,
                                      slave_count);
  }
  std::cout << "Selected slave: " << slave_id << std::endl;

  ec_master->run();

  // ---------------------------------------------------------------------------
  // 3. 绑定 LHandProLib 实例到当前从站
  // ---------------------------------------------------------------------------
  auto send_func = [ec_master, slave_id](const unsigned char* data,
                                         unsigned int size) {
    return ec_master->setSlaveOutputs(slave_id, data, size);
  };

  std::function<bool(const unsigned char*, unsigned int)> func = send_func;
  sdk_handle->set_send_rpdo_callback_ex(&func);

  monitor_thread = std::thread([=, &stop_monitor]() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kMonitorStartupDelayMs));
    while (!stop_monitor) {
      refresh_tpdo(*ec_master, slave_id, *sdk_handle);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMonitorLoopDelayMs));
    }
  });

  // ---------------------------------------------------------------------------
  // 4. 初始化 SDK
  // ---------------------------------------------------------------------------
  if (sdk_handle->initial(SDK_NS::LCN_ECAT) != SDK_NS::LER_NONE) {
    std::cerr << "LHandProLib init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  // ---------------------------------------------------------------------------
  // 5. 配置传感器 + 循环打印（独立块作用域，避免 goto Cleanup 跳过初始化）
  // ---------------------------------------------------------------------------
  {
  int sensor_format = 0;
  std::cout << "\nSelect sensor format [0/1] (default 0): ";
  std::cin >> sensor_format;
  if (sensor_format != 0 && sensor_format != 1)
    sensor_format = 0;

  if (sensor_format == 1) {
    sdk_handle->set_sensor_data_format(1);
    sdk_handle->set_sensor_order(sensor_order_fmt1, sensor_count_fmt1);
    std::cout << "Sensor format=1 (channel)" << std::endl;
  } else {
    std::cout << "Sensor format=0 (default)" << std::endl;
  }
  sdk_handle->set_sensor_enable(1);
  std::cout << "Press Esc to exit..." << std::endl << std::endl;

  // 打印的传感器列表（固定顺序，与 set_sensor_order 无关）
  static const int print_sensor_list_fmt0[] = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1,
      SDK_NS::LSS_FINGER_3_1, SDK_NS::LSS_FINGER_4_1,
      SDK_NS::LSS_FINGER_5_1, SDK_NS::LSS_HAND_PALM};
  static const int print_sensor_list_fmt1[] = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1,
      SDK_NS::LSS_FINGER_2_2, SDK_NS::LSS_FINGER_3_1,
      SDK_NS::LSS_FINGER_4_1, SDK_NS::LSS_FINGER_5_1,
      SDK_NS::LSS_HAND_PALM};
  const int* active_sensor_list =
      (sensor_format == 1) ? print_sensor_list_fmt1 : print_sensor_list_fmt0;
  const int active_sensor_count = (sensor_format == 1) ? 7 : 6;

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

    std::cout << "\033[2K----------------------------------------" << std::endl;
    for (int i = 0; i < active_sensor_count; ++i) {
      print_sensor_data(*sdk_handle, active_sensor_list[i]);
    }
    std::cout << std::flush;

    std::this_thread::sleep_for(
        std::chrono::milliseconds(kSensorLoopDelayMs));
  }
  }

Cleanup:
  stop_monitor = true;
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }

  sdk_handle->close();
  ec_master->stop();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
