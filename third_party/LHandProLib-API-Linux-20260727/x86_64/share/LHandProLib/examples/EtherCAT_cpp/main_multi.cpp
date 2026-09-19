// ============================================================================
// 文件说明:
// File overview:
// 该示例演示如何在同一个 EtherCAT 主站下，同时连接、初始化并分别控制两个从站。
// This sample shows how to connect, initialize, and control two EtherCAT
// slaves independently under the same EtherCAT master.
//
// 主要流程包括:
// Main flow includes:
// 1. 扫描并选择网口
// 1. Scan and select a network adapter
// 2. 扫描并选择两个从站
// 2. Scan and select two slaves
// 3. 为两个从站分别创建并绑定 LHandProLib 实例
// 3. Create and bind one LHandProLib instance for each slave
// 4. 后台循环接收并解析两个从站的 TPDO 数据
// 4. Receive and decode TPDO data for both slaves in a background loop
// 5. 执行双从站运动测试或传感器读取测试
// 5. Run the dual-slave motion test or sensor read test
// ============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <ctime>
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

// 监控线程轮询两个从站 TPDO 数据的周期，单位为毫秒。
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

// 读取一个指定范围内的数字输入，并可选地排除一个已占用的值。
// Read a numeric input within range, with an optional excluded value.
int select_number_in_range(const std::string& prompt, int min_value,
                           int max_value, int excluded_value = -1) {
  while (true) {
    std::cout << prompt << " [" << min_value << " - " << max_value << "]"
              << std::endl;

    int value = 0;
    std::cin >> value;
    if (value < min_value || value > max_value) {
      std::cout << "Input out of range, please try again." << std::endl;
      continue;
    }

    if (value == excluded_value) {
      std::cout << "This id is already used, please choose another one."
                << std::endl;
      continue;
    }

    return value;
  }
}

// 打印一组目标位置，便于观察当前步的控制目标。
// Print one position set so the current command step is easy to inspect.
void print_positions(const std::string& title, int line_index,
                     const std::vector<int>& positions, int return_code) {
  std::cout << title << " line: " << line_index << " positions: ";
  for (int value : positions) {
    std::cout << value << " ";
  }
  std::cout << "retn: " << return_code << std::endl;
}

struct FingerForceData {
  std::vector<float> normal_force;
  std::vector<float> tangential_force;
  std::vector<float> force_direction;
};

using FingerForceGetter = int (SDK_NS::SDK_CLASS::*)(int, float*, int*);

bool read_finger_force_array(SDK_NS::SDK_CLASS& lib, int sensor_id,
                             FingerForceGetter getter,
                             std::vector<float>& values) {
  int count = 0;
  int result = (lib.*getter)(sensor_id, nullptr, &count);
  if (result != SDK_NS::LER_NONE || count <= 0) {
    return false;
  }

  values.assign(static_cast<size_t>(count), 0.0f);
  result = (lib.*getter)(sensor_id, values.data(), &count);
  if (result != SDK_NS::LER_NONE || count < 0) {
    return false;
  }

  values.resize(static_cast<size_t>(count));
  return true;
}

bool read_finger_force_data(SDK_NS::SDK_CLASS& lib, int sensor_id,
                            FingerForceData& data) {
  return read_finger_force_array(
             lib, sensor_id, &SDK_NS::SDK_CLASS::get_finger_normal_force_ex,
             data.normal_force) &&
         read_finger_force_array(
             lib, sensor_id,
             &SDK_NS::SDK_CLASS::get_finger_tangential_force_ex,
             data.tangential_force) &&
         read_finger_force_array(
             lib, sensor_id, &SDK_NS::SDK_CLASS::get_finger_force_direction_ex,
             data.force_direction);
}

std::string format_finger_force_sample(const FingerForceData& data,
                                       size_t index) {
  std::ostringstream sample;
  if (index >= data.normal_force.size() || index >= data.tangential_force.size() ||
      index >= data.force_direction.size()) {
    sample << "force" << (index + 1) << "(n/a)";
    return sample.str();
  }

  sample << "force" << (index + 1) << "(nf:" << data.normal_force[index]
         << ",tf:" << data.tangential_force[index]
         << ",dir:" << data.force_direction[index] << ")";
  return sample.str();
}

std::string format_palm_pressure(SDK_NS::SDK_CLASS& lib,
                                 int palm_sensor_id) {
  std::ostringstream palm;
  palm << "palm:" << palm_sensor_id << " pressure:";

  int count = 0;
  int result = lib.get_finger_pressure(palm_sensor_id, nullptr, &count);
  if (result != SDK_NS::LER_NONE || count <= 0) {
    palm << "n/a";
    return palm.str();
  }

  std::vector<float> pressure_list(static_cast<size_t>(count), 0.0f);
  result = lib.get_finger_pressure(palm_sensor_id, pressure_list.data(), &count);
  if (result != SDK_NS::LER_NONE || count < 0) {
    palm << "n/a";
    return palm.str();
  }

  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      palm << ",";
    }
    palm << pressure_list[static_cast<size_t>(i)];
  }
  return palm.str();
}

// 打印每个手指的两组三维力数据和手心压力，便于观察原始反馈。
// Print two 3D force samples for each finger sensor and palm pressure.
void print_sensor_force_data(
    SDK_NS::SDK_CLASS& lib, const std::vector<int>& finger_sensor_id_list,
    int palm_sensor_id) {
  constexpr size_t kForceSamplesToPrint = 2;
  static size_t last_line_length = 0;

  std::ostringstream line;
  for (int sensor_id : finger_sensor_id_list) {
    FingerForceData data;
    read_finger_force_data(lib, sensor_id, data);

    line << "id:" << sensor_id << " ";
    for (size_t i = 0; i < kForceSamplesToPrint; ++i) {
      line << format_finger_force_sample(data, i);
      if (i + 1 < kForceSamplesToPrint) {
        line << " ";
      }
    }
    line << "\t";
  }
  line << format_palm_pressure(lib, palm_sensor_id);
  const std::string output = line.str();
  std::cout << '\r' << output;
  if (output.size() < last_line_length) {
    std::cout << std::string(last_line_length - output.size(), ' ');
  }
  std::cout << std::flush;
  last_line_length = output.size();
}

void finish_sensor_force_refresh_line() {
  std::cout << std::endl;
}

// 为一个从站批量下发本步运动参数，包括目标位置、速度和电流限制。
// Apply one motion step to a slave, including position, velocity, and current.
void configure_motion(SDK_NS::SDK_CLASS& lib,
                      const std::vector<int>& positions, int dof_active) {
  for (int axis = 0; axis < dof_active; ++axis) {
    const int joint_id = axis + 1;
    lib.set_target_position(joint_id, positions[axis]);
    lib.set_position_velocity(joint_id, kPositionVelocity);
    lib.set_max_current(joint_id, kMaxCurrent);
  }
}

// 从指定从站读取一帧 TPDO 输入数据，并交给对应 SDK 实例进行解析。
// Read one TPDO input frame from the given slave and decode it in the SDK.
void refresh_tpdo(EthercatMaster& master, int slave_id,
                  SDK_NS::SDK_CLASS& lib) {
  const int input_size = master.getSlaveInputSize(slave_id);
  if (input_size <= 0) {
    return;
  }

  std::vector<uint8_t> input_buffer(input_size);
  if (master.getSlaveInputs(slave_id, input_buffer.data(), input_size)) {
    lib.set_tpdo_data_decode(input_buffer.data(), input_size);
  }
}

}  // namespace

int main() {
  auto sdk_handle_1 = std::make_shared<SDK_NS::SDK_CLASS>();
  auto sdk_handle_2 = std::make_shared<SDK_NS::SDK_CLASS>();
  auto ec_master = std::make_shared<EthercatMaster>();

  std::atomic<bool> stop_monitor{false};
  std::thread monitor_thread;
  int return_code = 0;

  // 运动测试使用的双从站目标位置表。
  // Motion tables used by the dual-slave motion example.
  const std::vector<std::vector<int>> positions_1 = {
      {10000, 0, 0, 0, 0, 0}, {0, 10000, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0},
      {0, 0, 10000, 0, 0, 0}, {0, 0, 0, 10000, 0, 0}, {0, 0, 0, 0, 10000, 0},
      {0, 0, 0, 0, 0, 10000},
  };

  // 第二个从站使用另一组镜像/交错的目标位置表。
  // A second motion table for the other slave, using a mirrored/staggered order.
  const std::vector<std::vector<int>> positions_2 = {
      {0, 0, 0, 0, 0, 10000}, {0, 0, 0, 0, 10000, 0}, {0, 0, 0, 10000, 0, 0},
      {0, 0, 10000, 0, 0, 0}, {0, 0, 0, 0, 0, 0},     {0, 10000, 0, 0, 0, 0},
      {10000, 0, 0, 0, 0, 0},
  };

  const int position_count =
      std::min<int>(positions_1.size(), positions_2.size());

  // ---------------------------------------------------------------------------
  // 1. 初始化 EtherCAT 主站并选择网口
  // 1. Initialize EtherCAT master and let the user choose a network adapter.
  // ---------------------------------------------------------------------------
  std::cout << "Using EtherCAT communication (100M)." << std::endl << std::endl;

  const std::vector<std::string> interface_names =
      ec_master->scanNetworkInterfaces();
  std::cout << "Detected network adapters: " << interface_names.size()
            << std::endl
            << std::endl;

  if (interface_names.empty()) {
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
  // 2. 扫描从站并选择两个不同的从站编号
  // 2. Scan slaves and choose two different slave ids for this example.
  // ---------------------------------------------------------------------------
  const int slave_count = ec_master->getSlaveCount();
  std::cout << "\nDetected slave count: " << slave_count << std::endl;

  const auto slave_list = ec_master->getSlaveInfoList();
  for (const auto& slave : slave_list) {
    std::cout << "  Slave " << slave.index << " : " << slave.name << std::endl;
  }

  if (slave_count < 2) {
    std::cerr
        << "This sample is intended for two-slave control, but fewer than two "
           "slaves were found."
        << std::endl;
    return -1;
  }

  const int slave_id_1 = select_number_in_range(
      "\nPlease select the first slave id", 1, slave_count);
  std::cout << "Selected first slave: " << slave_id_1 << std::endl;

  const int slave_id_2 = select_number_in_range(
      "\nPlease select the second slave id", 1, slave_count, slave_id_1);
  std::cout << "Selected second slave: " << slave_id_2 << std::endl;

  // 启动 EtherCAT 后台交换循环，后续 SDK 的发送和接收都依赖这条循环。
  // Start the EtherCAT background exchange loop. The SDK send/receive callbacks
  // below depend on this loop to keep PDO data flowing.
  ec_master->run();

  // ---------------------------------------------------------------------------
  // 3. 为每个从站分别绑定一个 LHandProLib 实例
  // 3. Bind one LHandProLib instance to each slave.
  // ---------------------------------------------------------------------------
  auto send_func_1 = [ec_master, slave_id_1](const unsigned char* data,
                                             unsigned int size) {
    return ec_master->setSlaveOutputs(slave_id_1, data, size);
  };
  auto send_func_2 = [ec_master, slave_id_2](const unsigned char* data,
                                             unsigned int size) {
    return ec_master->setSlaveOutputs(slave_id_2, data, size);
  };

  std::function<bool(const unsigned char*, unsigned int)> func1 = send_func_1;
  std::function<bool(const unsigned char*, unsigned int)> func2 = send_func_2;

  sdk_handle_1->set_send_rpdo_callback_ex(&func1);
  sdk_handle_2->set_send_rpdo_callback_ex(&func2);

  // 监控线程持续从两个从站获取 TPDO 数据，并交给各自的 SDK 实例解析。
  // This monitor thread continuously fetches TPDO data from both slaves and
  // forwards it to the matching SDK instance for decoding.
  monitor_thread = std::thread([=, &stop_monitor]() {
    std::this_thread::sleep_for(
        std::chrono::milliseconds(kMonitorStartupDelayMs));

    while (!stop_monitor) {
      refresh_tpdo(*ec_master, slave_id_1, *sdk_handle_1);
      refresh_tpdo(*ec_master, slave_id_2, *sdk_handle_2);
      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMonitorLoopDelayMs));
    }
  });

  int dof_total = 0;
  int dof_active = 0;
  int dof_total_2 = 0;
  int dof_active_2 = 0;

  if (sdk_handle_1->initial(SDK_NS::LCN_ECAT) != SDK_NS::LER_NONE) {
    std::cerr << "LHandProLib 1 init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  if (sdk_handle_2->initial(SDK_NS::LCN_ECAT) != SDK_NS::LER_NONE) {
    std::cerr << "LHandProLib 2 init failed." << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  // 从这里开始，sdk_handle_1 对应 slave_id_1，sdk_handle_2 对应 slave_id_2。
  // From here on, sdk_handle_1 controls slave_id_1 and sdk_handle_2 controls
  // slave_id_2. Sending the same API call to each instance means controlling
  // the two slaves independently with the same program.

  // ---------------------------------------------------------------------------
  // 4. 读取从站能力，完成使能，然后执行回零
  // 4. Read slave capability, enable both devices, then home them.
  // ---------------------------------------------------------------------------
  sdk_handle_1->get_dof(&dof_total, &dof_active);
  std::cout << "Slave 1 DOF: total " << dof_total << ", active " << dof_active
            << std::endl;

  sdk_handle_2->get_dof(&dof_total_2, &dof_active_2);
  std::cout << "Slave 2 DOF: total " << dof_total_2 << ", active "
            << dof_active_2 << std::endl;

  if (dof_active != dof_active_2) {
    std::cerr << "The two slaves report different active DOF counts. This "
                 "sample uses one shared motion table, so it expects them to "
                 "match."
              << std::endl;
    return_code = -1;
    goto Cleanup;
  }

  sdk_handle_1->set_enable(0, 1);
  sdk_handle_2->set_enable(0, 1);

  std::cout << "Waiting for enable..." << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(kEnableWaitMs));

  std::cout << "Homing both slaves..." << std::endl;
  sdk_handle_1->home_motors(0);
  sdk_handle_2->home_motors(0);
  std::this_thread::sleep_for(std::chrono::milliseconds(kHomeWaitMs));

#if 1  // motion test
  // ---------------------------------------------------------------------------
  // 5. 运动测试
  // 5. Dual-slave motion example.
  // ---------------------------------------------------------------------------
  std::cout << "Starting cyclic motion test..." << std::endl;

  while (true) {
    for (int i = 0; i < position_count; ++i) {
      if (is_escape_pressed()) {
        std::cout << "Esc pressed, exiting..." << std::endl;
        goto Cleanup;
      }

      configure_motion(*sdk_handle_1, positions_1[i], dof_active);
      configure_motion(*sdk_handle_2, positions_2[i], dof_active);

      const int return_code_1 = sdk_handle_1->move_motors();
      if (return_code_1 != SDK_NS::LER_NONE) {
        std::cerr << "Slave 1 motion failed, error code: " << return_code_1
                  << std::endl;
        return_code = return_code_1;
        goto Cleanup;
      }

      const int return_code_2 = sdk_handle_2->move_motors();
      if (return_code_2 != SDK_NS::LER_NONE) {
        std::cerr << "Slave 2 motion failed, error code: " << return_code_2
                  << std::endl;
        return_code = return_code_2;
        goto Cleanup;
      }

      print_positions("slave 1", i, positions_1[i], return_code_1);
      print_positions("slave 2", i, positions_2[i], return_code_2);

      std::this_thread::sleep_for(
          std::chrono::milliseconds(kMotionStepDelayMs));
    }
  }

#else  // sensor test
  // ---------------------------------------------------------------------------
  // 5. 传感器测试
  // 5. Sensor read example.
  // ---------------------------------------------------------------------------
  // 该分支演示如何读取从站 1 的传感器数据；从站 2 的读取方式完全相同。
  // This branch shows how to read sensor data from slave 1. Reading from
  // slave 2 uses the same API on sdk_handle_2.
  std::cout << "Reading sensor data..." << std::endl;
  sdk_handle_1->set_sensor_enable(true);

  std::vector<int> finger_sensor_id_list = {
      SDK_NS::LSS_FINGER_1_1, SDK_NS::LSS_FINGER_2_1, SDK_NS::LSS_FINGER_3_1,
      SDK_NS::LSS_FINGER_4_1, SDK_NS::LSS_FINGER_5_1};
  const int palm_sensor_id = SDK_NS::LSS_HAND_PALM;

  while (true) {
    if (is_escape_pressed()) {
      finish_sensor_force_refresh_line();
      std::cout << "Esc pressed, exiting..." << std::endl;
      goto Cleanup;
    }

    print_sensor_force_data(*sdk_handle_1, finger_sensor_id_list, palm_sensor_id);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
#endif

Cleanup:
  // 使用统一清理路径，确保线程退出、SDK 关闭和主站停止都能一致执行。
  // Use a single cleanup path so thread shutdown and SDK close happen
  // consistently on normal exit and on early error.
  stop_monitor = true;
  if (monitor_thread.joinable()) {
    monitor_thread.join();
  }

  sdk_handle_1->close();
  sdk_handle_2->close();
  ec_master->stop();

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return return_code;
}
