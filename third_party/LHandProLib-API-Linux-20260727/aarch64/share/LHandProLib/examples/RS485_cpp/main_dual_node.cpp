// ============================================================================
// This example controls two RS485 hand nodes through one physical serial port.
//
// The SDK owns RS485 framing, CRC, receive dispatch, and the single shared
// monitor scheduler. User code only opens the serial port and forwards opaque
// byte chunks through RS485SharedBus.
// ============================================================================

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <iostream>
#include <memory>
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

namespace {

using Sdk = SDK_NS::SDK_CLASS;
using namespace std::chrono_literals;

constexpr unsigned int kNode1Id = 1;
constexpr unsigned int kNode2Id = 2;
constexpr auto kEnableWait = 1000ms;
constexpr auto kHomeWait = 5000ms;
constexpr int kPositionVelocity = 20000;
constexpr int kMaxCurrent = 1000;

struct HandNode {
  unsigned int node_id{0};
  std::shared_ptr<Sdk> sdk;
  int dof_total{0};
  int dof_active{0};
  int hand_type{SDK_NS::LAC_DOF_6};
  bool initialized{false};
};

int select_number_in_range(const std::string& prompt, int min_value,
                           int max_value, int default_value) {
  std::string line;
  while (true) {
    std::cout << prompt << " [" << min_value << " - " << max_value << "]"
              << std::endl;
    if (!std::getline(std::cin, line)) {
      return default_value;
    }
    if (line.empty()) {
      return default_value;
    }
    try {
      const int value = std::stoi(line);
      if (value >= min_value && value <= max_value) {
        return value;
      }
    } catch (...) {}
    std::cout << "Input out of range, please try again." << std::endl;
  }
}

void print_result(const HandNode& node, const std::string& operation,
                  int result) {
  std::cout << "Node " << node.node_id << " " << operation
            << ", result=" << result << std::endl;
}

int initialize_node(HandNode& node) {
  std::cout << "Initializing SDK for Node " << node.node_id << " ..."
            << std::endl;
  const int result = node.sdk->initial(SDK_NS::LCN_RS485, node.node_id);
  if (result != SDK_NS::LER_NONE) {
    print_result(node, "initial failed", result);
    return result;
  }

  node.sdk->get_dof(&node.dof_total, &node.dof_active);
  node.sdk->get_hand_type(&node.hand_type);
  node.initialized = true;

  std::cout << "Node " << node.node_id << " initialized: total DOF "
            << node.dof_total << ", active DOF " << node.dof_active
            << ", hand type " << node.hand_type << std::endl;
  return SDK_NS::LER_NONE;
}

int set_node_enable(HandNode& node, bool enable) {
  const int result = node.sdk->set_enable(0, enable ? 1 : 0);
  print_result(node, enable ? "enable" : "disable", result);
  return result;
}

int home_node(HandNode& node) {
  const int result = node.sdk->home_motors(0);
  print_result(node, "home all motors", result);
  if (result == SDK_NS::LER_NONE) {
    std::cout << "Waiting " << kHomeWait.count() << " ms for Node "
              << node.node_id << " homing..." << std::endl;
    std::this_thread::sleep_for(kHomeWait);
  }
  return result;
}

int move_node(HandNode& node) {
  const int joint_id = select_number_in_range("Joint ID (0 means all joints)",
                                              0, node.dof_active, 0);
  const int position = select_number_in_range("Target position", 0, 10000, 0);

  int result = node.sdk->set_target_position(joint_id, position);
  if (result == SDK_NS::LER_NONE) {
    result = node.sdk->set_position_velocity(joint_id, kPositionVelocity);
  }
  if (result == SDK_NS::LER_NONE) {
    result = node.sdk->set_max_current(joint_id, kMaxCurrent);
  }
  if (result == SDK_NS::LER_NONE) {
    result = node.sdk->move_motors(joint_id);
  }

  std::cout << "Node " << node.node_id << " move joint " << joint_id << " to "
            << position << ", result=" << result << std::endl;
  return result;
}

void print_node_state(HandNode& node) {
  std::cout << "\nNode " << node.node_id << " state" << std::endl;
  std::cout << " axis | position | enable | reached | status | alarm"
            << std::endl;
  std::cout << "------+----------+--------+---------+--------+------"
            << std::endl;

  for (int axis = 1; axis <= node.dof_active; ++axis) {
    int position = 0;
    int enable = 0;
    int reached = 0;
    int status = 0;
    int alarm = 0;
    node.sdk->get_now_position(axis, &position);
    node.sdk->get_enable(axis, &enable);
    node.sdk->get_position_reached(axis, &reached);
    node.sdk->get_now_status(axis, &status);
    node.sdk->get_now_alarm(axis, &alarm);

    std::cout << std::setw(5) << axis << " | " << std::setw(8) << position
              << " | " << std::setw(6) << enable << " | " << std::setw(7)
              << reached << " | " << std::setw(6) << status << " | "
              << std::setw(5) << alarm << std::endl;
  }
}

HandNode& select_node(HandNode& node1, HandNode& node2) {
  const int selected =
      select_number_in_range("Select Node ID", kNode1Id, kNode2Id, kNode1Id);
  return selected == static_cast<int>(kNode1Id) ? node1 : node2;
}

void print_menu() {
  std::cout << "\n========== Dual-node RS485 test ==========\n"
            << "  1 - Enable one node\n"
            << "  2 - Disable one node\n"
            << "  3 - Home one node\n"
            << "  4 - Move one node\n"
            << "  5 - Print one node state\n"
            << "  6 - Print both node states\n"
            << "  7 - Enable and home both nodes\n"
            << "  0 - Exit\n"
            << "==========================================" << std::endl;
}

}  // namespace

int main() {
  std::cout << "RS485 dual-node example: Node 1 + Node 2 on one serial bus."
            << std::endl;

  const std::vector<std::string> port_names = SerialPort::scanAvailablePorts();
  if (port_names.empty()) {
    std::cerr << "No serial port found." << std::endl;
    return -1;
  }

  for (size_t index = 0; index < port_names.size(); ++index) {
    std::cout << "  [" << index << "] " << port_names[index] << std::endl;
  }
  const int port_index = select_number_in_range(
      "Select serial port", 0, static_cast<int>(port_names.size()) - 1, 0);

  auto serial_port = std::make_shared<SerialPort>();
  if (!serial_port->open(port_names[port_index])) {
    std::cerr << "Failed to open serial port " << port_names[port_index]
              << std::endl;
    return -1;
  }

  auto shared_bus = std::make_shared<SDK_NS::RS485SharedBus>();
  std::atomic<bool> stop_receive{false};

  // The callback receives opaque, fully framed bytes. All command knowledge,
  // prioritization, and node arbitration remain inside the SDK.
  std::function<bool(const unsigned char*, unsigned int)> bus_send =
      [serial_port](const unsigned char* data, unsigned int size) {
        return serial_port->write(data, size);
      };
  shared_bus->set_send_callback_ex(&bus_send);

  HandNode node1{kNode1Id, std::make_shared<Sdk>()};
  HandNode node2{kNode2Id, std::make_shared<Sdk>()};
  node1.sdk->set_rs485_shared_bus(shared_bus.get());
  node2.sdk->set_rs485_shared_bus(shared_bus.get());

  const std::weak_ptr<SDK_NS::RS485SharedBus> weak_bus = shared_bus;
  serial_port->setReadCallback(
      [weak_bus, &stop_receive](const uint8_t* data, size_t size) {
        if (stop_receive.load())
          return;
        if (const auto bus = weak_bus.lock())
          bus->set_receive_data(data, static_cast<int>(size));
      });

  int return_code = 0;

  // Keep the existing initial(mode, node_id) API for each hand. Initialization
  // requests take priority over background polls on the shared bus.
  return_code = initialize_node(node1);
  if (return_code == SDK_NS::LER_NONE) {
    return_code = initialize_node(node2);
  }

  if (return_code == SDK_NS::LER_NONE) {
    std::cout << "Both SDK instances are monitored by one SDK RS485 bus worker."
              << std::endl;

    bool running = true;
    while (running) {
      print_menu();
      const int operation = select_number_in_range("Select operation", 0, 7, 0);

      switch (operation) {
        case 0:
          running = false;
          break;
        case 1: {
          HandNode& node = select_node(node1, node2);
          if (set_node_enable(node, true) == SDK_NS::LER_NONE) {
            std::this_thread::sleep_for(kEnableWait);
          }
          break;
        }
        case 2: {
          HandNode& node = select_node(node1, node2);
          set_node_enable(node, false);
          break;
        }
        case 3: {
          HandNode& node = select_node(node1, node2);
          home_node(node);
          break;
        }
        case 4: {
          HandNode& node = select_node(node1, node2);
          move_node(node);
          break;
        }
        case 5: {
          HandNode& node = select_node(node1, node2);
          print_node_state(node);
          break;
        }
        case 6:
          print_node_state(node1);
          print_node_state(node2);
          break;
        case 7: {
          const int enable1 = set_node_enable(node1, true);
          const int enable2 = set_node_enable(node2, true);
          if (enable1 == SDK_NS::LER_NONE && enable2 == SDK_NS::LER_NONE) {
            std::this_thread::sleep_for(kEnableWait);
            const int home1 = node1.sdk->home_motors(0);
            const int home2 = node2.sdk->home_motors(0);
            print_result(node1, "home all motors", home1);
            print_result(node2, "home all motors", home2);
            if (home1 == SDK_NS::LER_NONE && home2 == SDK_NS::LER_NONE) {
              std::this_thread::sleep_for(kHomeWait);
            }
          }
          break;
        }
        default:
          break;
      }
    }
  }

  // Keep the shared SDK bus alive until both hand instances stop monitoring.
  node1.sdk->close();
  node2.sdk->close();
  stop_receive.store(true);
  serial_port->setReadCallback({});
  shared_bus->close();
  serial_port->close();

  return return_code;
}
