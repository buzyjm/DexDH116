// EthercatMaster.cpp
#include "EthercatMaster.h"

#include "EthercatIoMapBounds.h"

#include <cctype>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {
constexpr std::size_t kIoMapCapacity =
    static_cast<std::size_t>(EC_MAXIOSEGMENTS) * EC_MAXLRWDATA;

constexpr int kProcessCycleUs = 1000;

const char* ecStateToString(uint16_t state) {
  switch (state & 0x0F) {
    case EC_STATE_INIT:
      return "INIT";
    case EC_STATE_PRE_OP:
      return "PRE_OP";
    case EC_STATE_SAFE_OP:
      return "SAFE_OP";
    case EC_STATE_OPERATIONAL:
      return "OP";
    default:
      return "UNKNOWN";
  }
}

std::string hexValue(uint32_t value, int width) {
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(width)
      << std::setfill('0') << value;
  return oss.str();
}

std::string sdoName(uint16_t index, uint8_t subindex) {
  std::ostringstream oss;
  oss << "0x" << std::uppercase << std::hex << std::setw(4)
      << std::setfill('0') << index << ":" << std::setw(2)
      << static_cast<int>(subindex);
  return oss.str();
}

bool isLslqDh116Slave(const ec_slavet* slave) {
  return slave && slave->eep_id == 0x00660000 &&
         slave->eep_rev == 0x00000001;
}
}  // namespace

#if ETHERCAT_DEBUG
#define ECOUT std::cout
#else
#define ECOUT \
  if (0)      \
  std::cout
#endif

// 平台特定的网口过滤关键词
#ifdef _WIN32
// Windows特定关键词
static const std::vector<std::string> EXCLUDE_KEYWORDS = {
    "wan miniport",  // Windows WAN Miniport
    "wi-fi",         // Windows WiFi
    "wifi",          // WiFi接口
    "wireless",      // 无线
    "bluetooth",     // 蓝牙
    "vmware",        // VMware虚拟网卡
    "virtual",       // 虚拟设备
    "loopback",      // 回环(Windows)
    "tap-",          // TAP虚拟设备
    "vpn",           // VPN接口
    "wintun",        // Windows TUN设备
    "teredo",        // Teredo隧道
    "isatap"         // ISATAP隧道
};
#else
// Linux特定关键词
static const std::vector<std::string> EXCLUDE_KEYWORDS = {
    "lo",       // Linux 回环接口
    "docker",   // Docker虚拟网卡
    "veth",     // 虚拟以太网设备
    "br-",      // 网桥接口
    "virbr",    // 虚拟网桥
    "vmnet",    // VMware虚拟网卡
    "tap",      // TAP虚拟设备
    "tun",      // TUN虚拟设备
    "wlan",     // 无线网卡
    "wlp",      // 无线网卡(新命名)
    "wlx",      // 无线网卡
    "wwan",     // 无线广域网
    "vboxnet",  // VirtualBox虚拟网卡
    "p2p",      // P2P连接
    "teredo",   // Teredo隧道
    "isatap"    // ISATAP隧道
};
#endif

std::string SlaveInfo::toString() const {
  // 状态字符串映射
  const char* state_str = ecStateToString(state);

  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "Slave %2d [%-16s] State: %8s (0x%02X) AL: 0x%04X (%s)%s", index,
           name.substr(0, 16).c_str(), state_str, state, al_status,
           al_status_str.c_str(), is_lost ? " [LOST]" : "");

  return std::string(buffer);
}

std::ostream& operator<<(std::ostream& os, const SlaveInfo& info) {
  return os << info.toString();
}

std::string EthercatMaster::getLastError() const {
  std::lock_guard<std::mutex> lock(error_mutex_);
  return last_error_;
}

void EthercatMaster::clearLastError() {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_.clear();
}

void EthercatMaster::setLastError(const std::string& message) {
  std::lock_guard<std::mutex> lock(error_mutex_);
  last_error_ = message;
}

int EthercatMaster::getExpectedWkc() const {
  const ec_groupt* grp = context_.grouplist + group_;
  return grp->outputsWKC * 2 + grp->inputsWKC;
}

std::string EthercatMaster::describeSlaveState(int slave_index) const {
  if (slave_index < 1 || slave_index > context_.slavecount) {
    return {};
  }

  const ec_slavet* slave = context_.slavelist + slave_index;
  std::ostringstream oss;
  oss << "S" << slave_index << "=" << ecStateToString(slave->state) << "(0x"
      << std::hex << std::setw(4) << std::setfill('0') << slave->state << ")";

  if (slave->ALstatuscode != 0) {
    oss << " AL=0x" << std::setw(4) << slave->ALstatuscode << " "
        << ec_ALstatuscode2string(slave->ALstatuscode);
  }

  oss << std::dec << std::setfill(' ');
  return oss.str();
}

std::string EthercatMaster::describeAllSlaveStates() const {
  std::ostringstream oss;
  for (int i = 1; i <= context_.slavecount; ++i) {
    if (i > 1) {
      oss << " | ";
    }
    oss << describeSlaveState(i);
  }
  return oss.str();
}

void EthercatMaster::dumpSoemErrors(const std::string& phase) {
  while (ecx_iserror(&context_)) {
    const char* error = ecx_elist2string(&context_);
    ECOUT << "[" << phase << "] SOEM error: " << (error ? error : "");
  }
}

void EthercatMaster::dumpSlaveDiagnostics(const std::string& phase) {
  ecx_readstate(&context_);
  ECOUT << "[" << phase << "] slave diagnostics:\n";

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    ECOUT << "  " << describeSlaveState(i) << " Name='" << slave->name
          << "' Obits=" << slave->Obits << " Ibits=" << slave->Ibits
          << " Obytes=" << slave->Obytes << " Ibytes=" << slave->Ibytes
          << " hasdc=" << static_cast<int>(slave->hasdc)
          << " CoE=0x" << std::hex << static_cast<int>(slave->CoEdetails)
          << " SM0=" << slave->SM[0].StartAddr << "/" << slave->SM[0].SMlength
          << " SM1=" << slave->SM[1].StartAddr << "/" << slave->SM[1].SMlength
          << " SM2=" << slave->SM[2].StartAddr << "/" << slave->SM[2].SMlength
          << " SM3=" << slave->SM[3].StartAddr << "/" << slave->SM[3].SMlength
          << std::dec << "\n";
  }

  dumpSoemErrors(phase);
}

void EthercatMaster::dumpMappingDiagnostics() const {
  const ec_groupt* grp = context_.grouplist + group_;
  ECOUT << "Group " << static_cast<int>(group_) << " mapping diagnostics:\n";
  ECOUT << "  Obytes=" << grp->Obytes << " Ibytes=" << grp->Ibytes
        << " outputsWKC=" << grp->outputsWKC
        << " inputsWKC=" << grp->inputsWKC
        << " expectedWKC=" << getExpectedWkc()
        << " nsegments=" << grp->nsegments << "\n";

  ECOUT << "  IO segments:";
  for (int i = 0; i < grp->nsegments; ++i) {
    ECOUT << " " << grp->IOsegment[i];
  }
  ECOUT << "\n";

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    ECOUT << "  Slave " << i << " '" << slave->name << "'"
          << " Obits=" << slave->Obits << " Ibits=" << slave->Ibits
          << " Obytes=" << slave->Obytes << " Ibytes=" << slave->Ibytes
          << " Ostartbit=" << static_cast<int>(slave->Ostartbit)
          << " Istartbit=" << static_cast<int>(slave->Istartbit)
          << " FMMUunused=" << static_cast<int>(slave->FMMUunused) << "\n";
  }
}

void EthercatMaster::applyLslqDh116EsiDefaults() {
  if (context_.slavecount < 1) {
    return;
  }

  ec_slavet* slave = context_.slavelist + 1;
  if (!isLslqDh116Slave(slave)) {
    return;
  }

  const bool sm2_incomplete =
      slave->SM[2].StartAddr == 0 || slave->SM[2].SMlength == 0 ||
      slave->SM[2].SMflags == 0 || slave->SMtype[2] == 0;
  const bool sm3_incomplete =
      slave->SM[3].StartAddr == 0 || slave->SM[3].SMlength == 0 ||
      slave->SM[3].SMflags == 0 || slave->SMtype[3] == 0;

  if (!sm2_incomplete && !sm3_incomplete) {
    ECOUT << "ESI fallback: SM2/SM3 already complete\n";
    return;
  }

  if (sm2_incomplete) {
    if (slave->SM[2].StartAddr == 0) {
      slave->SM[2].StartAddr = htoes(0x1100);
    }
    if (slave->SM[2].SMlength == 0) {
      slave->SM[2].SMlength = htoes(64);
    }
    if (slave->SM[2].SMflags == 0) {
      slave->SM[2].SMflags = htoel(0x00010064);
    }
    if (slave->SMtype[2] == 0) {
      slave->SMtype[2] = 3;
    }
  }

  if (sm3_incomplete) {
    if (slave->SM[3].StartAddr == 0) {
      slave->SM[3].StartAddr = htoes(0x1400);
    }
    if (slave->SM[3].SMlength == 0) {
      slave->SM[3].SMlength = htoes(192);
    }
    if (slave->SM[3].SMflags == 0) {
      slave->SM[3].SMflags = htoel(0x00010020);
    }
    if (slave->SMtype[3] == 0) {
      slave->SMtype[3] = 4;
    }
  }

  ECOUT << "ESI fallback: filled missing LSLQ DH116 SM2/SM3 fields\n";
}

void EthercatMaster::dumpSiiPdoDiagnostics() {
  if (context_.slavecount < 1) {
    return;
  }

  for (uint8_t type = 0; type <= 1; ++type) {
    ec_eepromPDOt pdo{};
    uint32_t bits = ecx_siiPDO(&context_, 1, &pdo, type);
    std::ostringstream oss;

    oss << "SII PDO type" << static_cast<int>(type) << ": bits=" << bits
        << " entries=" << pdo.nPDO << " indexes=";

    for (uint16_t i = 1; i <= pdo.nPDO; ++i) {
      if (i > 1) {
        oss << ",";
      }
      oss << hexValue(pdo.Index[i], 4) << "(" << pdo.BitSize[i] << "b"
          << ",SM" << static_cast<int>(pdo.SyncM[i]) << ")";
    }

    oss << " SMbits=[";
    for (int sm = 0; sm < EC_MAXSM; ++sm) {
      if (sm > 0) {
        oss << ",";
      }
      oss << "SM" << sm << ":" << pdo.SMbitsize[sm];
    }
    oss << "]";

    ECOUT << oss.str() << "\n";
  }
}

bool EthercatMaster::readSdoValue(uint16_t slave, uint16_t index,
                                  uint8_t subindex, int max_size,
                                  uint32_t* value) {
  if (!value || max_size <= 0 || max_size > 4) {
    return false;
  }

  uint8_t buffer[4] = {};
  int size = max_size;
  int result = ecx_SDOread(&context_, slave, index, subindex, FALSE, &size,
                           buffer, EC_TIMEOUTRXM * 3);

  if (result <= 0 || size <= 0) {
    ECOUT << "ESI verify: CoE read " << sdoName(index, subindex)
          << " unavailable ret=" << result << "\n";
    dumpSoemErrors("ESI-SDO");
    return false;
  }

  uint32_t parsed = 0;
  for (int i = 0; i < size && i < 4; ++i) {
    parsed |= static_cast<uint32_t>(buffer[i]) << (8 * i);
  }

  *value = parsed;
  return true;
}

void EthercatMaster::verifySdoValue(const std::string& name, uint16_t index,
                                    uint8_t subindex, uint32_t expected,
                                    int max_size, int* mismatch_count) {
  uint32_t actual = 0;
  if (!readSdoValue(1, index, subindex, max_size, &actual)) {
    return;
  }

  if (actual != expected) {
    if (mismatch_count) {
      ++(*mismatch_count);
    }
    ECOUT << "ESI mismatch: " << name << " " << sdoName(index, subindex)
          << " actual=" << hexValue(actual, max_size * 2)
          << " expected=" << hexValue(expected, max_size * 2) << "\n";
  } else {
    ECOUT << "ESI OK: " << name << " " << sdoName(index, subindex) << " = "
          << hexValue(actual, max_size * 2) << "\n";
  }
}

void EthercatMaster::verifyPdoMapping(const std::string& name,
                                      uint16_t pdo_index,
                                      uint16_t object_index,
                                      uint8_t expected_entries,
                                      int* mismatch_count) {
  uint32_t actual_count = 0;
  int local_mismatch_count = 0;

  if (!readSdoValue(1, pdo_index, 0, 1, &actual_count)) {
    ECOUT << "ESI verify: skip " << name << " " << hexValue(pdo_index, 4)
          << " CoE mapping read\n";
    return;
  }

  if (actual_count != expected_entries) {
    ++local_mismatch_count;
    ECOUT << "ESI mismatch: " << name << " " << sdoName(pdo_index, 0)
          << " actual=" << actual_count
          << " expected=" << static_cast<int>(expected_entries) << "\n";
  }

  uint8_t count_to_check =
      actual_count < expected_entries ? static_cast<uint8_t>(actual_count)
                                      : expected_entries;

  for (uint8_t subindex = 1; subindex <= count_to_check; ++subindex) {
    uint32_t actual = 0;
    uint32_t expected = (static_cast<uint32_t>(object_index) << 16) |
                        (static_cast<uint32_t>(subindex) << 8) | 0x10;

    if (!readSdoValue(1, pdo_index, subindex, 4, &actual)) {
      ++local_mismatch_count;
      continue;
    }

    if (actual != expected) {
      ++local_mismatch_count;
      ECOUT << "ESI mismatch: " << name << " " << sdoName(pdo_index, subindex)
            << " actual=" << hexValue(actual, 8)
            << " expected=" << hexValue(expected, 8) << "\n";
    }
  }

  if (actual_count > expected_entries) {
    local_mismatch_count +=
        static_cast<int>(actual_count - expected_entries);
    ECOUT << "ESI mismatch: " << name << " extra PDO entries "
          << (actual_count - expected_entries) << "\n";
  }

  if (local_mismatch_count == 0) {
    ECOUT << "ESI OK: " << name << " " << hexValue(pdo_index, 4)
          << " entries=" << static_cast<int>(expected_entries) << "\n";
  } else if (mismatch_count) {
    *mismatch_count += local_mismatch_count;
  }
}

void EthercatMaster::verifyEsiConfiguration() {
  if (context_.slavecount < 1) {
    return;
  }

  const ec_slavet* slave = context_.slavelist + 1;
  if (!isLslqDh116Slave(slave)) {
    return;
  }

  int mismatch_count = 0;

  verifySdoValue("SM2 RxPDO count", 0x1C12, 0, 0x01, 1, &mismatch_count);
  verifySdoValue("SM2 RxPDO assignment", 0x1C12, 1, 0x1600, 2,
                 &mismatch_count);
  verifySdoValue("SM3 TxPDO count", 0x1C13, 0, 0x01, 1, &mismatch_count);
  verifySdoValue("SM3 TxPDO assignment", 0x1C13, 1, 0x1A00, 2,
                 &mismatch_count);

  verifyPdoMapping("RxPDO", 0x1600, 0x9000, 32, &mismatch_count);
  verifyPdoMapping("TxPDO", 0x1A00, 0x9001, 96, &mismatch_count);
  dumpSiiPdoDiagnostics();

  ECOUT << "ESI verify complete: mismatches=" << mismatch_count << "\n";
}

void EthercatMaster::verifyMappedEsiConfiguration() {
  if (context_.slavecount < 1) {
    return;
  }

  int mismatch_count = 0;
  const ec_slavet* slave = context_.slavelist + 1;
  if (!isLslqDh116Slave(slave)) {
    return;
  }

  const bool sm2_ok =
      etohs(slave->SM[2].StartAddr) == 0x1100 &&
      etohs(slave->SM[2].SMlength) == 64;
  const bool sm3_ok =
      etohs(slave->SM[3].StartAddr) == 0x1400 &&
      etohs(slave->SM[3].SMlength) == 192;

  if (!sm2_ok) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: SM2 actual start="
          << hexValue(etohs(slave->SM[2].StartAddr), 4)
          << " len=" << etohs(slave->SM[2].SMlength)
          << " expected start=0x1100 len=64\n";
  }

  if (!sm3_ok) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: SM3 actual start="
          << hexValue(etohs(slave->SM[3].StartAddr), 4)
          << " len=" << etohs(slave->SM[3].SMlength)
          << " expected start=0x1400 len=192\n";
  }

  if (slave->Obytes != 64 || slave->Ibytes != 192) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: slave IO size actual=" << slave->Obytes << "O+"
          << slave->Ibytes << "I expected=64O+192I\n";
  }

  const ec_groupt* grp = context_.grouplist + group_;
  if (grp->Obytes != 64 || grp->Ibytes != 192) {
    ++mismatch_count;
    ECOUT << "ESI mismatch: group IO size actual=" << grp->Obytes << "O+"
          << grp->Ibytes << "I expected=64O+192I\n";
  }

  ECOUT << "ESI mapped verify complete: mismatches=" << mismatch_count << "\n";
}

EthercatMaster::EthercatMaster()
    : group_(0),
      roundtrip_time_(0),
      initialized_(false),
      started_(false),
      connected_index_(-1),
      running_(false),
      outputs_(nullptr),
      inputs_(nullptr),
      output_bytes_(0),
      input_bytes_(0) {
  memset(&context_, 0, sizeof(context_));
}

EthercatMaster::~EthercatMaster() {
  stop();
}

std::vector<std::string> EthercatMaster::scanNetworkInterfaces() {
  interfaces_.clear();
  std::vector<std::string> descrptions;
  ec_adaptert* adapter = ec_find_adapters();
  ec_adaptert* head = adapter;

  ECOUT << "Available adapters:\n";
  while (adapter != nullptr) {
    // 检查是否需要排除
    bool should_exclude = false;
    std::string adapter_name(adapter->name);
    std::string adapter_desc(adapter->desc);
    std::string adapter_name_lower = adapter_name;
    std::string adapter_desc_lower = adapter_desc;

    // 转换为小写便于比较
    for (char& c : adapter_name_lower) {
      c = std::tolower(static_cast<unsigned char>(c));
    }
    for (char& c : adapter_desc_lower) {
      c = std::tolower(static_cast<unsigned char>(c));
    }

    for (const std::string& keyword : EXCLUDE_KEYWORDS) {
      // 检查名称
      if (adapter_name_lower.find(keyword) != std::string::npos) {
        should_exclude = true;
        break;
      }
#ifdef _WIN32
      // 检查描述
      if (adapter_desc_lower.find(keyword) != std::string::npos) {
        should_exclude = true;
        break;
      }
#endif
    }

    if (!should_exclude) {
      ECOUT << "    - " << adapter->name << "  (" << adapter->desc << ")\n";
      interfaces_.push_back(std::string(adapter->name));
      descrptions.push_back(std::string(adapter->desc));
    }

    adapter = adapter->next;
  }
  ec_free_adapters(head);

  return descrptions;
}

bool EthercatMaster::probeSlaves(int index, std::vector<SlaveInfo>& slaves) {
  clearLastError();
  slaves.clear();

  if (running_.load() || initialized_.load()) {
    setLastError("Already connected");
    return false;
  }

  if (index < 0 || index >= static_cast<int>(interfaces_.size())) {
    setLastError("Network interface selection");
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  ecx_contextt probe_context;
  std::memset(&probe_context, 0, sizeof(probe_context));

  const std::string iface = interfaces_.at(index);
  if (!ecx_init(&probe_context, iface.c_str())) {
    setLastError("Open network interface");
    return false;
  }

  bool ok = false;
  if (ecx_config_init(&probe_context) <= 0) {
    setLastError("Scan EtherCAT slaves");
  } else {
    ecx_readstate(&probe_context);
    for (int i = 1; i <= probe_context.slavecount; ++i) {
      const ec_slavet* slave = probe_context.slavelist + i;
      SlaveInfo info;
      info.index = i;
      info.name = std::string(slave->name);
      info.state = slave->state;
      info.al_status = slave->ALstatuscode;
      info.al_status_str =
          std::string(ec_ALstatuscode2string(slave->ALstatuscode));
      info.is_lost = slave->islost;
      slaves.push_back(info);
    }
    ok = !slaves.empty();
    if (!ok) {
      setLastError("Scan EtherCAT slaves");
    }
  }

  ecx_close(&probe_context);
  if (ok) {
    clearLastError();
  }
  return ok;
}

bool EthercatMaster::init(int index) {
  clearLastError();

  if (running_.load() || initialized_.load()) {
    setLastError("Already connected");
    return false;
  }

  if (index < 0 || index >= interfaces_.size()) {
    setLastError("Network interface selection");
    ECOUT << "Index not available\n";
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  iface_ = interfaces_.at(index);

  ECOUT << "Initializing SOEM on '" << iface_ << "'... " << std::flush;
  if (!ecx_init(&context_, iface_.c_str())) {
    setLastError("Open network interface");
    ECOUT << "no socket connection\n";
    return false;
  }
  ECOUT << "done\n";

  ECOUT << "Finding autoconfig slaves... " << std::flush;
  if (ecx_config_init(&context_) <= 0) {
    dumpSoemErrors("config_init");
    setLastError("Scan EtherCAT slaves");
    ECOUT << "no slaves found\n";
    ecx_close(&context_);
    return false;
  }
  dumpSoemErrors("config_init");
  ECOUT << context_.slavecount << " slaves found\n";
  applyLslqDh116EsiDefaults();

  ECOUT << "Sequential mapping of I/O... " << std::flush;
  io_map_.assign(kIoMapCapacity, 0);
  int mapped_bytes = ecx_config_map_group(&context_, io_map_.data(), group_);
  ec_groupt* grp = context_.grouplist + group_;

  if (grp->Obytes <= 0 || grp->Ibytes <= 0 || grp->outputs == nullptr ||
      grp->inputs == nullptr) {
    setLastError("I/O mapping");
    outputs_ = nullptr;
    inputs_ = nullptr;
    output_bytes_ = 0;
    input_bytes_ = 0;
    io_map_.clear();
    ecx_close(&context_);
    return false;
  }

  if (mapped_bytes < 0 ||
      !ethercat_iomap::layoutWithinMapBounds(
          io_map_.data(), io_map_.size(), static_cast<std::size_t>(mapped_bytes),
          grp->outputs, static_cast<std::size_t>(grp->Obytes), grp->inputs,
          static_cast<std::size_t>(grp->Ibytes))) {
    setLastError("I/O mapping");
    ECOUT << "I/O map exceeds local buffer bounds: mapped " << mapped_bytes
          << " bytes, outputs " << grp->Obytes << " bytes, inputs "
          << grp->Ibytes << " bytes, capacity " << io_map_.size() << "\n";
    outputs_ = nullptr;
    inputs_ = nullptr;
    output_bytes_ = 0;
    input_bytes_ = 0;
    io_map_.clear();
    ecx_close(&context_);
    return false;
  }

  output_bytes_ = static_cast<int>(grp->Obytes);
  input_bytes_ = static_cast<int>(grp->Ibytes);
  outputs_ = grp->outputs;
  inputs_ = grp->inputs;

  //  初始化缓冲区大小
  output_buffer_.resize(output_bytes_);
  if (outputs_ && output_bytes_ > 0) {
    std::memset(outputs_, 0, output_bytes_);
  }

  // 填充每从站 PDO 布局信息
  slave_io_info_.clear();
  for (int i = 1; i <= context_.slavecount; ++i) {
    ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_) continue;
    SlaveIOInfo info;
    info.slave_id      = i;
    info.name          = std::string(slave->name);
    info.output_bytes  = (slave->Obytes > 0 && slave->outputs) ? (int)slave->Obytes : 0;
    info.output_offset = (info.output_bytes > 0 && outputs_)
                             ? static_cast<int>(slave->outputs - outputs_)
                             : -1;
    info.input_bytes   = (slave->Ibytes > 0 && slave->inputs) ? (int)slave->Ibytes : 0;
    info.input_offset  = (info.input_bytes > 0 && inputs_)
                             ? static_cast<int>(slave->inputs - inputs_)
                             : -1;
    slave_io_info_.push_back(info);
  }

  ECOUT << "mapped " << grp->Obytes << "O+" << grp->Ibytes << "I bytes from "
        << grp->nsegments << " segments";

  if (grp->nsegments > 1) {
    ECOUT << " (";
    for (int i = 0; i < grp->nsegments; ++i) {
      if (i > 0)
        ECOUT << "+";
      ECOUT << grp->IOsegment[i];
    }
    ECOUT << " slaves)";
  }
  ECOUT << "\n";
  ECOUT << "Mapped logical bytes: " << mapped_bytes << "\n";
  dumpMappingDiagnostics();
  dumpSoemErrors("config_map");
  verifyMappedEsiConfiguration();
#if ETHERCAT_DEBUG
  verifyEsiConfiguration();
#endif

  ECOUT << "Configuring distributed clock... " << std::flush;
  ecx_configdc(&context_);
  dumpSoemErrors("configdc");
  ECOUT << "done\n";

  connected_index_ = index;
  initialized_.store(true);
  connection_lost_.store(false);
  clearLastError();
  return true;
}

bool EthercatMaster::start() {
  clearLastError();

  if (!initialized_.load()) {
    setLastError("SOEM init");
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (context_.slavecount < 1) {
    setLastError("Scan EtherCAT slaves");
    return false;
  }

  // ec_groupt* grp = context_.grouplist + group_;
  ec_slavet* slave = context_.slavelist;

  ECOUT << "Waiting for all slaves in safe operational... " << std::flush;
  uint16 safe_state =
      ecx_statecheck(&context_, 0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
  ecx_readstate(&context_);
  if (safe_state != EC_STATE_SAFE_OP ||
      context_.slavelist[0].state != EC_STATE_SAFE_OP) {
    dumpSlaveDiagnostics("SAFE_OP");
    setLastError("SAFE_OP");
    ECOUT << "failed to reach SAFE_OP, " << describeAllSlaveStates() << "\n";
    return false;
  }
  ECOUT << "done\n";

  ECOUT << "Send a roundtrip to make outputs in slaves happy... " << std::flush;
  ec_timet start = osal_current_time();
  ecx_send_processdata(&context_);
  int first_wkc = ecx_receive_processdata(&context_, EC_TIMEOUTSAFE);
  ec_timet end = osal_current_time();
  ec_timet diff;
  osal_time_diff(&start, &end, &diff);
  roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);
  ECOUT << " first WKC " << first_wkc << " expected " << getExpectedWkc();
  if (first_wkc < getExpectedWkc()) {
    ECOUT << " warning";
  }
  ECOUT << "done\n";

  ECOUT << "Setting operational state.." << std::flush;

  slave->state = EC_STATE_OPERATIONAL;
  ecx_writestate(&context_, 0);

  constexpr int max_op_attempts = 2000;
  for (int i = 0; i < max_op_attempts; ++i) {
    if ((i % 100) == 0) {
      ECOUT << "." << std::flush;
    }
    start = osal_current_time();
    ecx_send_processdata(&context_);
    int wkc = ecx_receive_processdata(&context_, EC_TIMEOUTSAFE);
    end = osal_current_time();
    osal_time_diff(&start, &end, &diff);
    roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

    if ((i % 10) == 0) {
      ecx_statecheck(&context_, 0, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
    }
    if (slave->state == EC_STATE_OPERATIONAL) {
      ECOUT << " all slaves are now operational\n";
      started_.store(true);
      connection_lost_.store(false);
      resetLostFrames();
      return true;
    }
    if ((i % 100) == 0) {
      ECOUT << "(wkc=" << wkc << "/" << getExpectedWkc()
            << ", state=0x" << std::hex << slave->state << std::dec << ")";
    }
    osal_usleep(kProcessCycleUs);
  }

  ECOUT << " failed,";
  ecx_readstate(&context_);
  dumpSlaveDiagnostics("OP");
  setLastError("OPERATIONAL");
  for (int i = 1; i <= context_.slavecount; ++i) {
    slave = context_.slavelist + i;
    if (slave->state != EC_STATE_OPERATIONAL) {
      ECOUT << " slave " << i << " is 0x" << std::hex << std::setfill('0')
            << std::setw(4) << slave->state << " (AL-status=0x" << std::setw(4)
            << slave->ALstatuscode << std::dec << " "
            << ec_ALstatuscode2string(slave->ALstatuscode) << ")";
    }
  }
  ECOUT << "\n";

  return false;
}

void EthercatMaster::run() {
  if (!started_.load() || running_.load()) {
    return;
  }

  running_.store(true);

  worker_thread_ = std::thread([this]() {
    int min_time = 0, max_time = 0;
    int iteration = 1;
    int consecutive_lost_frames = 0;     // 连续丢帧计数
    constexpr int max_lost_frames = 10;  // 最大断线丢帧判断数

    while (running_.load()) {
      ECOUT << "Iteration " << std::setw(4) << std::setfill(' ') << iteration
            << ":";

      int wkc = 0;
      int expected_wkc = 0;
      int64 dc_time = 0;
      std::vector<uint8_t> output_dump;
      std::vector<uint8_t> input_dump;

      {
        std::lock_guard<std::mutex> soem_lock(soem_mutex_);
        std::lock_guard<std::mutex> io_lock(io_mutex_);

        if (!started_.load() || connection_lost_.load() || outputs_ == nullptr ||
            inputs_ == nullptr || output_bytes_ <= 0 || input_bytes_ <= 0) {
          setLastError("Process data");
          connection_lost_.store(true);
          running_.store(false);
          break;
        }

        // 检测是否有新数据，有则同步到 SOEM 输出缓冲区。
        if (const uint8_t* committed = output_buffer_.consume()) {
          std::memcpy(outputs_, committed, static_cast<size_t>(output_bytes_));
        }

        ec_timet start = osal_current_time();
        ecx_send_processdata(&context_);
        wkc = ecx_receive_processdata(&context_, EC_TIMEOUTSAFE);
        ec_timet end = osal_current_time();
        ec_timet diff;
        osal_time_diff(&start, &end, &diff);
        roundtrip_time_ = (int)(diff.tv_sec * 1000000 + diff.tv_nsec / 1000);

        expected_wkc = getExpectedWkc();
        dc_time = context_.DCtime;
#if ETHERCAT_DEBUG
        ec_groupt* grp = context_.grouplist + group_;
        output_dump.assign(grp->outputs, grp->outputs + grp->Obytes);
        input_dump.assign(grp->inputs, grp->inputs + grp->Ibytes);
#endif
      }

      if (expected_wkc <= 0) {
        setLastError("Process data");
        connection_lost_.store(true);
        running_.store(false);
        break;
      }

      // 统计丢帧：WKC 小于期望值就认为是丢帧。
      if (wkc < expected_wkc) {
        if (consecutive_lost_frames == 0) {
          std::lock_guard<std::mutex> soem_lock(soem_mutex_);
          ecx_readstate(&context_);
          ECOUT << " WKC abnormal: " << wkc << "/" << expected_wkc << " "
                << describeAllSlaveStates() << "\n";
        }
        lost_frames_.fetch_add(1);   // 增加丢帧计数
        consecutive_lost_frames++;  // 增加连续丢帧计数

        ECOUT << std::setw(6) << roundtrip_time_ << " usec  WKC " << wkc;
        ECOUT << " wrong (expected " << expected_wkc
              << "), consecutive: " << consecutive_lost_frames << "\n";

        // 连续丢帧超过阈值，判定链路断开，由上层决定是否重新连接。
        if (consecutive_lost_frames >= max_lost_frames) {
          ECOUT << "Connection lost!\n";
          setLastError("Process data");
          connection_lost_.store(true);
          running_.store(false);
          break;
        }
      } else {
        // WKC正常，重置连续丢帧计数
        consecutive_lost_frames = 0;

        ECOUT << std::setw(6) << roundtrip_time_ << " usec  WKC " << wkc;
        ECOUT << "  O:";
        for (uint8_t value : output_dump) {
          ECOUT << " " << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value) << std::dec;
        }
        ECOUT << "  I:";
        for (uint8_t value : input_dump) {
          ECOUT << " " << std::hex << std::setw(2) << std::setfill('0')
                << static_cast<int>(value) << std::dec;
        }
        ECOUT << "  T: " << std::dec << dc_time << "\r" << std::flush;
      }

      if (iteration == 1) {
        min_time = max_time = roundtrip_time_;
      } else {
        if (roundtrip_time_ < min_time)
          min_time = roundtrip_time_;
        if (roundtrip_time_ > max_time)
          max_time = roundtrip_time_;
      }

      iteration++;
      osal_usleep(kProcessCycleUs);
    }

    ECOUT << "\nRoundtrip time (usec): min " << min_time << " max " << max_time
          << "\n";
  });
}

EthercatMaster::EthercatState EthercatMaster::getState() const {
  if (!running_.load() || !initialized_.load() || connection_lost_.load()) {
    return EthercatState::Disconnected;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (context_.slavecount < 1) {
    return EthercatState::Disconnected;
  }

  EthercatMaster* mutable_this = const_cast<EthercatMaster*>(this);
  ecx_readstate(&mutable_this->context_);

  // 将单个从站状态映射到 EthercatState（优先级从低到高）
  auto slave_state_to_ethercat = [](uint16_t slave_state) -> EthercatState {
    if (slave_state & EC_STATE_ERROR)
      return EthercatState::Error;
    if (slave_state == EC_STATE_NONE)
      return EthercatState::Disconnected;
    switch (slave_state) {
      case EC_STATE_INIT:
      case EC_STATE_PRE_OP:
        return EthercatState::Initializing;
      case EC_STATE_SAFE_OP:
        return EthercatState::SafeOperational;
      case EC_STATE_OPERATIONAL:
        return EthercatState::Operational;
      default:
        return EthercatState::Initializing;
    }
  };

  // 优先级顺序（数值越小越差），用于取"最差"状态
  auto state_priority = [](EthercatState s) -> int {
    switch (s) {
      case EthercatState::Error:           return 0;
      case EthercatState::Disconnected:    return 1;
      case EthercatState::Initializing:    return 2;
      case EthercatState::SafeOperational: return 3;
      case EthercatState::Operational:     return 4;
      default:                             return 0;
    }
  };

  // 遍历所有属于本 group 的从站，取最差状态
  bool found_any = false;
  EthercatState worst = EthercatState::Operational;

  constexpr int MAX_NONE_COUNT = 3;

  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_)
      continue;

    found_any = true;
    EthercatState s = slave_state_to_ethercat(slave->state);
    if (state_priority(s) < state_priority(worst))
      worst = s;
  }

  if (!found_any) {
    return EthercatState::Disconnected;
  }

  // NONE 状态防抖：连续 MAX_NONE_COUNT 次才确认断开
  if (worst == EthercatState::Disconnected) {
    none_state_count_++;
    if (none_state_count_ >= MAX_NONE_COUNT) {
      none_state_count_ = 0;
      return EthercatState::Disconnected;
    }
    return EthercatState::Initializing;
  } else {
    none_state_count_ = 0;
  }

  return worst;
}

bool EthercatMaster::isConnected() const {
  return running_.load() && initialized_.load() && started_.load() &&
         !connection_lost_.load();
}

std::vector<SlaveInfo> EthercatMaster::getSlaveInfoList() const {
  std::vector<SlaveInfo> result;

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  for (int i = 1; i <= context_.slavecount; ++i) {
    const ec_slavet* slave = context_.slavelist + i;
    if (slave->group != group_)
      continue;

    SlaveInfo info;
    info.index = i;
    info.name = std::string(slave->name);
    info.state = slave->state;
    info.al_status = slave->ALstatuscode;
    info.al_status_str =
        std::string(ec_ALstatuscode2string(slave->ALstatuscode));
    info.is_lost = slave->islost;
    result.push_back(info);
  }

  return result;
}

uint64_t EthercatMaster::getLostFrames() const {
  return lost_frames_.load();
}

void EthercatMaster::resetLostFrames() {
  lost_frames_.store(0);
}

void EthercatMaster::stop() {
  if (running_.load()) {
    ECOUT << "Stopping EtherCAT thread... " << std::flush;
    running_.store(false);
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
    ECOUT << "done\n";

    // 显示最终的丢帧统计
    ECOUT << "Total lost frames: " << lost_frames_.load() << "\n";
  }

  {
    std::lock_guard<std::mutex> soem_lock(soem_mutex_);
    if (started_.load()) {
      if (!connection_lost_.load() && context_.slavecount > 0) {
        ec_slavet* slave = context_.slavelist;
        ECOUT << "Requesting init state on all slaves... " << std::flush;
        slave->state = EC_STATE_INIT;
        ecx_writestate(&context_, 0);
        ECOUT << "done\n";
      }
      started_.store(false);
    }

    if (initialized_.load()) {
      ECOUT << "Close socket... " << std::flush;
      ecx_close(&context_);
      ECOUT << "done\n";
      initialized_.store(false);
    }
  }

  {
    std::lock_guard<std::mutex> io_lock(io_mutex_);
    outputs_ = nullptr;
    inputs_ = nullptr;
    output_bytes_ = 0;
    input_bytes_ = 0;
  }
  output_buffer_.resize(0);
  io_map_.clear();
  slave_io_info_.clear();
  connected_index_ = -1;
  none_state_count_ = 0;
  resetLostFrames();
  connection_lost_.store(false);
}

void EthercatMaster::setOutput(int slave_id, int index, uint8_t value) {
  if (!started_.load() || connection_lost_.load()) return;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->output_bytes == 0 || info->output_offset < 0) return;
  if (index < 0 || index >= info->output_bytes) return;
  output_buffer_.write_byte(static_cast<size_t>(info->output_offset + index), value);
}

uint8_t EthercatMaster::getInput(int slave_id, int index) {
  if (!started_.load() || connection_lost_.load() || inputs_ == nullptr) return 0;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->input_bytes == 0 || info->input_offset < 0) return 0;
  if (index < 0 || index >= info->input_bytes) return 0;
  std::lock_guard<std::mutex> lock(io_mutex_);
  return inputs_[info->input_offset + index];
}

// ---------------------------------------------------------------------------
// 按从站粒度的 PDO 操作
// ---------------------------------------------------------------------------

const SlaveIOInfo* EthercatMaster::findSlaveInfo(int slave_id) const {
  for (const auto& info : slave_io_info_) {
    if (info.slave_id == slave_id) return &info;
  }
  return nullptr;
}

int EthercatMaster::getSlaveCount() const {
  return static_cast<int>(slave_io_info_.size());
}

int EthercatMaster::getSlaveOutputSize(int slave_id) const {
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  return info ? info->output_bytes : 0;
}

int EthercatMaster::getSlaveInputSize(int slave_id) const {
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  return info ? info->input_bytes : 0;
}

bool EthercatMaster::setSlaveOutputs(int slave_id, const uint8_t* data,
                                     unsigned int len) {
  if (!started_.load() || connection_lost_.load() || !data || len == 0) return false;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->output_bytes == 0 || info->output_offset < 0) {
    ECOUT << "setSlaveOutputs: slave " << slave_id << " has no outputs\n";
    return false;
  }
  if (len > (unsigned int)info->output_bytes) {
    ECOUT << "setSlaveOutputs: len " << len << " exceeds slave " << slave_id
          << " output bytes " << info->output_bytes << "\n";
    return false;
  }
  return output_buffer_.write_slice(
      static_cast<size_t>(info->output_offset), data, len);
}

bool EthercatMaster::getSlaveInputs(int slave_id, uint8_t* buffer,
                                    unsigned int len) const {
  if (!started_.load() || connection_lost_.load() || !buffer || len == 0) return false;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->input_bytes == 0 || info->input_offset < 0) {
    ECOUT << "getSlaveInputs: slave " << slave_id << " has no inputs\n";
    return false;
  }
  if (len > (unsigned int)info->input_bytes) {
    ECOUT << "getSlaveInputs: len " << len << " exceeds slave " << slave_id
          << " input bytes " << info->input_bytes << "\n";
    return false;
  }
  std::lock_guard<std::mutex> lock(io_mutex_);
  std::memcpy(buffer, inputs_ + info->input_offset, len);
  return true;
}

bool EthercatMaster::getSlaveOutputs(int slave_id, uint8_t* buffer,
                                     unsigned int len) const {
  if (!started_.load() || connection_lost_.load() || !buffer || len == 0) return false;
  const SlaveIOInfo* info = findSlaveInfo(slave_id);
  if (!info || info->output_bytes == 0 || info->output_offset < 0) {
    ECOUT << "getSlaveOutputs: slave " << slave_id << " has no outputs\n";
    return false;
  }
  if (len > (unsigned int)info->output_bytes) {
    ECOUT << "getSlaveOutputs: len " << len << " exceeds slave " << slave_id
          << " output bytes " << info->output_bytes << "\n";
    return false;
  }
  return output_buffer_.read_slice(
      static_cast<size_t>(info->output_offset), buffer, len);
}

// ---------------------------------------------------------------------------
// SDO 读写
// ---------------------------------------------------------------------------

bool EthercatMaster::sdoRead(uint16_t index, uint8_t subindex,
                             uint32_t* value, int slave_id) {
  if (!value) {
    return false;
  }

  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (!initialized_.load()) {
    ECOUT << "SOEM not initialized\n";
    return false;
  }

  if (slave_id < 1 || slave_id > context_.slavecount) {
    ECOUT << "Invalid slave_id " << slave_id << " (count="
          << context_.slavecount << ")\n";
    return false;
  }
  uint8_t buffer[4] = {};
  int size = sizeof(buffer);

  int result = ecx_SDOread(&context_, slave_id, index, subindex, false, &size,
                           buffer, EC_TIMEOUTRXM * 3);

  // 检查执行结果
  if (result <= 0) {
    ECOUT << "SDO Read [Slave " << slave_id << " 0x" << std::setfill('0')
          << std::setw(4) << std::hex << index << ":" << std::setw(2)
          << static_cast<int>(subindex) << "] failed. Error: " << result
          << "\n";
    return false;
  }
  if (size < static_cast<int>(sizeof(uint32_t))) {  // 检查实际读取的数据长度
    ECOUT << "SDO data too short (Expected 4, got " << size << ")\n";
    return false;
  }

  // 拷贝并转换字节序（小端→主机序）
  *value = (static_cast<uint32_t>(buffer[3]) << 24) |
           (static_cast<uint32_t>(buffer[2]) << 16) |
           (static_cast<uint32_t>(buffer[1]) << 8) | buffer[0];

  return true;
}

bool EthercatMaster::sdoWrite(uint16_t index, uint8_t subindex,
                              uint32_t value, int slave_id) {
  std::lock_guard<std::mutex> soem_lock(soem_mutex_);
  if (!initialized_.load()) {
    ECOUT << "SOEM not initialized\n";
    return false;
  }

  if (slave_id < 1 || slave_id > context_.slavecount) {
    ECOUT << "Invalid slave_id " << slave_id << " (count="
          << context_.slavecount << ")\n";
    return false;
  }
  uint8_t buffer[4];

  // 主机序转小端字节序（适用于EtherCAT设备）
  buffer[0] = static_cast<uint8_t>(value & 0xFF);  // LSB
  buffer[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
  buffer[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
  buffer[3] = static_cast<uint8_t>((value >> 24) & 0xFF);  // MSB

  int size = sizeof(buffer);
  int result = ecx_SDOwrite(&context_, slave_id, index, subindex, FALSE, size,
                            buffer, EC_TIMEOUTRXM * 3);

  // 检查执行结果（ecx_SDOwrite返回1表示成功）
  if (result != 1) {
    ECOUT << "SDO Write [Slave " << slave_id << " 0x" << std::setfill('0')
          << std::setw(4) << std::hex << index << ":" << std::setw(2)
          << static_cast<int>(subindex) << "] failed. Error: " << result
          << " (Value: 0x" << std::setw(8) << value << ")\n";
    return false;
  }

  return true;
}
