// EthercatMaster.h
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include <cstring>

extern "C" {
#include <soem/soem.h>
}

struct SlaveInfo {
  int index;
  std::string name;
  uint16_t state;
  uint16_t al_status;
  std::string al_status_str;
  bool is_lost;

  std::string toString() const;
};

std::ostream& operator<<(std::ostream& os, const SlaveInfo& info);

// 三层 OutputBuffer：pending_ → flush_locked → committed_[A/B]
// 用户线程持 pending_mtx_，comm thread 的 consume() 仅用原子操作（零 mutex 开销）
struct OutputBuffer {
  std::vector<uint8_t> pending_;
  mutable std::mutex   pending_mtx_;
  std::vector<uint8_t> committed_[2];
  std::atomic<int>     latest_idx_{0};
  std::atomic<bool>    dirty_{false};

  void resize(size_t size) {
    std::lock_guard<std::mutex> lock(pending_mtx_);
    pending_.assign(size, 0);
    committed_[0].assign(size, 0);
    committed_[1].assign(size, 0);
    latest_idx_.store(0, std::memory_order_release);
    dirty_.store(false, std::memory_order_release);
  }

  // 全量写（覆盖 pending_ 前 len 字节，然后提交快照）
  bool write(const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(pending_mtx_);
    if (!data || len == 0 || len > pending_.size()) return false;
    std::memcpy(pending_.data(), data, len);
    flush_locked();
    return true;
  }

  // 分片写（修改 pending_ 中 [off, off+len) 范围，然后提交快照）
  bool write_slice(size_t off, const uint8_t* data, size_t len) {
    std::lock_guard<std::mutex> lock(pending_mtx_);
    if (!data || len == 0 || off + len > pending_.size()) return false;
    std::memcpy(pending_.data() + off, data, len);
    flush_locked();
    return true;
  }

  // 单字节写
  bool write_byte(size_t idx, uint8_t val) {
    std::lock_guard<std::mutex> lock(pending_mtx_);
    if (idx >= pending_.size()) return false;
    pending_[idx] = val;
    flush_locked();
    return true;
  }

  // 全量读 pending_（反映最新写入意图）
  bool read_all(uint8_t* out, size_t len) const {
    std::lock_guard<std::mutex> lock(pending_mtx_);
    if (!out || len == 0 || len > pending_.size()) return false;
    std::memcpy(out, pending_.data(), len);
    return true;
  }

  // 分片读 pending_
  bool read_slice(size_t off, uint8_t* out, size_t len) const {
    std::lock_guard<std::mutex> lock(pending_mtx_);
    if (!out || len == 0 || off + len > pending_.size()) return false;
    std::memcpy(out, pending_.data() + off, len);
    return true;
  }

  // comm thread 专用：仅原子操作，返回 committed 指针（无新数据返回 nullptr）
  const uint8_t* consume() {
    if (!dirty_.exchange(false, std::memory_order_acquire)) return nullptr;
    return committed_[latest_idx_.load(std::memory_order_acquire)].data();
  }

 private:
  // 在 pending_mtx_ 保护下将 pending_ 快照写入下一个 committed_ 槽
  void flush_locked() {
    int cur  = latest_idx_.load(std::memory_order_relaxed);
    int next = 1 - cur;
    std::memcpy(committed_[next].data(), pending_.data(), pending_.size());
    latest_idx_.store(next, std::memory_order_release);
    dirty_.store(true, std::memory_order_release);
  }
};

struct SlaveIOInfo {
  int         slave_id;       // 1-based SOEM 从站编号
  std::string name;
  int         output_offset;  // 在 grp->outputs 中的字节偏移（无输出则 -1）
  int         output_bytes;   // 0 = 无输出
  int         input_offset;   // 在 grp->inputs 中的字节偏移（无输入则 -1）
  int         input_bytes;    // 0 = 无输入
};

class EthercatMaster {
 public:
  enum class EthercatState {
    Disconnected,     // 未初始化或已停止
    Initializing,     // init 成功，但未 start
    SafeOperational,  // 进入 SAFE_OP
    Operational,      // 正常运行（OPERATIONAL）
    Error             // 有从站报错
  };

  EthercatMaster();
  ~EthercatMaster();

  std::vector<std::string> scanNetworkInterfaces();
  bool probeSlaves(int index, std::vector<SlaveInfo>& slaves);
  std::string getLastError() const;
  bool init(int index);
  bool start();
  void stop();
  void run();  // 启动子线程（非阻塞）
  bool isConnected() const;

  // 获取从站状态
  EthercatState getState() const;

  // 获取所有从站信息列表
  std::vector<SlaveInfo> getSlaveInfoList() const;

  // 获取丢帧统计
  uint64_t getLostFrames() const;

  // 重置丢帧统计
  void resetLostFrames();

  // 单字节操作
  void    setOutput(int slave_id, int index, uint8_t value);
  uint8_t getInput(int slave_id, int index);

  // PDO 批量操作
  int  getSlaveCount() const;
  int  getSlaveOutputSize(int slave_id) const;
  int  getSlaveInputSize(int slave_id) const;
  bool setSlaveOutputs(int slave_id, const uint8_t* data, unsigned int len);
  bool getSlaveInputs(int slave_id, uint8_t* buffer, unsigned int len) const;
  bool getSlaveOutputs(int slave_id, uint8_t* buffer, unsigned int len) const;

  // SDO 读写: 对象字典 (index, subindex)
  bool sdoRead(uint16_t index, uint8_t subindex, uint32_t* value,
               int slave_id);
  bool sdoWrite(uint16_t index, uint8_t subindex, uint32_t value,
                int slave_id);

 private:
  ecx_contextt context_;                 // SOEM context
  std::vector<std::string> interfaces_;  // 扫描到的网卡名
  std::string iface_;                    // 网卡名
  uint8 group_;                          // 组号
  int roundtrip_time_;                   // 往返时间
  std::vector<uint8_t> io_map_;          // I/O 映射缓冲区
  std::atomic<bool> initialized_;
  std::atomic<bool> started_;
  int connected_index_;

  // 多线程控制
  std::atomic<bool> running_;
  std::thread worker_thread_;
  mutable std::mutex soem_mutex_;  // 保护 SOEM context 访问
  mutable std::mutex io_mutex_;  // 保护 outputs/inputs 访问

  // 缓存指针
  OutputBuffer output_buffer_;  // 双缓冲实现写入
  uint8* outputs_;
  uint8* inputs_;
  int output_bytes_;
  int input_bytes_;

  std::atomic<uint64_t> lost_frames_{0};  // 丢帧计数器
  std::atomic<bool> connection_lost_{false};
  mutable int none_state_count_{0};
  mutable std::mutex error_mutex_;
  std::string last_error_;

  // 每从站 PDO 布局信息（init() 后只读，无需锁）
  std::vector<SlaveIOInfo> slave_io_info_;
  const SlaveIOInfo* findSlaveInfo(int slave_id) const;

  void clearLastError();
  void setLastError(const std::string& message);
  int getExpectedWkc() const;
  std::string describeSlaveState(int slave_index) const;
  std::string describeAllSlaveStates() const;
  void dumpSoemErrors(const std::string& phase);
  void dumpSlaveDiagnostics(const std::string& phase);
  void dumpMappingDiagnostics() const;
  void applyLslqDh116EsiDefaults();
  void dumpSiiPdoDiagnostics();
  bool readSdoValue(uint16_t slave, uint16_t index, uint8_t subindex,
                    int max_size, uint32_t* value);
  void verifySdoValue(const std::string& name, uint16_t index,
                      uint8_t subindex, uint32_t expected, int max_size,
                      int* mismatch_count);
  void verifyPdoMapping(const std::string& name, uint16_t pdo_index,
                        uint16_t object_index, uint8_t expected_entries,
                        int* mismatch_count);
  void verifyEsiConfiguration();
  void verifyMappedEsiConfiguration();
};
