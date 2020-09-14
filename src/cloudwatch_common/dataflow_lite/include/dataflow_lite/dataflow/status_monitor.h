/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#pragma once

#include <atomic>
#include <condition_variable> // std::condition_variable
#include <mutex>              // std::mutex, std::unique_lock
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace Aws {
namespace DataFlow {

/**
 * Status.
 */
enum Status : uint {
  UNAVAILABLE = 0,
  AVAILABLE
};

class MultiStatusConditionMonitor;

/**
 * Handle a single set/get status. Notify the multi_status_condition when the status is changed.
 */
class StatusMonitor {
public:
  virtual ~StatusMonitor() = default;
  void setStatus(const Status &status);

  inline Status getStatus() const {
    return status_;
  }
private:
  friend MultiStatusConditionMonitor;
  inline void setStatusObserver(MultiStatusConditionMonitor *multi_status_cond) {
    multi_status_cond_ = multi_status_cond;
  }
  Status status_ = UNAVAILABLE;
  MultiStatusConditionMonitor *multi_status_cond_ = nullptr;
};

class MaskFactory {
public:

  /**
   * Generate a new mask that is no longer in use by this factor.
   *
   * @return mask
   */
  uint64_t getNewMask() {
    uint64_t current_mask = 0, new_mask;
    size_t shift = 0;
    while (current_mask == 0) {
      new_mask = static_cast<uint64_t>(1) << shift++;
      current_mask = !(collective_mask_ & new_mask) ? new_mask : 0;
      if (shift > max_size) {
        throw std::overflow_error("No more masks available");
      }
    }
    collective_mask_ |= current_mask;
    return current_mask;
  }

  /**
   * Remove a mask from use.
   *
   * @param mask to remove
   */
  void removeMask(uint64_t mask) {
    collective_mask_ &= ~mask;
  }

  /**
   * @return Get the collective mask.
   */
  uint64_t getCollectiveMask() const {
    return collective_mask_;
  }
private:
  static constexpr size_t max_size = sizeof(uint64_t) * 8;
  uint64_t collective_mask_ = 0;
};

class ThreadMonitor {
public:
  virtual ~ThreadMonitor() = default;
  void waitForWork();
  std::cv_status waitForWork(const std::chrono::microseconds &duration);
  void notify();
private:
  virtual bool hasWork() = 0;
  std::mutex idle_mutex_;
  std::condition_variable work_condition_;
};

/**
 * Multi Status Condition Monitor listens to N StatusMonitors and determines whether to trigger wait for work
 * based on the hasWork() function.
 */
class MultiStatusConditionMonitor : public ThreadMonitor {
public:
  MultiStatusConditionMonitor() {
    mask_ = 0;
  }
  ~MultiStatusConditionMonitor() override = default;
  void addStatusMonitor(std::shared_ptr<StatusMonitor> &status_monitor);
protected:
  friend StatusMonitor;
  virtual void setStatus(const Status &status, StatusMonitor *status_monitor);
  bool hasWork() override;
  MaskFactory mask_factory_;
  std::atomic<uint64_t> mask_{};
  std::unordered_map<StatusMonitor*, uint64_t> status_monitors_;
};

}  // namespace DataFlow
}  // namespace Aws
