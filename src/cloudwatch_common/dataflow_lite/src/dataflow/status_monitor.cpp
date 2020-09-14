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

#include <numeric>

#include <dataflow_lite/dataflow/status_monitor.h>

namespace Aws {
namespace DataFlow {

void StatusMonitor::setStatus(const Status &status) {
  status_ = status;
  if (multi_status_cond_) {
    multi_status_cond_->setStatus(status, this);
  }
}

void ThreadMonitor::notify() {
  if (hasWork()) {
    std::lock_guard<std::mutex> lck(idle_mutex_);
    work_condition_.notify_one();
  }
}

void ThreadMonitor::waitForWork() {
  if (!hasWork()) {
    std::unique_lock<std::mutex> lck(idle_mutex_);
    work_condition_.wait(lck);
  }
}

std::cv_status ThreadMonitor::waitForWork(const std::chrono::microseconds& duration) {
  std::cv_status status = std::cv_status::no_timeout;
  if (!hasWork()) {
    std::unique_lock<std::mutex> lck(idle_mutex_);
    status = work_condition_.wait_for(lck, duration);
  }
  return status;
}

void MultiStatusConditionMonitor::addStatusMonitor(
  std::shared_ptr<StatusMonitor> &status_monitor)
{
  if (status_monitor) {
    status_monitor->setStatusObserver(this);
    status_monitors_[(status_monitor.get())] = mask_factory_.getNewMask();
    setStatus(status_monitor->getStatus(), status_monitor.get());
  }
}

void MultiStatusConditionMonitor::setStatus(
  const Status &status, StatusMonitor *status_monitor) {
  if (status == Status::AVAILABLE) {
    mask_ |= status_monitors_[status_monitor];
  } else {
    mask_ &= ~status_monitors_[status_monitor];
  }
  notify();
}

bool MultiStatusConditionMonitor::hasWork() {
  return mask_factory_.getCollectiveMask() == mask_;
}

}  // namespace DataFlow
}  // namespace Aws
