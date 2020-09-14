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

#include <deque>
#include <functional>
#include <mutex>
#include <semaphore.h>

#include <dataflow_lite/dataflow/sink.h>
#include <dataflow_lite/dataflow/source.h>
#include <dataflow_lite/dataflow/status_monitor.h>

namespace Aws {
namespace DataFlow {

template<
  class T,
  class Allocator = std::allocator<T>>
class IObservedQueue:
  public Sink<T>,
  public Source<T>
{
public:
  virtual bool empty() const = 0;
  virtual size_t size() const = 0;
  virtual void setStatusMonitor(std::shared_ptr<StatusMonitor> status_monitor) = 0;
};

/**
 * An observed queue is a dequeue wrapper which notifies an observer when a task is added.
 *
 * @tparam T type of data
 * @tparam Allocator
 */
template<
  class T,
  class Allocator = std::allocator<T>>
class ObservedQueue :
  public IObservedQueue<T, Allocator> {
public:

  ~ObservedQueue() override = default;

  /**
   * Set the observer for the queue.
   *
   * @param status_monitor
   */
  inline void setStatusMonitor(std::shared_ptr<StatusMonitor> status_monitor) override {
    status_monitor_ = status_monitor;
  }

  /**
   * Enqueue data and notify the observer of data available.
   *
   * @param value to enqueue
   */
  inline bool enqueue(T&& value) override {
    dequeue_.push_back(value);
    notifyMonitor(AVAILABLE);
    return true;
  }

  /**
   * Enqueue data and notify the observer of data available.
   *
   * @param value to enqueue
   */
  inline bool enqueue(T& value) override {
    dequeue_.push_back(value);
    notifyMonitor(AVAILABLE);
    return true;
  }

  inline bool tryEnqueue(
      T& value,
      const std::chrono::microseconds&) override
  {
    return enqueue(value);
  }

  inline bool tryEnqueue(
      T&& value,
      const std::chrono::microseconds&) override
  {
    return enqueue(value);
  }

  /**
   * Dequeue data and notify the observer of data unavailable if the queue is empty.
   *
   * @return the front of the dequeue
   */
  inline bool dequeue(
    T& data,
    const std::chrono::microseconds&) override
  {
    bool is_data = false;
    if (!dequeue_.empty()) {
      data = dequeue_.front();
      dequeue_.pop_front();
      is_data = true;
      if (dequeue_.empty()) {
        notifyMonitor(UNAVAILABLE);
      }
    }
    return is_data;
  }

  /**
   * @return true if the queue is empty
   */
  inline bool empty() const override {
    return dequeue_.empty();
  }

  /**
   * @return the size of the queue
   */
  inline size_t size() const override {
    return dequeue_.size();
  }

  /**
   * Clear the dequeue
   */
  void clear() override {
    dequeue_.clear();
  }

protected:

  /**
   * Notify the monitor if it exists.
   *
   * @param status the status to notify the monitor of.
   */
  void notifyMonitor(const Status &status) {
    if (status_monitor_) {
      status_monitor_->setStatus(status);
    }
  }

  /**
   * The status monitor observer.
   */
  std::shared_ptr<StatusMonitor> status_monitor_;

  /**
   * The dequeue to store data.
   */
  std::deque<T, Allocator> dequeue_;

};

/**
 * Adds basic thread safety to the ObservedQueue.
 *
 * @tparam T
 * @tparam Allocator
 */
template<
    class T,
    class Allocator = std::allocator<T>>
class ObservedSynchronizedQueue : public ObservedQueue<T, Allocator> {
public:
  ~ObservedSynchronizedQueue() override = default;

  /**
   * Enqueue data and notify the observer of data available.
   *
   * @param value to enqueue
   */
  inline bool enqueue(T&& value) override {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_);
    return OQ::enqueue(std::move(value));
  }

  /**
   * Enqueue data and notify the observer of data available.
   *
   * @param value to enqueue
   */
  inline bool enqueue(T& value) override {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_);
    return OQ::enqueue(value);
  }

  inline bool tryEnqueue(
      T& value,
      const std::chrono::microseconds &duration) override
  {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_, std::defer_lock);
    bool result = lock.try_lock_for(duration);
    if (result) {
      OQ::enqueue(value);
    }
    return result;
  }

  inline bool tryEnqueue(
      T&& value,
      const std::chrono::microseconds &duration) override
  {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_, std::defer_lock);
    bool result = lock.try_lock_for(duration);
    if (result) {
      OQ::enqueue(std::move(value));
    }
    return result;
  }

  /**
   * Dequeue data and notify the observer of data unavailable if the queue is empty.
   *
   * @return the front of the dequeue
   */
  inline bool dequeue(
      T& data,
      const std::chrono::microseconds &duration) override
  {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_, std::defer_lock);
    bool result = lock.try_lock_for(duration);
    if (result) {
      result = OQ::dequeue(data, duration);
    }
    return result;
  }

  /**
   * @return true if the queue is empty
   */
  inline bool empty() const override {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_);
    return OQ::empty();
  }

  /**
   * @return the size of the queue
   */
  inline size_t size() const override {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_);
    return OQ::size();
  }

  /**
   * Clear the dequeue
   */
  void clear() override {
    std::unique_lock<DequeueMutex> lock(dequeue_mutex_);
    OQ::clear();
  }

private:
  using OQ = ObservedQueue<T, Allocator>;
  // @todo (rddesmon): Dual semaphore for read optimization
  using DequeueMutex = std::timed_mutex;
  mutable DequeueMutex dequeue_mutex_;
};
/**
 * An observed queue is a dequeue wrapper which notifies an observer when a task is added.
 *
 * @tparam T type of data
 * @tparam Allocator
 */
template<
  class T,
  class Allocator = std::allocator<T>>
class ObservedBlockingQueue : public ObservedQueue<T, Allocator> {
public:

  /**
   * Create an observed blocking queue.
   *
   * @param max_queue_size to configure.
   * @throws std::invalid_argument max_queue_size is 0
   */
  explicit ObservedBlockingQueue(const size_t &max_queue_size) {
    if (max_queue_size == 0) {
      throw std::invalid_argument("Max queue size invalid: 0");
    }
    max_queue_size_ = max_queue_size;
  }

  ~ObservedBlockingQueue() override = default;
  /**
   * Enqueue data and notify the observer of data available.
   *
   * @param value to enqueue
   */
  inline bool enqueue(T&& value) override
  {
    bool is_queued = false;
    std::unique_lock<std::mutex> lk(dequeue_mutex_);
    if (OQ::size() <= max_queue_size_) {
      OQ::enqueue(value);
      is_queued = true;
    }
    return is_queued;
  }

  inline bool enqueue(T& value) override {
    bool is_queued = false;
    std::unique_lock<std::mutex> lk(dequeue_mutex_);
    if (OQ::size() <= max_queue_size_) {
      OQ::enqueue(value);
      is_queued = true;
    }
    return is_queued;
  }

  /**
   * Blocking call.
   *
   * @param value
   * @param duration
   * @return
   */
  inline bool tryEnqueue(
    T& value,
    const std::chrono::microseconds &duration) override
  {
    std::cv_status (std::condition_variable::*wf)(std::unique_lock<std::mutex>&, const std::chrono::microseconds&);
    wf = &std::condition_variable::wait_for;
    return enqueueOnCondition(
      value,
      std::bind(wf, &condition_variable_, std::placeholders::_1, duration));
  }

  inline bool tryEnqueue(
      T&& value,
      const std::chrono::microseconds &duration) override
  {
    std::cv_status (std::condition_variable::*wf)(std::unique_lock<std::mutex>&, const std::chrono::microseconds&);
    wf = &std::condition_variable::wait_for;
    return enqueueOnCondition(
      value,
      std::bind(wf, &condition_variable_, std::placeholders::_1, duration));
  }

  /**
   * Dequeue data and notify the observer of data unavailable if the queue is empty.
   *
   * @return the front of the dequeue
   */
  inline bool dequeue(T& data, const std::chrono::microseconds &duration) override {
    auto is_retrieved = OQ::dequeue(data, duration);
    if (is_retrieved) {
      std::unique_lock<std::mutex> lck(dequeue_mutex_);
      condition_variable_.notify_one();
    }
    return is_retrieved;
  }

  /**
   * @return true if the queue is empty
   */
  inline bool empty() const override {
    std::lock_guard<std::mutex> lock(dequeue_mutex_);
    return OQ::empty();
  }

  /**
   * @return the size of the queue
   */
  inline size_t size() const override {
    std::lock_guard<std::mutex> lock(dequeue_mutex_);
    return OQ::size();
  }

  /**
   * Clear the dequeue
   */
  void clear() override {
    std::lock_guard<std::mutex> lock(dequeue_mutex_);
    OQ::clear();
  }

private:
  using OQ = ObservedQueue<T, Allocator>;
  using WaitFunc = std::function <std::cv_status (std::unique_lock<std::mutex>&)>;

  /**
   * Static wait function which returns no_timeout on completion.
   *
   * @param condition_variable
   * @param lock
   * @return std::cv_status::no_timeout
   */
  static std::cv_status wait(
    std::condition_variable &condition_variable,
    std::unique_lock<std::mutex> &lock)
  {
    condition_variable.wait(lock);
    return std::cv_status::no_timeout;
  }

  /**
   * Enqueue on the condition variable.
   *
   * @param value to enqueue
   * @param wait_func to wait for availability
   * @return true if the value was enqueued
   */
  inline bool enqueueOnCondition(T& value,
    const WaitFunc &wait_func)
  {
    std::unique_lock<std::mutex> lk(dequeue_mutex_);
    bool can_enqueue = true;
    if (OQ::size() >= max_queue_size_) {
      can_enqueue = wait_func(lk) == std::cv_status::no_timeout;
    }
    if (can_enqueue) {
      OQ::enqueue(value);
    }
    return can_enqueue;
  }

  size_t max_queue_size_;
  std::condition_variable condition_variable_;
  mutable std::mutex dequeue_mutex_;
};

}  // namespace DataFlow
}  // namespace Aws