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

#include <vector>

#include <dataflow_lite/dataflow/status_monitor.h>
#include <dataflow_lite/dataflow/observed_queue.h>
#include <dataflow_lite/dataflow/pipeline.h>
#include <dataflow_lite/dataflow/priority_options.h>

namespace Aws {
namespace DataFlow {

template <typename T>
class QueueDemux {
public:
  virtual ~QueueDemux() = default;
  virtual void addSource(std::shared_ptr<IObservedQueue < T>>, PriorityOptions) = 0;
};

/**
 * Manage multiple queue's and their priorities.
 * Exposes a dequeue API which enforces the desired priorities and returns the data with the highest priority.
 *
 * @tparam T type of data in the queues.
 */
template <typename T>
class QueueMonitor :
  public QueueDemux<T>,
  public MultiStatusConditionMonitor,
  public Source<T>
{
public:
  QueueMonitor() = default;
  ~QueueMonitor() override = default;

  inline void addSource(
    std::shared_ptr<IObservedQueue < T>>observed_queue,
    PriorityOptions priority_options) override
  {
    auto status_monitor = std::make_shared<StatusMonitor>();
    addStatusMonitor(status_monitor);
    observed_queue->setStatusMonitor(status_monitor);
    priority_vector_.push_back(QueuePriorityPair(observed_queue, priority_options));
    std::sort(priority_vector_.begin(), priority_vector_.end(), std::greater<QueuePriorityPair>());
  }

  /**
   * Dequeue data off of a queue with the highest priority.
   *
   * @return the dequeue'd data
   */
  inline bool dequeue(
    T& data,
    const std::chrono::microseconds &duration) override
{
    ThreadMonitor::waitForWork(duration);
    bool is_dequeued = false;
    for (auto &queue : priority_vector_)
    {
      is_dequeued = queue.observed_queue->dequeue(data, std::chrono::microseconds(0));
      if (is_dequeued)
      {
        break;
      }
    }
    return is_dequeued;
  }

protected:
  /**
   * @return True if any of the status monitors are enabled.
   */
  inline bool hasWork() override {
    return static_cast<bool>(mask_);
  }

private:

  /**
   * Private class for handling priority options and queues.
   */
  struct QueuePriorityPair {
    std::shared_ptr<IObservedQueue<T>> observed_queue;
    PriorityOptions priority_options;

    explicit QueuePriorityPair(
      std::shared_ptr<IObservedQueue<T>> queue,
      PriorityOptions options)
    {
      observed_queue = queue;
      priority_options = options;
    }

    inline bool operator > (const QueuePriorityPair &pair) const {
      return priority_options > pair.priority_options;
    }

    inline bool operator < (const QueuePriorityPair &pair) const {
      return priority_options < pair.priority_options;
    }
  };

  /**
   * Priority queue of managed shared queues.
   */
  std::vector<QueuePriorityPair> priority_vector_;
};

}  // namespace DataFlow
}  // namespace Aws