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
#include <chrono>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <stdexcept>

#include <dataflow_lite/utils/service.h>

/**
 * Abstract class used to define a batching interface.
 *
 * @tparam T the type of data to be batched.
 */
template<typename T>
class DataBatcher : public Service {
public:

  /**
   * SIZE_MAX is used as the NOT set default.
   */
  static const size_t kDefaultTriggerSize = SIZE_MAX;
  static const size_t kDefaultMaxBatchSize = 1024;

  /**
   * Create a DataBatcher instance
   *
   * @param max_allowable_batch_size if this limit is reached then the queue is emptied via the emptyCollection method
   * @param trigger_size if this limit is reached then the queue is emptied via the publish method
   * @param try_enqueue_duration maximum amount of time to attempt to empty queue during the publish method
   */
  // NOLINTNEXTLINE(google-explicit-constructor, hicpp-explicit-conversions)
  DataBatcher(size_t max_allowable_batch_size = DataBatcher::kDefaultMaxBatchSize,
              size_t trigger_size = DataBatcher::kDefaultTriggerSize,
              std::chrono::microseconds try_enqueue_duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::seconds(2))) {

    validateConfigurableSizes(max_allowable_batch_size, trigger_size);

    this->max_allowable_batch_size_.store(max_allowable_batch_size);
    this->trigger_batch_size_.store(trigger_size);
    this->try_enqueue_duration_ = try_enqueue_duration;

    resetBatchedData();
  }

  /**
   * Destruct a DataBatcher instance
   */
  ~DataBatcher() override = default;

  /**
   * Batch an item.
   *
   * @param data_to_batch
   * @return true if the data was accepted, false if internal size limit was exceeded
   */
  virtual bool batchData(const T &data_to_batch) {
    std::lock_guard<std::recursive_mutex> lk(mtx);

    this->batched_data_->push_back(data_to_batch);

    // check if we have exceeded the allowed bounds
    auto allowed_max = getMaxAllowableBatchSize();
    if (getCurrentBatchSize() > allowed_max) {
      emptyCollection();
      return false;
    }

    // publish if the size has been configured
    auto mbs = this->getTriggerBatchSize();
    if (mbs != kDefaultTriggerSize && this->batched_data_->size() >= mbs) {
      publishBatchedData(); // don't return publisher success / fail here
    }

    return true;
  }

  /**
   * Return the number of currently batched items.
   * @return
   */
  size_t getCurrentBatchSize() {
    std::lock_guard<std::recursive_mutex> lk(mtx);

    return this->batched_data_->size();
  }

  /**
   * Reset the batched data shared pointer.
   */
  void resetBatchedData() {
    std::lock_guard<std::recursive_mutex> lk(mtx);

    this->batched_data_ = std::make_shared<std::list<T>>();
  }

  /**
   * Set the trigger value, which will call the publish method when the number of batched items is greater than this
   * limit. Note: this value must be strictly less than the max_allowable_batch_size_.
   * @param new_value
   */
  void setTriggerBatchSize(size_t new_value) {
    validateConfigurableSizes(this->max_allowable_batch_size_, new_value);

    this->trigger_batch_size_.store(new_value);
  }

  /**
   * Return the trigger value. Note: this is set to kDefaultTriggerSize = SIZE_MAX by default, which means the
   * trigger is not used and publish must be called manually.
   *
   * @return
   */
  size_t getTriggerBatchSize() {
    return this->trigger_batch_size_.load();

  }

  /**
   * Return the maximum allowable batch size. When this limit is reached the queue is emptied via the emptyCollection
   * method.
   *
   * @return
   */
  size_t getMaxAllowableBatchSize() {
    return this->max_allowable_batch_size_.load();
  }

  /**
   * Set the maximum allowable batch size. When this limit is reached the queue is emptied via the emptyCollection
   * method.
   *
   * @param max_allowable_batch_size
   */
  void setMaxAllowableBatchSize(int new_value) {

    validateConfigurableSizes(new_value, this->trigger_batch_size_);

    this->max_allowable_batch_size_.store(new_value);
  }

  /**
   * Reset the trigger batch size to the default value, which means the mechanism is no longer set. Publish will
   * need to be called manually.
   */
  void resetTriggerBatchSize() {
    this->trigger_batch_size_.store(kDefaultTriggerSize);
  }

  /**
   * Set the maximum amount of time publish can attempt to empty the queue.
   *
   * @param duration
   */
  void setTryEnqueueDuration(std::chrono::microseconds duration) {
    this->try_enqueue_duration_.store(duration);
  }

  /**
   * Get the current try_enqueue_duration_.
   *
   * @return
   */
  std::chrono::microseconds getTryEnqueueDuration() {
    return this->try_enqueue_duration_.load();
  }

  /**
   * When the getTriggerBatchSize is met this method will be called.
   *
   * @return
   */
  virtual bool publishBatchedData() = 0;

  /**
   *
   * @throws invalid argument if the batch_trigger_publish_size is strictly greater than max_allowable_batch_size
   * @param batch_max_queue_size
   * @param batch_trigger_publish_size
   */
  static void validateConfigurableSizes(size_t batch_max_queue_size, size_t batch_trigger_publish_size) {

    if (0 == batch_max_queue_size || 0 == batch_trigger_publish_size) {
      throw std::invalid_argument("0 is not a valid size");
    }

    if(kDefaultTriggerSize != batch_trigger_publish_size && batch_trigger_publish_size >= batch_max_queue_size) {
      throw std::invalid_argument("batch_trigger_publish_size must be less than batch_max_queue_size");
    }
  }

  /**
   * Shutdown the batcher: this blocks until publish has completed in order to attempt to empty any unpublished data.
   * @return the result of Service::shutdown()
   */
  bool shutdown() override {
    bool is_shutdown = Service::shutdown();
    std::lock_guard<std::recursive_mutex> lk(mtx);
    this->emptyCollection();  // attempt to write to disk before discarding
    return is_shutdown;
  }

protected:

  /**
   * Safeguard in case the handleTriggerSize method does not properly drain the collection. This will clear the
   * currently batched data. If other behavior is desired to empty the collection implementing classes should override.
   *
   */
  virtual void emptyCollection() {
    std::lock_guard<std::recursive_mutex> lk(mtx);
    this->batched_data_->clear();
  }

  std::shared_ptr<std::list<T>> batched_data_;
  mutable std::recursive_mutex mtx;

private:
    /**
     * Size used for the internal storage
     */
    std::atomic<size_t> max_allowable_batch_size_{};
    std::atomic<size_t> trigger_batch_size_{};
    std::atomic<std::chrono::microseconds> try_enqueue_duration_{};
};

