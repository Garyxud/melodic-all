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

#include <dataflow_lite/dataflow/observed_queue.h>
#include <dataflow_lite/dataflow/sink.h>

#include <future>
#include <memory>

#pragma once

namespace Aws {
namespace DataFlow {

/**
 * Upload status for some data that was attempted to be published
 */
enum UploadStatus {
  UNKNOWN,  // safe default
  FAIL,  // the upload failed
  SUCCESS,  // the upload succeeded
  INVALID_DATA  // the upload was attempted and failed because the input data is bad and will never succeed
};

/**
 * Generic publisher interface to attempt to upload generic data.
 *
 * @tparam T
 * @return the UploadStatus resulting from the publish attempt
 */
template<typename T>
class IPublisher {
public:
    virtual UploadStatus attemptPublish(T &batch_data) = 0;
};

/**
 * Function callback when a message has attempted an upload to the cloud.
 */
template <typename Status, typename T>
using UploadStatusFunction = std::function<void (const Status& upload_status, const T &message)>;

/**
* Define a task (runnable) to get batch data and call a callback when finished with this task.
* @tparam T
*/
template <typename T>
class Task {
public:
    virtual ~Task() = default;

    /**
     * Run this task with the input publisher. Run calls IPublisher::attemptPublish and then calls onComplete
     * with the resulting IPublisher UploadStatus status.
     *
     * @param publisher mechanism to publish
     */
    virtual void run(std::shared_ptr<IPublisher<T>> publisher) {
      auto status = publisher->attemptPublish(getBatchData());
      this->onComplete(status);
    }

    /**
     * This task is no longer valid. Call onComplete with a FAIL status.
     */
    virtual void cancel() {
      this->onComplete(FAIL);
    }

    /**
     * Override this method to handle completion status.
     *
     * @param status
     */
    virtual void onComplete(const UploadStatus &status) = 0;

    /**
     * Get this task's data
     *
     * @return
     */
    virtual T& getBatchData() = 0;
};

template<typename T>
class BasicTask :
        public Task<T> {
public:
    explicit BasicTask(
            std::shared_ptr<T> batch_data) : Task<T>()
    {
      this->batch_data_ = batch_data;
      // null is allowable as there is a guard above (default action do nothing)
      this->upload_status_function_ = nullptr;
    }

    virtual ~BasicTask() = default;

    void onComplete(const UploadStatus &status) override {
      if (upload_status_function_) {
        upload_status_function_(status, *batch_data_);
      }
    }

    void setOnCompleteFunction(
            const UploadStatusFunction<UploadStatus, T> upload_status_function)
    {
      // null is allowable as there is a guard above (default action do nothing)
      upload_status_function_ = upload_status_function;
    }

    T& getBatchData() override {
      return *batch_data_;
    }

private:
    std::shared_ptr<T> batch_data_;
    UploadStatusFunction<UploadStatus, T> upload_status_function_;
};

}  // namespace DataFlow
}  // namespace Aws