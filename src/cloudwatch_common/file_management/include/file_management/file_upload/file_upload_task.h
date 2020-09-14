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

#include <file_management/file_upload/file_manager.h>

#include <dataflow_lite/dataflow/observed_queue.h>
#include <dataflow_lite/dataflow/sink.h>

#include <dataflow_lite/task/task.h>

#include <future>
#include <memory>

#pragma once

namespace Aws {
namespace FileManagement {

/**
 * The file upload task which calls the upload status callback with the data from the initial task.
 *
 * @tparam T
 */
template<typename T>
class FileUploadTask : public Aws::DataFlow::Task<T> {
 public:
  using FileUploadStatusFunc =  Aws::DataFlow::UploadStatusFunction<Aws::DataFlow::UploadStatus, FileObject<T>>;

  explicit FileUploadTask(
      FileObject<T> &&batch_data,
      FileUploadStatusFunc upload_status_function
      ) : Aws::DataFlow::Task<T>()
  {
    this->batch_data_ = batch_data;
    this->upload_status_function_ = upload_status_function;
  }

  ~FileUploadTask() override = default;

  void onComplete(const Aws::DataFlow::UploadStatus &status) override {
    if (upload_status_function_) {
      upload_status_function_(status, batch_data_);
    }
  }

  T& getBatchData() override {
    return batch_data_.batch_data;
  }

private:
  FileObject<T> batch_data_;
  FileUploadStatusFunc upload_status_function_ = nullptr;
};

/**
 * The file upload task which calls the upload status callback with the data from the initial task.
 *
 * @tparam T
 */
template<typename T>
class FileUploadTaskAsync : public Aws::DataFlow::Task<T> {
 public:
  explicit FileUploadTaskAsync(
      FileObject<T> &&batch_data) : Aws::DataFlow::Task<T>()
  {
    this->batch_data_ = batch_data;
  }

  virtual ~FileUploadTaskAsync() = default;

  void onComplete(const Aws::DataFlow::UploadStatus &status) override {
    file_upload_promise_.set_value(
        std::pair<FileObject<T>, Aws::DataFlow::UploadStatus>{batch_data_, status});
  }

  inline std::future<std::pair<FileObject<T>, Aws::DataFlow::UploadStatus>> getResult() {
    return file_upload_promise_.get_future();
  }

  T& getBatchData() override {
    return batch_data_.batch_data;
  }

private:
  FileObject<T> batch_data_;
  std::promise<std::pair<FileObject<T>, Aws::DataFlow::UploadStatus>> file_upload_promise_ = nullptr;
};

//------------- Definitions --------------//
template<typename T>
using TaskPtr = std::shared_ptr<Aws::DataFlow::Task<T>>;

template<typename T>
using FileUploadTaskPtr = std::shared_ptr<FileUploadTask<T>>;

template<typename T>
using TaskObservedQueue = Aws::DataFlow::ObservedQueue<TaskPtr<T>>;

template<typename T>
using TaskObservedBlockingQueue = Aws::DataFlow::ObservedBlockingQueue<TaskPtr<T>>;

template<typename T>
using TaskSink = Aws::DataFlow::Sink<TaskPtr<T>>;
//----------------------------------------//

}  // namespace FileManagement
}  // namespace Aws
