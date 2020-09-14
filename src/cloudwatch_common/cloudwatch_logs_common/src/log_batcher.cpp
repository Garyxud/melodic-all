/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <cloudwatch_logs_common/log_batcher.h>

#include <file_management/file_upload/file_upload_task.h>

#include <dataflow_lite/utils/data_batcher.h>
#include <cloudwatch_logs_common/definitions/definitions.h>
#include <dataflow_lite/task/task.h>

#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>


namespace Aws {
namespace CloudWatchLogs {

LogBatcher::LogBatcher(size_t max_allowable_batch_size,
                       size_t publish_trigger_size)
                       : DataBatcher(max_allowable_batch_size, publish_trigger_size) {
}

LogBatcher::~LogBatcher() = default;

bool LogBatcher::publishBatchedData() {
  static const char * const kFuncName = __func__;

  std::lock_guard<std::recursive_mutex> lk(mtx);

  // is there anything to send?
  if (getCurrentBatchSize() == 0) {
    AWS_LOGSTREAM_DEBUG(__func__, "LogBatcher: nothing batched to publish");
    return false;
  }

  std::shared_ptr<LogCollection> log_type = this->batched_data_;
  std::shared_ptr<Aws::DataFlow::BasicTask<LogCollection>> log_task = std::make_shared<Aws::DataFlow::BasicTask<LogCollection>>(log_type);

  // connect to the log_file_manager_ to write to disk on task failure
  if (log_file_manager_) {

    // register the task failure function
    auto function = [&log_file_manager = this->log_file_manager_](const DataFlow::UploadStatus &upload_status,
                                                                  const LogCollection &log_messages)
    {
        if (!log_messages.empty()) {

          if (DataFlow::UploadStatus::INVALID_DATA == upload_status) {

            // publish indicated the task data was bad, this task should be discarded
            AWS_LOG_WARN(kFuncName, "LogBatcher: Task failed due to invalid log data, dropping");

          } else if (DataFlow::UploadStatus::SUCCESS != upload_status) {

            AWS_LOG_INFO(kFuncName, "LogBatcher: Task failed to upload: writing logs to file. Status = %d", upload_status);
            log_file_manager->write(log_messages);

          } else {
            AWS_LOG_DEBUG(kFuncName, "LogBatcher: Task log upload successful");
          }
        } else {
          AWS_LOG_INFO(kFuncName, "LogBatcher: not publishing task as log_messages is empty");
        }
    };

    log_task->setOnCompleteFunction(function);
  }

  // dont attempt to queue if not started
  if(ServiceState::STARTED != this->getState()) {
    AWS_LOG_WARN(__func__, "current service state is not Started, canceling task: %s", Service::getStatusString().c_str());
    log_task->cancel();
    return false;
  }

  bool enqueue_success = false;

  if (getSink()) {

    enqueue_success = getSink()->tryEnqueue(log_task, this->getTryEnqueueDuration());

    if (!enqueue_success) {
      AWS_LOG_WARN(__func__, "Unable to enqueue log data, canceling task");
    }

  } else {
    // if we can't queue, then cancel (write to disk)
    AWS_LOGSTREAM_WARN(__func__, "Unable to obtain queue, canceling task");
  }

  if (!enqueue_success) {
    log_task->cancel();
  }
  this->resetBatchedData();
  return enqueue_success;
}

void LogBatcher::emptyCollection() {
  std::lock_guard<std::recursive_mutex> lck(mtx);

  if (this->log_file_manager_) {
    AWS_LOG_INFO(__func__, "Writing data to file");
    log_file_manager_->write(*this->batched_data_);
  } else {
    AWS_LOG_WARN(__func__, "Dropping data");
  }
  this->resetBatchedData();
}


bool LogBatcher::start() {
  if (log_file_manager_ == nullptr) {
    AWS_LOGSTREAM_WARN(__func__, "FileManager not found: data will be dropped on failure.");
  }
  return Service::start();
}

void LogBatcher::setLogFileManager(std::shared_ptr<Aws::FileManagement::FileManager<LogCollection>> log_file_manager)
{
  if (nullptr == log_file_manager) {
    throw std::invalid_argument("input FileManager cannot be null");
  }
  this->log_file_manager_ = std::move(log_file_manager);
}

}  // namespace CloudWatchLogs
}  // namespace Aws
