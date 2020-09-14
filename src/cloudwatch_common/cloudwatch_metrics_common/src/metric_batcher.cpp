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

#include <cloudwatch_metrics_common/metric_batcher.h>

#include <file_management/file_upload/file_upload_task.h>

#include <dataflow_lite/utils/data_batcher.h>

#include <dataflow_lite/task/task.h>

#include <chrono>
#include <iostream>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <stdexcept>


namespace Aws {
namespace CloudWatchMetrics {

MetricBatcher::MetricBatcher(size_t max_allowable_batch_size,
                             size_t publish_trigger_size)
        : DataBatcher(max_allowable_batch_size, publish_trigger_size) {
}

MetricBatcher::~MetricBatcher() = default;

bool MetricBatcher::publishBatchedData() {
  static const char * const kFuncName = __func__;

  std::lock_guard<std::recursive_mutex> lk(mtx);

  // is there anything to send?
  if (getCurrentBatchSize() == 0) {
    AWS_LOGSTREAM_DEBUG(__func__, "Nothing batched to publish");
    return false;
  }

  auto metrics_to_publish = this->batched_data_;
  std::shared_ptr<Aws::DataFlow::BasicTask<MetricDatumCollection>> metric_task = std::make_shared<Aws::DataFlow::BasicTask<MetricDatumCollection>>(metrics_to_publish);

  // connect to the file manager to write to disk on fail / cancel
  if (metric_file_manager_ ) {

    // register the task failure function
    auto function = [&metric_file_manager = this->metric_file_manager_](const DataFlow::UploadStatus &upload_status,
                                                                        const MetricDatumCollection &metrics_to_publish)
    {
        if (!metrics_to_publish.empty()) {

          if (DataFlow::UploadStatus::INVALID_DATA == upload_status) {

            // publish indicated the task data was bad, this task should be discarded
            AWS_LOG_WARN(kFuncName, "MetricBatcher: Task failed due to invalid metric data, dropping");

          } else if (DataFlow::UploadStatus::SUCCESS != upload_status) {

            AWS_LOG_INFO(kFuncName, "MetricBatcher: Task failed: writing metrics to file");
            metric_file_manager->write(metrics_to_publish);

          } else {
            AWS_LOG_DEBUG(kFuncName, "MetricBatcher: Task metric upload successful");
          }
        } else {
          AWS_LOG_INFO(kFuncName, "MetricBatcher: not publishing task as metrics_to_publish is empty");
        }
    };

    metric_task->setOnCompleteFunction(function);
  }

  // dont attempt to queue if not started
  if(ServiceState::STARTED != this->getState()) {
    AWS_LOG_WARN(__func__, "current service state is not Started, canceling task: %s", Service::getStatusString().c_str());
    metric_task->cancel();
    return false;
  }

  bool enqueue_success = false;

  // try to enqueue
  if (getSink()) {

    enqueue_success = getSink()->tryEnqueue(metric_task, this->getTryEnqueueDuration());

    if (!enqueue_success) {
      AWS_LOG_WARN(__func__, "Unable to enqueue data, canceling task");
    }

  } else {
    AWS_LOGSTREAM_WARN(__func__, "Unable to obtain queue, canceling task");
  }

  if (!enqueue_success) {
    metric_task->cancel();
  }
  this->resetBatchedData();
  return enqueue_success;
}

void MetricBatcher::emptyCollection() {
  std::lock_guard<std::recursive_mutex> lk(mtx);

  if (this->metric_file_manager_) {
    AWS_LOG_INFO(__func__, "Writing data to file");
    metric_file_manager_->write(*this->batched_data_);
  } else {
    AWS_LOG_WARN(__func__, "Dropping data");
  }
  this->resetBatchedData();
}


bool MetricBatcher::start() {
  if (metric_file_manager_ == nullptr) {
    AWS_LOGSTREAM_WARN(__func__, "FileManager not found: data will be dropped on failure.");
  }
  return Service::start();
}

void MetricBatcher::setMetricFileManager(std::shared_ptr<Aws::FileManagement::FileManager<MetricDatumCollection>> metric_file_manager)
{
  if (nullptr == metric_file_manager) {
    throw std::invalid_argument("input FileManager cannot be null");
  }
  this->metric_file_manager_ = std::move(metric_file_manager);
}

}  // namespace CloudWatchMetrics
}  // namespace Aws
