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

#pragma once

#include <aws/core/Aws.h>

#include <file_management/file_upload/file_upload_streamer.h>
#include <file_management/file_upload/file_manager.h>

#include <dataflow_lite/utils/data_batcher.h>
#include <dataflow_lite/utils/publisher.h>
#include <dataflow_lite/utils/service.h>

#include <chrono>
#include <list>
#include <stdexcept>
#include <string>

namespace Aws {
namespace CloudWatch {

using namespace Aws::FileManagement;

static const std::chrono::milliseconds kDequeueDuration = std::chrono::milliseconds(100);

/**
 * Generic class to batch and publish data to CloudWatch. Note: if the file_upload_streamer is set then data will be
 * stored on disk if offline (data cannot be successfully sent) and in the event of send success data will be read
 * from disk and sent.
 *
 * @tparam D the type to be batched and converted to the CloudWatch published type T
 * @tparam T the type to be published to CloudWatch, specific to the AWS SDk
 */
template<typename D, typename T>
class CloudWatchService : public Aws::DataFlow::InputStage<TaskPtr<std::list<T>>>, public RunnableService {
public:
/**
 * @throws invalid argument if either publisher or batcher are null
 * @param publisher
 * @param batcher
 * @return
 */
CloudWatchService(std::shared_ptr<Publisher<std::list<T>>> publisher,
                  std::shared_ptr<DataBatcher<T>> batcher) : RunnableService() {

  if (nullptr == publisher) {
    throw std::invalid_argument("Invalid argument: publisher cannot be null");
  }

  if (nullptr == batcher) {
    throw std::invalid_argument("Invalid argument: batcher cannot be null");
  }

  this->publisher_ = publisher;
  this->batcher_ = batcher;
  this->file_upload_streamer_ = nullptr;
  this->dequeue_duration_ = kDequeueDuration;
  this->number_dequeued_.store(0);
}

~CloudWatchService() = default;

/**
 * Start the publisher, batcher, and file upload streamer (collection of services).
 *
 * @return true if everything started correctly
 */
virtual bool start() override {
  bool started = true;

  started &= publisher_->start();
  started &= batcher_->start();

  if (file_upload_streamer_) {
    started &= file_upload_streamer_->start();
  }

  //start the thread to dequeue
  started &= RunnableService::start();

  return started;
}

/**
 * Shut all services down. Note: this method blocks and waits for the FileUploadStreamer's and its own RunnableService
 * work threads to be complete.
 *
 * @return true if everything shutdown correctly
 */
virtual inline bool shutdown() {

  //  stop the work thread immediately, don't hand any more tasks to the publisher
  bool is_shutdown = RunnableService::shutdown();

  is_shutdown &= publisher_->shutdown();
  is_shutdown &= batcher_->shutdown();

  if (file_upload_streamer_) {
    is_shutdown &= file_upload_streamer_->shutdown();
    // wait for file_upload_streamer_ (RunnableService) shutdown to complete
    file_upload_streamer_->join();
  }

  // wait for RunnableService shutdown to complete
  this->join();

  return is_shutdown;
}

/**
 * Entry point to batch data for publishing
 *
 * @param data_to_batch
 * @return true of the data was successfully batched, false otherwise
 */
virtual inline bool batchData(const D &data_to_batch) {

  T t = convertInputToBatched(data_to_batch);
  return batcher_->batchData(t);
}

/**
 * Entry point to batch data for publishing
 *
 * @param data_to_batch
 * @param milliseconds timestamp of the data
 * @return true of the data was successfully batched, false otherwise
 */
virtual inline bool batchData(const D &data_to_batch, const std::chrono::milliseconds &milliseconds) {

  // convert
  T t = convertInputToBatched(data_to_batch, milliseconds);
  return batcher_->batchData(t);
}

/**
 * Publishing mechanism, force the batcher to yield its data to the publisher and attempt to send.
 *
 * @param data_to_batch
 *
 */
virtual inline bool publishBatchedData() {
  if (batcher_) {
    return batcher_->publishBatchedData();
  }
  return false;
}

/**
 * Return the amount of time to wait to dequeue a work item to attempt to publish.
 *
 * @return std::chrono::milliseconds the amount of time to wait for tryDequeue
 */
virtual inline std::chrono::milliseconds getDequeueDuration() {
  return this->dequeue_duration_;
}

virtual inline bool setDequeueDuration(std::chrono::milliseconds new_value) {
  bool is_set = false;
  if (new_value.count() >= 0) {
    this->dequeue_duration_ = new_value;
    is_set = true;
  }
  return is_set;
}

/**
 * Get the total number of items dequeued (and sent to the publishing mechanism) from the work thread.
 *
 * @return the number of items dequeued and attempted to publish
 */
int getNumberDequeued() {
  return this->number_dequeued_.load();
}

/**
 * Return the current connected state.
 *
 * @return true if a connection to CloudWatch has been made, false if offline.
 */
virtual bool isConnected() {
  return this->publisher_->getPublisherState() == PublisherState::CONNECTED;
}


/**
 * Return the current timestamp
 *
 * @return current timestamp in std::chrono::milliseconds
 */
virtual std::chrono::milliseconds getCurrentTimestamp() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
}

protected:

virtual T convertInputToBatched(const D &input, const std::chrono::milliseconds &milliseconds) = 0;
virtual T convertInputToBatched(const D &input) = 0;


/**
 * Main workhorse thread that dequeues from the source and calls Task run.
 */
void work() {

  TaskPtr<std::list<T>> task_to_publish;
  bool is_dequeued = Aws::DataFlow::InputStage<TaskPtr<std::list<T>>>::getSource()
          ->dequeue(task_to_publish, dequeue_duration_);

  if (is_dequeued) {

    if (task_to_publish) {
      this->number_dequeued_++;
      AWS_LOGSTREAM_DEBUG(__func__, "Number of tasks dequeued = " << this->number_dequeued_++);

      if (Service::getState() == ServiceState::STARTED) {
        // publish mechanism via the TaskFactory
        task_to_publish->run(publisher_);
      } else {
        // unable to publish, fast fail and cancel the current task
        task_to_publish->cancel();
      }
    }
  }
}

std::shared_ptr<FileUploadStreamer<std::list<T>>> file_upload_streamer_;
std::shared_ptr<Publisher<std::list<T>>> publisher_;
std::shared_ptr<DataBatcher<T>> batcher_;

/**
 * Duration to wait for work from the queue.
 */
std::chrono::milliseconds dequeue_duration_;

private:
/**
 * Count of total items dequeued.
 */
std::atomic<int> number_dequeued_;
};

}  // namespace Cloudwatch
}  // namespace AWS

