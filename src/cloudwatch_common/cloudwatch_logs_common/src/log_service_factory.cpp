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

#include <list>
#include <string>

#include <cloudwatch_logs_common/cloudwatch_options.h>
#include <cloudwatch_logs_common/log_batcher.h>
#include <cloudwatch_logs_common/log_service_factory.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <cloudwatch_logs_common/log_service.h>

#include <cloudwatch_logs_common/utils/log_file_manager.h>

#include <file_management/file_upload/file_management_factory.h>
#include <file_management/file_upload/file_upload_task.h>

#include <dataflow_lite/dataflow/dataflow.h>

#include <cloudwatch_logs_common/definitions/definitions.h>

using Aws::CloudWatchLogs::Utils::LogFileManager;
using Aws::CloudWatchLogs::LogPublisher;
using Aws::FileManagement::TaskObservedBlockingQueue;

namespace Aws {
namespace CloudWatchLogs {

// NOLINTNEXTLINE(google-default-arguments)
std::shared_ptr<LogService> LogServiceFactory::CreateLogService(
  const std::string & log_group,
  const std::string & log_stream,
  const Aws::Client::ClientConfiguration & client_config,
  const Aws::SDKOptions & sdk_options,
  const CloudWatchOptions & cloudwatch_options)
{
  Aws::InitAPI(sdk_options); // per the SDK team this only ever needs to be called once

  auto log_file_manager = std::make_shared<LogFileManager>(cloudwatch_options.file_manager_strategy_options);

  auto publisher = std::make_shared<LogPublisher>(log_group, log_stream, client_config);

  auto queue_monitor =
      std::make_shared<Aws::DataFlow::QueueMonitor<Aws::FileManagement::TaskPtr<LogCollection>>>();

  Aws::FileManagement::FileUploadStreamerOptions file_upload_options{
    cloudwatch_options.uploader_options.file_upload_batch_size,
    cloudwatch_options.uploader_options.file_max_queue_size
  };

  auto log_file_upload_streamer =
      Aws::FileManagement::createFileUploadStreamer<LogCollection>(log_file_manager, file_upload_options);

  // connect publisher state changes to the File Streamer
  publisher->addPublisherStateListener([upload_streamer = log_file_upload_streamer](const PublisherState& state) {
    auto status =
      (state == PublisherState::CONNECTED) ? Aws::DataFlow::Status::AVAILABLE : Aws::DataFlow::Status::UNAVAILABLE;
    upload_streamer->onPublisherStateChange(status);
  });

  // Create an observed queue to trigger a publish when data is available
  auto file_data_queue =
      std::make_shared<TaskObservedBlockingQueue<LogCollection>>(cloudwatch_options.uploader_options.file_max_queue_size);

  auto stream_data_queue = std::make_shared<TaskObservedBlockingQueue<LogCollection>>(cloudwatch_options.uploader_options.stream_max_queue_size);

  auto log_batcher = std::make_shared<LogBatcher>(
    cloudwatch_options.uploader_options.batch_max_queue_size,
    cloudwatch_options.uploader_options.batch_trigger_publish_size
  );
  log_batcher->setLogFileManager(log_file_manager);

  log_file_upload_streamer->setSink(file_data_queue);
  queue_monitor->addSource(file_data_queue, DataFlow::PriorityOptions{Aws::DataFlow::LOWEST_PRIORITY});

  log_batcher->setSink(stream_data_queue);
  queue_monitor->addSource(stream_data_queue, DataFlow::PriorityOptions{Aws::DataFlow::HIGHEST_PRIORITY});

  auto log_service = std::make_shared<LogService>(publisher, log_batcher, log_file_upload_streamer);
  log_service->setSource(queue_monitor);

  return log_service;  // allow user to start
}

}  // namespace CloudWatchLogs
}  // namespace Aws
