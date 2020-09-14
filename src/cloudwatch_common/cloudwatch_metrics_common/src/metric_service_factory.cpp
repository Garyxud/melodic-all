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


#include <cloudwatch_metrics_common/metric_batcher.h>
#include <cloudwatch_metrics_common/metric_publisher.hpp>
#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_service_factory.hpp>
#include <cloudwatch_metrics_common/utils/metric_file_manager.hpp>

#include <file_management/file_upload/file_management_factory.h>
#include <file_management/file_upload/file_upload_task.h>

#include <dataflow_lite/dataflow/dataflow.h>
#include <dataflow_lite/dataflow/dataflow.h>
#include <cloudwatch_metrics_common/cloudwatch_options.h>

#include <cloudwatch_metrics_common/definitions/definitions.h>

using Aws::CloudWatchMetrics::Utils::MetricFileManager;
using Aws::CloudWatchMetrics::MetricPublisher;
using Aws::FileManagement::TaskObservedBlockingQueue;


namespace Aws {
namespace CloudWatchMetrics {

// NOLINTNEXTLINE(google-default-arguments)
std::shared_ptr<MetricService> MetricServiceFactory::createMetricService(
        const std::string & metrics_namespace,
        const Aws::Client::ClientConfiguration & client_config,
        const Aws::SDKOptions & sdk_options,
        const CloudWatchOptions & cloudwatch_options)
{
  Aws::InitAPI(sdk_options); // per the SDK team this only ever needs to be called once

  auto metric_file_manager = std::make_shared<MetricFileManager>(cloudwatch_options.file_manager_strategy_options);

  auto metric_publisher = std::make_shared<MetricPublisher>(metrics_namespace, client_config);

  auto queue_monitor =
          std::make_shared<Aws::DataFlow::QueueMonitor<Aws::FileManagement::TaskPtr<MetricDatumCollection>>>();

  Aws::FileManagement::FileUploadStreamerOptions file_upload_options{
    cloudwatch_options.uploader_options.file_upload_batch_size,
    cloudwatch_options.uploader_options.file_max_queue_size
  };

  auto metric_file_upload_streamer =
          Aws::FileManagement::createFileUploadStreamer<MetricDatumCollection>(metric_file_manager, file_upload_options);

  // connect publisher state changes to the File Streamer
  metric_publisher->addPublisherStateListener([upload_streamer = metric_file_upload_streamer](const PublisherState& state) {
      auto status =
              (state == PublisherState::CONNECTED) ? Aws::DataFlow::Status::AVAILABLE : Aws::DataFlow::Status::UNAVAILABLE;
      upload_streamer->onPublisherStateChange(status);
  });

  // Create an observed queue to trigger a publish when data is available
  auto file_data_queue =
          std::make_shared<TaskObservedBlockingQueue<MetricDatumCollection>>(cloudwatch_options.uploader_options.file_max_queue_size);

  auto stream_data_queue = std::make_shared<TaskObservedBlockingQueue<MetricDatumCollection>>(cloudwatch_options.uploader_options.stream_max_queue_size);

  metric_file_upload_streamer->setSink(file_data_queue);
  queue_monitor->addSource(file_data_queue, DataFlow::PriorityOptions{Aws::DataFlow::LOWEST_PRIORITY});

  auto metric_batcher = std::make_shared<MetricBatcher>(
          cloudwatch_options.uploader_options.batch_max_queue_size,
          cloudwatch_options.uploader_options.batch_trigger_publish_size
  );
  metric_batcher->setMetricFileManager(metric_file_manager);

  metric_batcher->setSink(stream_data_queue);
  queue_monitor->addSource(stream_data_queue, Aws::DataFlow::PriorityOptions{Aws::DataFlow::HIGHEST_PRIORITY});

  auto metric_service = std::make_shared<MetricService>(metric_publisher, metric_batcher, metric_file_upload_streamer);
  metric_service->setSource(queue_monitor);

  return metric_service;
}

}  // namespace CloudWatchMetrics
}  // namespace Aws
