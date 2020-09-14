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

#include <dataflow_lite/utils/service.h>
#include <dataflow_lite/cloudwatch/cloudwatch_service.h>
#include <dataflow_lite/dataflow/dataflow.h>

#include <cloudwatch_metrics_common/definitions/definitions.h>

#include <cloudwatch_metrics_common/metric_batcher.h>
#include <cloudwatch_metrics_common/metric_publisher.hpp>

#include <cloudwatch_metrics_common/utils/metric_object.h>

#include <chrono>
#include <stdexcept>
#include <utility>

namespace Aws {
namespace CloudWatchMetrics {

/**
 * Implementation to send metrics to CloudWatch. Note: though the batcher and publisher are required, the file streamer
 * is not. If the file streamer is not provided then metric data is dropped if any failure is observed during the
 * attempt to publish.
 */
class MetricService : public Aws::CloudWatch::CloudWatchService<Utils::MetricObject, MetricDatum>
{
public:

    /**
     * Construct a new instance of MetricService.
     *
     * @param publisher used to publish metrics to CloudWatch
     * @param batcher used to batch / queue metrics before publishing
     * @param file_upload_streamer used to save metric data and upload later in the event of network connectivity changes
     */
    MetricService(
            std::shared_ptr<Publisher<MetricDatumCollection>> publisher,
            std::shared_ptr<DataBatcher<MetricDatum>> batcher,
            std::shared_ptr<Aws::FileManagement::FileUploadStreamer<MetricDatumCollection>> file_upload_streamer = nullptr)
            : CloudWatchService(std::move(publisher), std::move(batcher)) {

      this->file_upload_streamer_ = std::move(file_upload_streamer);
    }

    /**
     * Convert an input MetricObject (provided but the user) to the specific AWS SDK type.
     *
     * @param input MetricObject to convert
     * @param milliseconds timestamp to use for metric
     * @return MetricDatum to publish
     */
    MetricDatum convertInputToBatched(
            const Utils::MetricObject &input,
            const std::chrono::milliseconds &milliseconds) override {

      return metricObjectToDatum(input, static_cast<int64_t>(milliseconds.count()));
    }

    /**
     * Convert an input MetricObject (provided but the user) to the specific AWS SDK type.
     *
     * @param input MetricObject to convert
     * @return MetricDatum to publish
     */
    MetricDatum convertInputToBatched(const Utils::MetricObject &input) override {

      return metricObjectToDatum(input, input.timestamp);
    }
};

}  // namespace CloudWatchMetrics
}  // namespace Aws

