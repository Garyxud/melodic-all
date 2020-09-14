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
#include <aws_common/sdk_utils/aws_error.h>

#include <cloudwatch_metrics_common/definitions/definitions.h>
#include <cloudwatch_metrics_common/utils/cloudwatch_metrics_facade.hpp>

#include <memory>
#include <thread>

#include <dataflow_lite/utils/publisher.h>

namespace Aws {
namespace CloudWatchMetrics {

/**
 *  @brief Class that handles sending metrics data to CloudWatch
 *  This class is responsible for emitting all the stored metrics to AWS CloudWatch.
 *  Metrics are published asynchronously using a thread. The thread waits on a condition
 *  variable and is signaled (by AWSCloudWatchMetricManager) whenever new metrics are
 *  available.
 */
class MetricPublisher : public Publisher<MetricDatumCollection>
{
public:

  MetricPublisher(const std::string & metrics_namespace, const Aws::Client::ClientConfiguration & client_config);
  MetricPublisher(const std::string & metrics_namespace, std::shared_ptr<Utils::CloudWatchMetricsFacade> cloudwatch_metrics_facade);

  /**
   *  @brief Tears down the MetricPublisher object
   */
  ~MetricPublisher() override = default;

  bool shutdown() override;
  /**
   * Create the cloudwatch facade
   *
   * @return
   */
  bool start() override;
  /**
   * Attempt to publish the input data.
   *
   * @param data input to publish to CloudWatch
   * @return the resulting Aws::DataFlow::UploadStatus from the publish attempt
   */
  Aws::DataFlow::UploadStatus publishData(MetricDatumCollection &data) override;

private:

  std::shared_ptr<Aws::CloudWatchMetrics::Utils::CloudWatchMetricsFacade> cloudwatch_metrics_facade_;

  Aws::Client::ClientConfiguration client_config_;
  Aws::SDKOptions aws_sdk_options_;
  std::string metrics_namespace_;
  mutable std::recursive_mutex mtx_;
};

}  // namespace CloudWatchMetrics
}  // namespace Aws
