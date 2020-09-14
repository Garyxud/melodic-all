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

#include <cloudwatch_metrics_common/metric_service.hpp>
#include <cloudwatch_metrics_common/metric_publisher.hpp>
#include <cloudwatch_metrics_common/cloudwatch_options.h>

namespace Aws {
namespace CloudWatchMetrics {

class MetricServiceFactory
{
public:
  MetricServiceFactory() = default;

  /**
   * Block copy constructor and assignment operator for Factory object.
   */
  MetricServiceFactory(const MetricServiceFactory &) = delete;
  MetricServiceFactory & operator=(const MetricServiceFactory &) = delete;

  ~MetricServiceFactory() = default;

  /**
   * Create a MetricService object used to publish metrics to CloudWatch.
   *
   * @param metrics_namespace
   * @param client_config
   * @param sdk_options
   * @param cloudwatch_options
   * @return
   */
  // NOLINTNEXTLINE(google-default-arguments)
  virtual std::shared_ptr<MetricService> createMetricService(
          const std::string & metrics_namespace,
          const Aws::Client::ClientConfiguration & client_config,
          const Aws::SDKOptions & sdk_options,
          const CloudWatchOptions & cloudwatch_options = kDefaultCloudWatchOptions);
};

}  // namespace CloudWatchMetrics
}  // namespace Aws
