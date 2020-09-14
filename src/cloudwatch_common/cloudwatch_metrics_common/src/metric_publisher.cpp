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
#include <aws/monitoring/CloudWatchClient.h>
#include <aws/monitoring/model/PutMetricDataRequest.h>

#include <aws_common/sdk_utils/aws_error.h>

#include <cloudwatch_metrics_common/metric_publisher.hpp>
#include <cloudwatch_metrics_common/definitions/definitions.h>

#include <memory>

using Aws::CloudWatchMetrics::Utils::CloudWatchMetricsFacade;
using Aws::CloudWatchMetrics::Utils::CloudWatchMetricsStatus;


namespace Aws {
namespace CloudWatchMetrics {

MetricPublisher::MetricPublisher(
  const std::string & metrics_namespace,
  const Aws::Client::ClientConfiguration & client_config)
{
  this->metrics_namespace_ = metrics_namespace;
  this->client_config_ = client_config;
}

MetricPublisher::MetricPublisher(const std::string & metrics_namespace,
                                 std::shared_ptr<Utils::CloudWatchMetricsFacade> cloudwatch_metrics_facade)
{
  this->metrics_namespace_ = metrics_namespace;
  this->cloudwatch_metrics_facade_ = std::move(cloudwatch_metrics_facade);
}

bool MetricPublisher::start() {

  if (!this->cloudwatch_metrics_facade_) {
    this->cloudwatch_metrics_facade_ = std::make_shared<CloudWatchMetricsFacade>(this->client_config_);
  }
  return Service::start();
}

bool MetricPublisher::shutdown() {
  return true;
}

Aws::DataFlow::UploadStatus MetricPublisher::publishData(MetricDatumCollection &data)
{

  if (data.empty()) {
    return Aws::DataFlow::INVALID_DATA;
  }

  CloudWatchMetricsStatus status = this->cloudwatch_metrics_facade_->SendMetricsToCloudWatch(this->metrics_namespace_, data);
  switch(status) {
    case CloudWatchMetricsStatus::SUCCESS:
      return Aws::DataFlow::UploadStatus::SUCCESS;
    case CloudWatchMetricsStatus::INVALID_DATA:
      return Aws::DataFlow::UploadStatus::INVALID_DATA;
    default:
      return Aws::DataFlow::UploadStatus::FAIL;
  }
}

}  // namespace CloudWatchMetrics
}  // namespace Aws
