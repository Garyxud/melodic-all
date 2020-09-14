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
#include <aws/monitoring/CloudWatchClient.h>
#include <aws/monitoring/model/MetricDatum.h>
#include <aws_common/sdk_utils/aws_error.h>
#include <cloudwatch_metrics_common/definitions/definitions.h>

namespace Aws {
namespace CloudWatchMetrics {
namespace Utils {

/**
 * Used to wrap the Aws::CloudWatch::CloudWatchErrors. The current interesting states, that are handled given
 * the AWS SDK returns, are success, failure, invalid data, and not connected to the internet / CloudWatch service.
 */
enum CloudWatchMetricsStatus {
    SUCCESS,  // the CloudWatch API call was successful
    FAILURE,  // the CloudWatch API call failed for some reason
    NETWORK_FAILURE,  // the CloudWatch API call returned a timeout / not connected error
    INVALID_DATA  // invalid data was attempted to be published
};

/**
 *  @brief This class is a simple Facade over the CloudWatch client.
 *  This class is a very small abstraction over the CloudWatch client. It allows us to change the
 * details of how we're communicating with CloudWatch without the need to expose this in the rest of
 * our code. It also provides a shim for us to be able to Mock to unit test the rest of the code.
 *
 *  This class expects Aws::InitAPI() to have already been called before an instance is constructed
 *
 */
class CloudWatchMetricsFacade
{
public:
  /**
   *  @brief Creates a new CloudWatchMetricsFacade
   *  @param client_config The configuration for the cloudwatch client
   */
  // NOLINTNEXTLINE(google-explicit-constructor, hicpp-explicit-conversions)
  CloudWatchMetricsFacade(const Aws::Client::ClientConfiguration & client_config);
  /**
   * @brief Creates a new CloudWatchMetricsFacade with an existing client
   * @param cw_client The client for interacting with cloudwatch
   */
  // NOLINTNEXTLINE(google-explicit-constructor, hicpp-explicit-conversions)
  CloudWatchMetricsFacade(const std::shared_ptr<Aws::CloudWatch::CloudWatchClient>& cw_client);

  virtual ~CloudWatchMetricsFacade() = default;

  /**
   *  @brief Sends a list of metrics to CloudWatch
   *  Used to send a list of metrics to CloudWatch
   *
   *  @param metric_namespace A reference to a string with the namespace for all the metrics being
   * posted
   *  @param metrics A reference to a list of metrics that you want sent to CloudWatch
   *  @return An error code that will be Aws::AwsError::AWS_ERR_OK if all metrics were sent
   * successfully.
   */
  virtual CloudWatchMetricsStatus SendMetricsToCloudWatch(
    const std::string & metric_namespace, MetricDatumCollection & metrics);

protected:
  CloudWatchMetricsFacade() = default;

private:

  std::shared_ptr<Aws::CloudWatch::CloudWatchClient> cw_client_;
  CloudWatchMetricsStatus SendMetricsRequest(const Aws::CloudWatch::Model::PutMetricDataRequest & request);
};

}  // namespace Utils
}  // namespace CloudWatchMetrics
}  // namespace Aws
