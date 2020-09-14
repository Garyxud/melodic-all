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

#include <cloudwatch_logs_common/log_batcher.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <cloudwatch_logs_common/log_service.h>

#include <cloudwatch_logs_common/cloudwatch_options.h>

namespace Aws {
namespace CloudWatchLogs {

// todo should we validate options here?

class LogServiceFactory
{
public:
  LogServiceFactory() = default;

  /**
   * Block copy constructor and assignment operator for Factory object.
   */
  LogServiceFactory(const LogServiceFactory &) = delete;
  LogServiceFactory & operator=(const LogServiceFactory &) = delete;

  ~LogServiceFactory() = default;

  /**
   *  @brief Creates a new LogService object
   *  Factory method used to create a new LogService object, along with a creating and starting a
   * LogPublisher for use with the LogService.
   *
   *  @param client_config The client configuration to use when creating the CloudWatch clients
   *  @param options The options used for the AWS SDK when creating and publishing with a CloudWatch
   * client
   *
   *  @return An instance of LogService
   */
  // NOLINTNEXTLINE(google-default-arguments)
  virtual std::shared_ptr<LogService> CreateLogService(
    const std::string & log_group,
    const std::string & log_stream,
    const Aws::Client::ClientConfiguration & client_config,
    const Aws::SDKOptions & sdk_options,
    const CloudWatchOptions & cloudwatch_options = kDefaultCloudWatchOptions);
};

}  // namespace CloudWatchLogs
}  // namespace Aws
