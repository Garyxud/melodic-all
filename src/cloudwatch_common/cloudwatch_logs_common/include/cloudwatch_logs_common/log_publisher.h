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
#include <aws/logs/CloudWatchLogsClient.h>
#include <cloudwatch_logs_common/utils/cloudwatch_logs_facade.h>

#include <dataflow_lite/utils/publisher.h>
#include <dataflow_lite/dataflow/source.h>

#include <file_management/file_upload/file_upload_task.h>
#include <dataflow_lite/task/task.h>

#include <cloudwatch_logs_common/definitions/definitions.h>

#include <list>
#include <memory>
#include <thread>

namespace Aws {
namespace CloudWatchLogs {

/** 
 * @enum Aws::CloudWatchLogs::LogPublisherRunState
 * @brief Defines the different runtime states for the Publisher
 * This enum is used by the LogPublisher to track the current runtime state of the Run function
 */
enum LogPublisherRunState {
  LOG_PUBLISHER_RUN_CREATE_GROUP,
  LOG_PUBLISHER_RUN_CREATE_STREAM,
  LOG_PUBLISHER_RUN_INIT_TOKEN,
  LOG_PUBLISHER_ATTEMPT_SEND_LOGS,
};

const static Aws::String UNINITIALIZED_TOKEN = "_NOT_SET_";

/**
 * Wrapping class around the CloudWatch Logs API.
 */
class LogPublisher : public Publisher<LogCollection>
{
public:
  /**
   *  @brief Creates a LogPublisher object that uses the provided client and SDK configuration
   *  Constructs a LogPublisher object that will use the provided CloudWatchClient and SDKOptions
   * when it publishes logs.
   *
   *  @param client_config The ClientConfiguration that this publisher will use to create the Client.
   */
  LogPublisher(const std::string & log_group, const std::string & log_stream,
               const Aws::Client::ClientConfiguration & client_config);

  /**
   *  @brief Creates a LogPublisher object that uses the provided client and SDK configuration
   *  Constructs a LogPublisher object that will use the provided CloudWatchClient and SDKOptions
   * when it publishes logs.
   *
   *  @param cw_client The CloudWatchClient that this publisher will use when pushing logs to
   * CloudWatch.
   */
  LogPublisher(const std::string & log_group, const std::string & log_stream,
               std::shared_ptr<Aws::CloudWatchLogs::Utils::CloudWatchLogsFacade> cw_client);

  LogPublisher(const LogPublisher & other) = delete;

  LogPublisher & operator=(const LogPublisher & other) = delete;

  /**
   *  @brief Tears down the LogPublisher object
   */
  ~LogPublisher() override;

  bool shutdown() override;

  /**
   * Create the cloudwatch facade
   * @return
   */
  bool start() override;

  LogPublisherRunState getRunState();

private:

  /**
   * Check if the input status is related to a network failure.
   *
   * @param error
   * @return true if connected to the internet, false otherwise
   */
  bool checkIfConnected(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors error);

  /**
   * Reset the current init token to UNINITIALIZED_TOKEN
   */
  void resetInitToken();

  //overall config
  bool configure();

  //main publish mechanism
  Aws::DataFlow::UploadStatus publishData(std::list<Aws::CloudWatchLogs::Model::InputLogEvent> & data) override;

  bool CreateGroup();
  bool CreateStream();
  bool InitToken(Aws::String & next_token);

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors SendLogs(Aws::String & next_token, std::list<Aws::CloudWatchLogs::Model::InputLogEvent> & data);

  std::shared_ptr<Aws::CloudWatchLogs::Utils::CloudWatchLogsFacade> cloudwatch_facade_;
  Aws::SDKOptions aws_sdk_options_;
  std::string log_group_;
  std::string log_stream_;
  Aws::SDKOptions options_;
  Aws::Client::ClientConfiguration client_config_;

  ObservableObject<LogPublisherRunState> run_state_;
  Aws::String next_token;
  mutable std::recursive_mutex mtx_;
};

}  // namespace CloudWatchLogs
}  // namespace Aws
