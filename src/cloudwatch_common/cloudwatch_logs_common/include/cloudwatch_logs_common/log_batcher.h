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
#include <aws/logs/model/InputLogEvent.h>
#include <cloudwatch_logs_common/definitions/ros_cloudwatch_logs_errors.h>

#include <file_management/file_upload/file_upload_streamer.h>
#include <file_management/file_upload/file_manager.h>

#include <dataflow_lite/utils/data_batcher.h>

#include <cloudwatch_logs_common/definitions/definitions.h>

#include <chrono>
#include <list>
#include <memory>

namespace Aws {
namespace CloudWatchLogs {

class LogBatcher :
  public Aws::DataFlow::OutputStage<Aws::FileManagement::TaskPtr<LogCollection>>,
  public DataBatcher<LogType>
{
public:


  /**
   *  @brief Creates a new LogBatcher
   *  Creates a new LogBatcher that will group/buffer logs. Note: logs are only automatically published if the
   *  size is set, otherwise the publishBatchedData is necesary to push data to be published.
   *
   *  @throws invalid argument if publish_trigger_size is strictly greater than max_allowable_batch_size
   *  @param size of the batched data that will trigger a publish
   */
  explicit LogBatcher(size_t max_allowable_batch_size = DataBatcher::kDefaultMaxBatchSize,
                      size_t publish_trigger_size = DataBatcher::kDefaultTriggerSize);

  LogBatcher(const LogBatcher & other) = delete;

  LogBatcher & operator=(const LogBatcher & other) = delete;

  /**
   *  @brief Tears down a LogBatcher object
   */
  ~LogBatcher() override;

  /**
   *  @brief Services the log manager by performing periodic tasks when called.
   *  Calling the Service function allows for periodic tasks associated with the log manager, such
   * as flushing buffered logs, to be performed.
   *
   *  @return true of the data was succesfully published, false otherwise
   */
  bool publishBatchedData() override;

  /**
   * Override default behavior to attempt to write to file to disk when emptying the collection.
   */
  void emptyCollection() override;
  bool start() override;

  /**
   * Set the log file manager, used for task publishing failures (write to disk if unable to send to CloudWatch).
   *
   * @throws invalid argument if the input is null
   * @param log_file_manager
   */
  virtual void setLogFileManager(std::shared_ptr<Aws::FileManagement::FileManager<LogCollection>> log_file_manager);

private:
  std::shared_ptr<Aws::FileManagement::FileManager<LogCollection>> log_file_manager_;
};

}  // namespace CloudWatchLogs
}  // namespace Aws
