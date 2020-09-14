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

/**
 *  @file
 *  @brief Contains Error handling functionality for ROS AWS CloudWatch Logs libraries.
 *
 */
namespace Aws {
namespace CloudWatchLogs {
/**
 *  @enum Aws::CloudWatchLogs::ROSCloudWatchLogsErrors
 *  @brief Defines error return codes for functions
 *  This enum defines standard error codes that will be returned by AWS CloudWatch Logs libraries.
 */
enum ROSCloudWatchLogsErrors {
  /** Indicates that there is no error. */
  CW_LOGS_SUCCEEDED = 0,
  /** A generic error occured */
  CW_LOGS_FAILED,
  /** An error when a NULL value is supplied as a parameter when a non-NULL value is required */
  CW_LOGS_NULL_PARAMETER,
  /** An error indicating that data cannot be used for other operations because it's locked. */
  CW_LOGS_DATA_LOCKED,
  /** An error indicating that a thread is not initialized. */
  CW_LOGS_PUBLISHER_THREAD_NOT_INITIALIZED,
  /** An error indicating that a thread is not available. */
  CW_LOGS_THREAD_BUSY,
  /** An error indicating that a data structure is empty. */
  CW_LOGS_EMPTY_PARAMETER,
  /** An error indicating that a log stream list is empty. */
  CW_LOGS_LOG_STREAM_LIST_EMPTY,
  /** An error indicating that the attempt to create a log group failed. */
  CW_LOGS_CREATE_LOG_GROUP_FAILED,
  /** An error indicating that the attempt to create a log stream failed. */
  CW_LOGS_CREATE_LOG_STREAM_FAILED,
  /** An error indicating that a log group already exists. */
  CW_LOGS_LOG_GROUP_ALREADY_EXISTS,
  /** An error indicating that a log stream already exists. */
  CW_LOGS_LOG_STREAM_ALREADY_EXISTS,
  /** An error indicating that a log group cannot be found. */
  CW_LOGS_LOG_GROUP_NOT_FOUND,
  /** An error indicating that a log stream cannot be found in a log group. */
  CW_LOGS_LOG_STREAM_NOT_FOUND,
  /** Log stream was not configured properly. */
  CW_LOGS_LOG_STREAM_NOT_CONFIGURED,
  // todo this is a gross hack, should encapsulate NETWORK_CONNECTION and REQUEST_TIMEOUT (latter not in 1.6.53)
  CW_LOGS_NOT_CONNECTED
};

}  // namespace CloudWatchLogs
}  // namespace Aws
