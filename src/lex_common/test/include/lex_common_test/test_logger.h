/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#ifndef LEX_COMMON_TEST__TEST_LOGGER_H_
#define LEX_COMMON_TEST__TEST_LOGGER_H_

#include <aws_common/sdk_utils/logging/aws_log_system.h>

#include <aws/core/utils/logging/LogSystemInterface.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogLevel.h>

#include <iostream>
#include <cstdio>
#include <string>

namespace Aws
{
namespace Utils
{
namespace Logging
{
/**
 * Test log system.
 */
class AWS_CORE_API TestLogSystem : public AWSLogSystem
{
public:
  /**
   * @param log_level Defaults to Trace. This log level is an additional layer on top of ROS' log
   * filtering. Typically, you would instantiate this with the lowest (most permissive) log level
   * (i.e. Trace), and control the log level via ROS.
   */
  explicit TestLogSystem(Aws::Utils::Logging::LogLevel log_level = LogLevel::Trace)
  : AWSLogSystem(log_level)
  {}
  virtual ~TestLogSystem() = default;

protected:
  void LogData(const char * log_level, const char * tag, const std::string & message)
  {
    std::cout << "<" << log_level << ", " << tag << ">: " << message << std::endl;
  }
  void LogTrace(const char * tag, const std::string & message) override
  {
    LogData("Trace", tag, message);
  }
  void LogInfo(const char * tag, const std::string & message) override
  {
    LogData("Info", tag, message);
  }
  void LogDebug(const char * tag, const std::string & message) override
  {
    LogData("Debug", tag, message);
  }
  void LogWarn(const char * tag, const std::string & message) override
  {
    LogData("Warn", tag, message);
  }
  void LogError(const char * tag, const std::string & message) override
  {
    LogData("Error", tag, message);
  }
  void LogFatal(const char * tag, const std::string & message) override
  {
    LogData("Fatal", tag, message);
  }
};

}  // namespace Logging
}  // namespace Utils
}  // namespace Aws

#endif  // LEX_COMMON_TEST__TEST_LOGGER_H_
