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

#include <iostream>
#include <fstream>
#include <cstdio>
#include <experimental/filesystem>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <aws/logs/model/InputLogEvent.h>
#include <aws/core/utils/memory/stl/AWSString.h>
#include <aws/core/utils/logging/ConsoleLogSystem.h>
#include <aws/core/utils/logging/AWSLogging.h>

#include <file_management/file_upload/file_manager.h>
#include <file_management/file_upload/file_manager_strategy.h>
#include <file_management/utils/log_file_manager.h>


using namespace Aws::CloudWatchLogs;
using namespace Aws::FileManagement;

class FileManagerTest : public ::testing::Test {
public:
  void SetUp() override
  {
  }

  void TearDown() override
  {
    std::experimental::filesystem::path storage_path(options.storage_directory);
    std::experimental::filesystem::remove_all(storage_path);
  }

protected:
  FileManagerStrategyOptions options{"test", "log_tests/", ".log", 1024*1024, 1024*1024};
};

/**
 * Test that the upload complete with CW Failure goes to a file.
 */
TEST_F(FileManagerTest, file_manager_write) {
  std::shared_ptr<FileManagerStrategy> file_manager_strategy = std::make_shared<FileManagerStrategy>(options);
  LogFileManager file_manager(file_manager_strategy);
  LogEventCollection log_data;
  Aws::CloudWatchLogs::Model::InputLogEvent input_event;
  input_event.SetTimestamp(0);
  input_event.SetMessage("Hello my name is foo");
  log_data.push_back(input_event);
  file_manager.write(log_data);
  std::string line;
  file_manager_strategy->read(line);
  EXPECT_EQ(line, "{\"timestamp\":0,\"message\":\"Hello my name is foo\"}");
}

int main(int argc, char** argv)
{
  Aws::Utils::Logging::InitializeAWSLogging(
      Aws::MakeShared<Aws::Utils::Logging::ConsoleLogSystem>(
          "RunUnitTests", Aws::Utils::Logging::LogLevel::Trace));
  ::testing::InitGoogleMock(&argc, argv);
  int exitCode = RUN_ALL_TESTS();
  Aws::Utils::Logging::ShutdownAWSLogging();
  return exitCode;
}

