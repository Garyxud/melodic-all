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
#include <aws/monitoring/model/MetricDatum.h>
#include <aws/core/utils/memory/stl/AWSString.h>
#include <aws/core/utils/logging/ConsoleLogSystem.h>
#include <aws/core/utils/logging/AWSLogging.h>

#include <file_management/file_upload/file_manager.h>
#include <file_management/file_upload/file_manager_strategy.h>
#include <cloudwatch_metrics_common/utils/metric_file_manager.hpp>


using namespace Aws::CloudWatchMetrics::Utils;
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
  FileManagerStrategyOptions options{"test", "metric_tests/", ".metrics", 1024*1024, 1024*1024};
};

/**
 * Test that the upload complete with CW Failure goes to a file.
 */
TEST_F(FileManagerTest, file_manager_write) {
  std::shared_ptr<FileManagerStrategy> file_manager_strategy = std::make_shared<FileManagerStrategy>(options);
  std::shared_ptr<StatusMonitor> status_monitor = std::make_shared<StatusMonitor>();
  MetricFileManager file_manager(file_manager_strategy);
  file_manager.setStatusMonitor(status_monitor);
  file_manager.start();
  MetricDatumCollection metric_data;
  Aws::CloudWatch::Model::MetricDatum input_event;
  input_event.AddCounts(2);
  input_event.SetMetricName("MetricName");
  metric_data.push_back(input_event);
  file_manager.write(metric_data);
  std::string line;
  auto batch = file_manager.readBatch(1);
  ASSERT_EQ(1u, batch.batch_data.size());
  auto result = *batch.batch_data.begin();
  EXPECT_EQ(input_event.GetCounts(), result.GetCounts());
  EXPECT_EQ(input_event.GetMetricName(), result.GetMetricName());
}
