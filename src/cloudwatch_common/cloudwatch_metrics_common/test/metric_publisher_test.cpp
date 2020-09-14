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
#include <aws/monitoring/model/MetricDatum.h>

#include <gtest/gtest.h>

#include <cloudwatch_metrics_common/metric_publisher.hpp>
#include <cloudwatch_metrics_common/utils/cloudwatch_metrics_facade.hpp>

#include <cloudwatch_metrics_common/definitions/definitions.h>

using namespace Aws::CloudWatchMetrics;

constexpr int WAIT_TIME =
  2000;  // the amount of time (ms) to wait for publisher thread to do its work

class MockCloudWatchFacade : public Utils::CloudWatchMetricsFacade
{
public:
  Utils::CloudWatchMetricsStatus send_metrics_ret_val;
  uint32_t send_metrics_call_count{};
  std::string last_metric_namespace;
  MetricDatumCollection last_metrics;

  MockCloudWatchFacade() { reset(); }

  void reset()
  {
    this->last_metric_namespace = "";
    this->send_metrics_ret_val = Utils::CloudWatchMetricsStatus::SUCCESS;
    this->send_metrics_call_count = 0;
  }

  Utils::CloudWatchMetricsStatus SendMetricsToCloudWatch(
    const std::string & metric_namespace,
    MetricDatumCollection & metrics) override
  {
    this->last_metric_namespace = metric_namespace;
    this->last_metrics = metrics;
    this->send_metrics_call_count++;
    return this->send_metrics_ret_val;
  }
};

/* This test requires a fixture to init and shutdown the SDK or else it causes seg faults when
 * trying to construct a CloudWatch client. All tests must use the TEST_F function with this fixutre
 * name.
 */
class TestMetricPublisherFixture : public ::testing::Test
{
protected:

  MetricDatumCollection metrics_list_;
  Aws::SDKOptions options_;
  std::shared_ptr<MockCloudWatchFacade> mock_cw_;
  std::shared_ptr<MetricPublisher> publisher_;

  void SetUp() override
  {
    mock_cw_ = std::make_shared<MockCloudWatchFacade>();
    std::shared_ptr<Utils::CloudWatchMetricsFacade> cw = mock_cw_;
    publisher_ = std::make_shared<MetricPublisher>("test_namespace", cw);
    metrics_list_.emplace_back();
    metrics_list_.emplace_back();
    EXPECT_FALSE(metrics_list_.empty());
    EXPECT_TRUE(publisher_->start());
  }

  void TearDown() override
  {
    EXPECT_TRUE(publisher_->shutdown());
  }
};

TEST_F(TestMetricPublisherFixture, Sanity) {
  ASSERT_TRUE(true);
}

TEST_F(TestMetricPublisherFixture, TestLogPublisher_PublishLogs_ReturnsFalseWhenEmpty)
{
  //empty list
  MetricDatumCollection test_list;
  EXPECT_EQ(Aws::DataFlow::UploadStatus::INVALID_DATA, publisher_->attemptPublish(test_list));
  EXPECT_EQ(PublisherState::NOT_CONNECTED, publisher_->getPublisherState());
  EXPECT_EQ(0u, mock_cw_->send_metrics_call_count);
}

TEST_F(TestMetricPublisherFixture, TestMetricPublisher_PublishMetrics_ReturnsSuccessWhenListIngested)
{
  EXPECT_EQ(Aws::DataFlow::UploadStatus::SUCCESS, publisher_->attemptPublish(metrics_list_));
  EXPECT_EQ(PublisherState::CONNECTED, publisher_->getPublisherState());
  EXPECT_EQ(1u, mock_cw_->send_metrics_call_count);
}

TEST_F(TestMetricPublisherFixture, TestMetricPublisher_MultiplePublishes)
{

  EXPECT_EQ(Aws::DataFlow::UploadStatus::SUCCESS, publisher_->attemptPublish(metrics_list_));
  EXPECT_EQ(PublisherState::CONNECTED, publisher_->getPublisherState());
  EXPECT_EQ(1u, mock_cw_->send_metrics_call_count);

  EXPECT_EQ(Aws::DataFlow::UploadStatus::SUCCESS, publisher_->attemptPublish(metrics_list_));
  EXPECT_EQ(PublisherState::CONNECTED, publisher_->getPublisherState());
  EXPECT_EQ(2u, mock_cw_->send_metrics_call_count);
}
