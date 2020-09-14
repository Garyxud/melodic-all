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
#include <aws/logs/model/InputLogEvent.h>
#include <cloudwatch_logs_common/log_publisher.h>
#include <cloudwatch_logs_common/definitions/ros_cloudwatch_logs_errors.h>
#include <gtest/gtest.h>

using namespace Aws::CloudWatchLogs;

#define _unused(x) ((void)(x))

constexpr int WAIT_TIME =
  2000;  // the amount of time (ms) to wait for publisher thread to do its work

class MockCloudWatchFacade : public Aws::CloudWatchLogs::Utils::CloudWatchLogsFacade
{
public:
  uint32_t send_logs_call_count{};
  std::string last_log_group;
  std::string last_log_stream;
  std::list<Aws::CloudWatchLogs::Model::InputLogEvent> last_logs;
  Aws::String next_token;
  bool fail_cw_log_group{};
  bool fail_cw_log_stream{};
  bool fail_cw_create_log_group{};
  bool fail_cw_create_log_stream{};
  bool fail_cw_init_token{};
  bool fail_cw_send_logs{};
  void Reset()
  {
    this->last_log_group = "";
    this->last_log_stream = "";
    this->send_logs_call_count = 0;
    this->fail_cw_log_group = false;
    this->fail_cw_log_stream = false;
    this->fail_cw_create_log_group = false;
    this->fail_cw_create_log_stream = false;
    this->fail_cw_init_token = false;
    this->fail_cw_send_logs = false;
  }

  MockCloudWatchFacade() {
    Reset();
  }

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors CheckLogGroupExists(
    const std::string & log_group) override
  {
    _unused(log_group);
    return fail_cw_log_group ? CW_LOGS_FAILED : Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED;
  }

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors CheckLogStreamExists(
    const std::string & log_group, const std::string & log_stream,
    Aws::CloudWatchLogs::Model::LogStream * log_stream_object) override
  {
    _unused(log_group);
    _unused(log_stream);
    _unused(log_stream_object);
    return fail_cw_log_stream ? CW_LOGS_FAILED : Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED;
  }

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors CreateLogGroup(
    const std::string & log_group) override
  {
    _unused(log_group);

    return fail_cw_create_log_group ? CW_LOGS_FAILED : Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED;
  }

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors CreateLogStream(
    const std::string & log_group, const std::string & log_stream) override
  {
    _unused(log_group);
    _unused(log_stream);
    return fail_cw_create_log_stream ? CW_LOGS_FAILED : Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED;
  }

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors GetLogStreamToken(
          const std::string & log_group,
          const std::string & log_stream,
          Aws::String & next_token) override {
    _unused(log_group);
    _unused(log_stream  );
    _unused(next_token);
    if (fail_cw_init_token) {

      return CW_LOGS_FAILED;
    } else {
      next_token = Aws::String("this token has been set");
      return Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED;
    }
  }

  Aws::CloudWatchLogs::ROSCloudWatchLogsErrors SendLogsToCloudWatch(
    Aws::String & next_token,
    const std::string & last_log_group,
    const std::string & last_log_stream,
    std::list<Aws::CloudWatchLogs::Model::InputLogEvent> & logs) override
  {
    this->last_log_group = last_log_group;
    this->last_log_stream = last_log_stream;
    this->last_logs = logs;
    this->send_logs_call_count++;
    this->next_token = next_token;
    return fail_cw_send_logs ? CW_LOGS_FAILED : Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED;
  }
};

/**
 * @brief This test requires a fixture to init and shutdown the SDK or else it causes seg faults
 * when trying to construct a CloudWatch client. All tests must use the TEST_F function with this
 * fixture name.
 */
class TestLogPublisherFixture : public ::testing::Test
{
protected:

  std::list<Aws::CloudWatchLogs::Model::InputLogEvent> logs_list_;
  Aws::SDKOptions options_;
  std::shared_ptr<MockCloudWatchFacade> mock_cw_;
  std::shared_ptr<LogPublisher> publisher_;

  void StartPublisher()
  {
    EXPECT_EQ(ServiceState::CREATED, publisher_->getState());
    EXPECT_EQ(true, publisher_->start());
    EXPECT_EQ(ServiceState::STARTED, publisher_->getState());
  }

  void SetUp() override
  {
    // the tests require non-empty logs_list_
    logs_list_.emplace_back();
    logs_list_.emplace_back();
    EXPECT_FALSE(logs_list_.empty());

    mock_cw_ = std::make_shared<MockCloudWatchFacade>();
    std::shared_ptr<Aws::CloudWatchLogs::Utils::CloudWatchLogsFacade> cw = mock_cw_;
    publisher_ = std::make_shared<LogPublisher>("test_log_group", "test_log_stream", cw);
    EXPECT_EQ(LOG_PUBLISHER_RUN_CREATE_GROUP, publisher_->getRunState());
  }

  void TearDown() override
  {
    publisher_->shutdown();
  }
};

TEST_F(TestLogPublisherFixture, Sanity) {
  ASSERT_TRUE(true);
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_PublishLogs_ReturnsFalseWhenEmpty)
{
  StartPublisher();

  //empty list
  std::list<Aws::CloudWatchLogs::Model::InputLogEvent> empty_list;
  EXPECT_EQ(Aws::DataFlow::UploadStatus::INVALID_DATA, publisher_->attemptPublish(empty_list));
  EXPECT_EQ(PublisherState::NOT_CONNECTED, publisher_->getPublisherState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_PublishLogs_ReturnsSuccessWhenListIngested)
{
  StartPublisher();

  EXPECT_EQ(Aws::DataFlow::UploadStatus::SUCCESS, publisher_->attemptPublish(logs_list_));
  EXPECT_EQ(PublisherState::CONNECTED, publisher_->getPublisherState());
  EXPECT_EQ(LOG_PUBLISHER_ATTEMPT_SEND_LOGS, publisher_->getRunState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_CallsSendLogsToCW)
{
  StartPublisher();

  EXPECT_EQ(Aws::DataFlow::UploadStatus::SUCCESS, publisher_->attemptPublish(logs_list_));
  EXPECT_EQ(PublisherState::CONNECTED, publisher_->getPublisherState());
  EXPECT_EQ(1u, mock_cw_->send_logs_call_count);
  EXPECT_EQ(LOG_PUBLISHER_ATTEMPT_SEND_LOGS, publisher_->getRunState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_FailCreateGroup)
{
  StartPublisher();

  mock_cw_->fail_cw_log_group = true;
  mock_cw_->fail_cw_create_log_group = true;
  EXPECT_EQ(Aws::DataFlow::UploadStatus::FAIL, publisher_->attemptPublish(logs_list_));

  EXPECT_EQ(LOG_PUBLISHER_RUN_CREATE_GROUP, publisher_->getRunState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_FailCreateStream)
{
  StartPublisher();

  mock_cw_->fail_cw_log_stream = true;
  mock_cw_->fail_cw_create_log_stream = true;
  EXPECT_EQ(Aws::DataFlow::UploadStatus::FAIL, publisher_->attemptPublish(logs_list_));

  EXPECT_EQ(LOG_PUBLISHER_RUN_CREATE_STREAM, publisher_->getRunState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_FailInitToken)
{
  StartPublisher();

  mock_cw_->fail_cw_init_token = true;
  EXPECT_EQ(Aws::DataFlow::UploadStatus::FAIL, publisher_->attemptPublish(logs_list_));

  EXPECT_EQ(LOG_PUBLISHER_RUN_INIT_TOKEN, publisher_->getRunState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_FailSendLogs)
{
  StartPublisher();

  mock_cw_->fail_cw_send_logs = true;
  EXPECT_EQ(Aws::DataFlow::UploadStatus::FAIL, publisher_->attemptPublish(logs_list_));

  EXPECT_EQ(LOG_PUBLISHER_RUN_INIT_TOKEN, publisher_->getRunState());
}

TEST_F(TestLogPublisherFixture, TestLogPublisher_FailNotStarted)
{
  EXPECT_EQ(Aws::DataFlow::UploadStatus::FAIL, publisher_->attemptPublish(logs_list_));
  EXPECT_TRUE(publisher_->shutdown());
  EXPECT_EQ(Aws::DataFlow::UploadStatus::FAIL, publisher_->attemptPublish(logs_list_));
}
