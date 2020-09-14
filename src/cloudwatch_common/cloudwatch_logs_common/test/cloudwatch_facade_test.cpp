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

#include <aws/core/Aws.h>
#include <aws/core/client/AWSClient.h>
#include <aws/core/NoResult.h>
#include <cloudwatch_logs_common/cloudwatch_logs_client_mock.h>
#include <cloudwatch_logs_common/definitions/ros_cloudwatch_logs_errors.h>
#include <cloudwatch_logs_common/utils/cloudwatch_logs_facade.h>
#include <gtest/gtest.h>

using namespace Aws::CloudWatchLogs::Utils;

constexpr char LOG_GROUP_NAME1[] = "TestGroup1";
constexpr char LOG_GROUP_NAME2[] = "TestGroup2";
constexpr char LOG_STREAM_NAME1[] = "TestStream1";
constexpr char LOG_STREAM_NAME2[] = "TestStream2";

class TestCloudWatchFacade : public ::testing::Test
{
protected:
  std::list<Aws::CloudWatchLogs::Model::InputLogEvent> logs_list_;
  Aws::SDKOptions options_;
  std::shared_ptr<CloudWatchLogsFacade> facade_;
  std::shared_ptr<CloudWatchLogsClientMock> mock_client;
  CloudWatchLogsClientMock* mock_client_p{};

  void SetUp() override
  {
    // the tests require non-empty logs_list_
    logs_list_.emplace_back();
    logs_list_.emplace_back();

    Aws::InitAPI(options_);
    mock_client = std::make_shared<CloudWatchLogsClientMock>();
    mock_client_p = mock_client.get();
    facade_ = std::make_shared<CloudWatchLogsFacade>(mock_client);
  }

  void TearDown() override
  {
    logs_list_.clear();
    Aws::ShutdownAPI(options_);
  }
};

/*
 * SendLogsToCloudWatch Tests
 */
TEST_F(TestCloudWatchFacade, TestCWLogsFacade_SendLogsToCloudWatch_EmptyLogs)
{
    std::list<Aws::CloudWatchLogs::Model::InputLogEvent> empty_logs_list;
    Aws::String nextToken;
    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_EMPTY_PARAMETER,
        facade_->SendLogsToCloudWatch(nextToken, "", "", empty_logs_list));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_SendLogsToCloudWatch_FailedResponse)
{
    Aws::CloudWatchLogs::Model::PutLogEventsOutcome failedOutcome;
    EXPECT_CALL(*mock_client_p, PutLogEvents(testing::_))
        .WillOnce(testing::Return(failedOutcome));
    Aws::String nextToken;

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_FAILED,
        facade_->SendLogsToCloudWatch(nextToken, "", "", logs_list_));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_SendLogsToCloudWatch_SuccessResponse)
{
    Aws::CloudWatchLogs::Model::PutLogEventsResult successResult;
    Aws::CloudWatchLogs::Model::PutLogEventsOutcome successOutcome(successResult);
    EXPECT_CALL(*mock_client_p, PutLogEvents(testing::_))
        .WillOnce(testing::Return(successOutcome));
    Aws::String nextToken;

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED,
        facade_->SendLogsToCloudWatch(nextToken, "", "", logs_list_));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_SendLogsToCloudWatch_LongSuccessResponse)
{
    Aws::CloudWatchLogs::Model::PutLogEventsResult successResult;
    Aws::CloudWatchLogs::Model::PutLogEventsOutcome successOutcome(successResult);
    EXPECT_CALL(*mock_client_p, PutLogEvents(testing::_))
        .Times(2)
        .WillRepeatedly(testing::Return(successOutcome));

    Aws::String nextToken;
    std::list<Aws::CloudWatchLogs::Model::InputLogEvent> logs_list;
    for (int i=0;i<200;i++) {
        logs_list.emplace_back();
    }

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED,
        facade_->SendLogsToCloudWatch(nextToken, "", "", logs_list));
}


/*
 * CreateLogGroup Tests
 */

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CreateLogGroup_SuccessResponse)
{
    Aws::CloudWatchLogs::Model::CreateLogGroupOutcome* successOutcome =
        new Aws::CloudWatchLogs::Model::CreateLogGroupOutcome(Aws::NoResult());

    EXPECT_CALL(*mock_client_p, CreateLogGroup(testing::_))
        .WillOnce(testing::Return(*successOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED,
        facade_->CreateLogGroup(LOG_GROUP_NAME1));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CreateLogGroup_FailedResponse)
{
    auto* failedOutcome =
        new Aws::CloudWatchLogs::Model::CreateLogGroupOutcome();

    EXPECT_CALL(*mock_client_p, CreateLogGroup(testing::_))
        .WillOnce(testing::Return(*failedOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_CREATE_LOG_GROUP_FAILED,
        facade_->CreateLogGroup(LOG_GROUP_NAME1));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CreateLogGroup_AlreadyExists)
{
    Aws::Client::AWSError<Aws::CloudWatchLogs::CloudWatchLogsErrors> error
    (Aws::CloudWatchLogs::CloudWatchLogsErrors::RESOURCE_ALREADY_EXISTS, false);

    auto* failedOutcome =
        new Aws::CloudWatchLogs::Model::CreateLogGroupOutcome(error);

    EXPECT_CALL(*mock_client_p, CreateLogGroup(testing::_))
        .WillOnce(testing::Return(*failedOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_LOG_GROUP_ALREADY_EXISTS,
        facade_->CreateLogGroup(LOG_GROUP_NAME1));
}

/*
 * CheckLogGroupExists Tests
 */

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CheckLogGroupExists_FailedResponse)
{
    Aws::CloudWatchLogs::Model::DescribeLogGroupsOutcome failedOutcome;
    EXPECT_CALL(*mock_client_p, DescribeLogGroups(testing::_))
        .WillOnce(testing::Return(failedOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_FAILED,
        facade_->CheckLogGroupExists(LOG_GROUP_NAME1));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CheckLogGroupExists_LogGroupExists)
{
    Aws::CloudWatchLogs::Model::LogGroup TestGroup;
    TestGroup.SetLogGroupName(LOG_GROUP_NAME1);
    Aws::CloudWatchLogs::Model::DescribeLogGroupsResult existsResult;
    existsResult.AddLogGroups(TestGroup);
    existsResult.SetNextToken("token");
    Aws::CloudWatchLogs::Model::DescribeLogGroupsOutcome existsOutcome(existsResult);
    EXPECT_CALL(*mock_client_p, DescribeLogGroups(testing::_))
        .WillOnce(testing::Return(existsOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED,
        facade_->CheckLogGroupExists(LOG_GROUP_NAME1));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CheckLogGroupExists_LogGroupDoesntExist)
{   Aws::CloudWatchLogs::Model::LogGroup TestGroup;
    TestGroup.SetLogGroupName(LOG_GROUP_NAME1);
    Aws::CloudWatchLogs::Model::DescribeLogGroupsResult doesntExistResult;
    doesntExistResult.AddLogGroups(TestGroup);
    doesntExistResult.SetNextToken("");
    Aws::CloudWatchLogs::Model::DescribeLogGroupsOutcome doesntExistOutcome(doesntExistResult);
    EXPECT_CALL(*mock_client_p, DescribeLogGroups(testing::_))
        .WillOnce(testing::Return(doesntExistOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_LOG_GROUP_NOT_FOUND,
        facade_->CheckLogGroupExists(LOG_GROUP_NAME2));
}

/*
 * CreateLogStream Tests
 */

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CreateLogStream_SuccessResponse)
{
    Aws::CloudWatchLogs::Model::CreateLogStreamOutcome* successOutcome =
        new Aws::CloudWatchLogs::Model::CreateLogStreamOutcome(Aws::NoResult());

    EXPECT_CALL(*mock_client_p, CreateLogStream(testing::_))
        .WillOnce(testing::Return(*successOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED,
        facade_->CreateLogStream(LOG_GROUP_NAME1, LOG_STREAM_NAME1));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CreateLogStream_FailedResponse)
{
    auto* failedOutcome =
        new Aws::CloudWatchLogs::Model::CreateLogStreamOutcome();

    EXPECT_CALL(*mock_client_p, CreateLogStream(testing::_))
        .WillOnce(testing::Return(*failedOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_CREATE_LOG_STREAM_FAILED,
        facade_->CreateLogStream(LOG_GROUP_NAME1, LOG_STREAM_NAME1));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CreateLogStream_AlreadyExists)
{
    Aws::Client::AWSError<Aws::CloudWatchLogs::CloudWatchLogsErrors> error
    (Aws::CloudWatchLogs::CloudWatchLogsErrors::RESOURCE_ALREADY_EXISTS, false);

    auto* failedOutcome =
        new Aws::CloudWatchLogs::Model::CreateLogStreamOutcome(error);

    EXPECT_CALL(*mock_client_p, CreateLogStream(testing::_))
        .WillOnce(testing::Return(*failedOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_LOG_STREAM_ALREADY_EXISTS,
        facade_->CreateLogStream(LOG_GROUP_NAME1, LOG_STREAM_NAME1));
}

/*
 * CheckLogStreamExists Tests
 */

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CheckLogStreamExists_FailedResponse)
{
    Aws::CloudWatchLogs::Model::DescribeLogStreamsOutcome failedOutcome;
    Aws::CloudWatchLogs::Model::LogStream * log_stream_object = nullptr;
    EXPECT_CALL(*mock_client_p, DescribeLogStreams(testing::_))
        .WillOnce(testing::Return(failedOutcome));

    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_FAILED,
        facade_->CheckLogStreamExists(LOG_GROUP_NAME1, LOG_STREAM_NAME1, log_stream_object));
}

TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CheckLogStreamExists_LogStreamExists)
{
    Aws::CloudWatchLogs::Model::LogStream TestStream;
    TestStream.SetLogStreamName(LOG_STREAM_NAME1);
    Aws::CloudWatchLogs::Model::DescribeLogStreamsResult existsResult;
    existsResult.AddLogStreams(TestStream);
    existsResult.SetNextToken("token");
    Aws::CloudWatchLogs::Model::DescribeLogStreamsOutcome existsOutcome(existsResult);
    EXPECT_CALL(*mock_client_p, DescribeLogStreams(testing::_))
        .WillOnce(testing::Return(existsOutcome));

    auto * log_stream_object = new Aws::CloudWatchLogs::Model::LogStream();
    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_SUCCEEDED,
        facade_->CheckLogStreamExists(LOG_GROUP_NAME1, LOG_STREAM_NAME1, log_stream_object));
}


TEST_F(TestCloudWatchFacade, TestCWLogsFacade_CheckLogStreamExists_LogStreamDoesntExist)
{   Aws::CloudWatchLogs::Model::LogStream TestStream;
    TestStream.SetLogStreamName(LOG_STREAM_NAME2);
    Aws::CloudWatchLogs::Model::DescribeLogStreamsResult doesntExistResult;
    doesntExistResult.AddLogStreams(TestStream);
    Aws::CloudWatchLogs::Model::DescribeLogStreamsOutcome doesntExistOutcome(doesntExistResult);
    EXPECT_CALL(*mock_client_p, DescribeLogStreams(testing::_))
        .WillOnce(testing::Return(doesntExistOutcome));

    Aws::CloudWatchLogs::Model::LogStream * log_stream_object = nullptr;
    EXPECT_EQ(Aws::CloudWatchLogs::ROSCloudWatchLogsErrors::CW_LOGS_LOG_STREAM_NOT_FOUND,
        facade_->CheckLogStreamExists(LOG_GROUP_NAME1, LOG_STREAM_NAME1, log_stream_object));
}
