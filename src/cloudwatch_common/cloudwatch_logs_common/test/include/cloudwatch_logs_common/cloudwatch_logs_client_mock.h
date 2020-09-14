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

#include <gmock/gmock.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/Outcome.h>
#include <aws/core/client/AWSClient.h>
#include <aws/logs/CloudWatchLogsClient.h>
#include <aws/logs/model/CreateLogGroupRequest.h>
#include <aws/logs/model/DescribeLogGroupsRequest.h>
#include <aws/logs/model/CreateLogStreamRequest.h>
#include <aws/logs/model/DescribeLogStreamsRequest.h>



class CloudWatchLogsClientMock : public Aws::CloudWatchLogs::CloudWatchLogsClient
{
public:
    MOCK_CONST_METHOD1(
        PutLogEvents,
        Aws::CloudWatchLogs::Model::PutLogEventsOutcome(
            const Aws::CloudWatchLogs::Model::PutLogEventsRequest & request));
    MOCK_CONST_METHOD1(
        CreateLogGroup,
        Aws::CloudWatchLogs::Model::CreateLogGroupOutcome(
            const Aws::CloudWatchLogs::Model::CreateLogGroupRequest & request));
    MOCK_CONST_METHOD1(
        DescribeLogGroups,
        Aws::CloudWatchLogs::Model::DescribeLogGroupsOutcome(
            const Aws::CloudWatchLogs::Model::DescribeLogGroupsRequest & request));
    MOCK_CONST_METHOD1(
        CreateLogStream,
        Aws::CloudWatchLogs::Model::CreateLogStreamOutcome(
            const Aws::CloudWatchLogs::Model::CreateLogStreamRequest & request));
    MOCK_CONST_METHOD1(
        DescribeLogStreams,
        Aws::CloudWatchLogs::Model::DescribeLogStreamsOutcome(
            const Aws::CloudWatchLogs::Model::DescribeLogStreamsRequest & request));
};

