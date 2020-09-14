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
#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <lex_common_test/parameter_reader_mock.h>
#include <lex_common_test/test_logger.h>

#include <lex_common/lex_param_helper.h>

#include <aws/core/config/AWSProfileConfigLoader.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogLevel.h>

#include <string>
#include <memory>

using Aws::Lex::ErrorCode;
using Aws::Lex::LexConfiguration;
using testing::_;
using testing::Eq;
using Aws::Client::ParameterPath;
using Aws::Client::ParameterReaderMock;
using Aws::AwsError;

TEST(ParameterTest, LoadLexParamsUserIdFail) {
  ParameterReaderMock mock_reader;
  SetupMockReader(AwsError::AWS_ERR_NOT_FOUND, AwsError::AWS_ERR_OK, AwsError::AWS_ERR_OK,
    mock_reader);
  LexConfiguration lex_config;
  ASSERT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION, LoadLexParameters(mock_reader, lex_config));
}

TEST(ParameterTest, LoadLexParamsBotNameFail) {
  ParameterReaderMock mock_reader;
  SetupMockReader(AwsError::AWS_ERR_OK, AwsError::AWS_ERR_NOT_FOUND, AwsError::AWS_ERR_OK,
    mock_reader);
  LexConfiguration lex_config;
  ASSERT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION, LoadLexParameters(mock_reader, lex_config));
}

TEST(ParameterTest, LoadLexParamsBotAliasFail) {
  ParameterReaderMock mock_reader;
  SetupMockReader(AwsError::AWS_ERR_OK, AwsError::AWS_ERR_OK, AwsError::AWS_ERR_NOT_FOUND,
    mock_reader);
  LexConfiguration lex_config;
  ASSERT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION, LoadLexParameters(mock_reader, lex_config));
}

/**
 * Test all params are read successfully from the lex param loader.
 */
TEST(ParameterTest, LoadLexParamsSuccess) {
  ParameterReaderMock mock_reader;
  const std::string user_id = "user_id";
  const std::string bot_name = "bot_name";
  const std::string bot_alias = "bot_alias";

  auto read_std_str =
    [](std::string expected, const ParameterPath & name, std::string & out) -> AwsError {
      out = expected;
      return AwsError::AWS_ERR_OK;
    };
  using namespace std::placeholders;
  using testing::Invoke;
  EXPECT_CALL(mock_reader, ReadParam(Eq(Aws::Client::user_id_key), testing::A<std::string&>()))
  .WillOnce(Invoke(std::bind(read_std_str, user_id, _1, _2)));
  EXPECT_CALL(mock_reader, ReadParam(Eq(Aws::Client::bot_name_key), testing::A<std::string&>()))
  .WillOnce(Invoke(std::bind(read_std_str, bot_name, _1, _2)));
  EXPECT_CALL(mock_reader, ReadParam(Eq(Aws::Client::bot_alias_key), testing::A<std::string&>()))
  .WillOnce(Invoke(std::bind(read_std_str, bot_alias, _1, _2)));
  LexConfiguration lex_config;
  ASSERT_EQ(ErrorCode::SUCCESS, LoadLexParameters(mock_reader, lex_config));
  EXPECT_EQ(user_id, lex_config.user_id);
  EXPECT_EQ(bot_name, lex_config.bot_name);
  EXPECT_EQ(bot_alias, lex_config.bot_alias);
}

int main(int argc, char ** argv)
{
  Aws::Utils::Logging::InitializeAWSLogging(
    std::make_shared<Aws::Utils::Logging::TestLogSystem>(Aws::Utils::Logging::LogLevel::Trace));
  Aws::SDKOptions options;
  Aws::InitAPI(options);
  // The following line must be executed to initialize Google Mock
  // (and Google Test) before running the tests.
  ::testing::InitGoogleMock(&argc, argv);
  auto result = RUN_ALL_TESTS();
  Aws::Utils::Logging::ShutdownAWSLogging();
  Aws::ShutdownAPI(options);
  return result;
}
