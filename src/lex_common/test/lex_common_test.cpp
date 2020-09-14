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


#include <lex_common/lex_common.h>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <aws/core/utils/Outcome.h>
#include <aws/lex/LexRuntimeServiceClient.h>
#include <aws/core/utils/memory/AWSMemory.h>
#include <aws/lex/model/PostContentResult.h>

#include <lex_common_test/test_utils.h>
#include <lex_common_test/parameter_reader_mock.h>

#include <memory>
#include <string>
#include <utility>
#include <istream>
#include <ostream>

namespace Aws
{
namespace Lex
{
class LexCommonTestFixture : public ::testing::Test
{
protected:
  TestData test_data;
  Aws::LexRuntimeService::Model::PostContentResult result;

  void SetUp() override
  {
    test_data.ConfigureExampleResult(result);
  }
};

TEST_F(LexCommonTestFixture, CopyResultFailureInvalidJson) {
  {
    auto io_stream = new Aws::StringStream();
    *io_stream << "test_text";
    result.ReplaceBody(io_stream);
  }
  std::string invalid_json = "This causes failure not valid json";
  const unsigned char * json = reinterpret_cast<const unsigned char *>(
    invalid_json.c_str());
  Aws::Utils::ByteBuffer bb(json, invalid_json.size());
  result.SetSlots(Aws::Utils::HashingUtils::Base64Encode(bb));
  LexResponse response;
  ASSERT_EQ(ErrorCode::INVALID_RESULT, CopyResult(result, response));
}

TEST_F(LexCommonTestFixture, CopyResultSuccessValidJson) {
  {
    auto io_stream = new Aws::StringStream();
    *io_stream << "test_text";
    result.ReplaceBody(io_stream);
  }
  Aws::Utils::Json::JsonValue value;
  value.WithString("key", "value");
  auto value_str = value.View().WriteReadable();
  const unsigned char * json = reinterpret_cast<const unsigned char *>(
    value_str.c_str());
  Aws::Utils::ByteBuffer bb(json, value_str.size());
  result.SetSlots(Aws::Utils::HashingUtils::Base64Encode(bb));
  LexResponse response;
  ASSERT_EQ(ErrorCode::SUCCESS, CopyResult(result, response));
  EXPECT_EQ(1, response.slots.size());
  EXPECT_EQ("value", response.slots["key"]);
}

TEST_F(LexCommonTestFixture, CopyResultSuccessNoSlots) {
  {
    auto io_stream = new Aws::StringStream();
    *io_stream << "test_text";
    result.ReplaceBody(io_stream);
  }
  LexResponse response;
  ASSERT_EQ(ErrorCode::SUCCESS, CopyResult(result, response));
  test_data.ExpectEq(response);
}

TEST(BuildLexInteractor, TestBuildInteractorError) {
  auto param_mock = std::make_shared<Aws::Client::ParameterReaderMock>();
  Aws::Client::SetupMockReader(AwsError::AWS_ERR_OK,
    AwsError::AWS_ERR_OK,
    AwsError::AWS_ERR_NOT_FOUND,
    *param_mock);

  LexInteractor lex_interactor;
  ASSERT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION,
    BuildLexInteractor(param_mock, lex_interactor));
}

TEST(BuildLexInteractor, TestBuildInteractorSuccess) {
  auto param_mock = std::make_shared<Aws::Client::ParameterReaderMock>();
  Aws::Client::SetupMockReader(AwsError::AWS_ERR_OK,
    AwsError::AWS_ERR_OK,
    AwsError::AWS_ERR_OK,
    *param_mock);
  ::testing::DefaultValue<AwsError>::Set(AwsError::AWS_ERR_OK);

  LexInteractor lex_interactor;
  ASSERT_EQ(ErrorCode::SUCCESS,
    BuildLexInteractor(param_mock, lex_interactor));
}

}  // namespace Lex
}  // namespace Aws
