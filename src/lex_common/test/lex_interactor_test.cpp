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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <istream>
#include <ostream>
#include <vector>

namespace
{

template <typename T>
class Mover
{
public:
  Mover(T && object)
    : object(std::move(object)),
      valid(true) {}

  Mover(const Mover<T> & other)
    : object(const_cast<T &&>(other.object)),
      valid(true)
  {
    assert(other.valid);
    other.valid = false;
  }

  Mover & operator=(const Mover & other)
  {
    assert(other.valid);
    object = const_cast<T &&>(other.object);
    other.valid = false;
    valid = true;
  }

  T & get()
  {
    assert(valid);
    return object;
  }

  const T & get() const
  {
    assert(valid);
    return object;
  }

private:
  T object;
  mutable bool valid;
};

template <typename T>
inline Mover<T> Movable(T && object)
{
  return Mover<T>(std::move(object));
}

}

namespace Aws
{
namespace Lex
{
using Aws::LexRuntimeService::Model::PostContentOutcome;
using Aws::LexRuntimeService::Model::PostContentRequest;
using Aws::LexRuntimeService::Model::PostContentResult;
using Aws::LexRuntimeService::LexRuntimeServiceErrors;
using testing::Return;

class TestLexConfiguration
{
public:
  std::string user_id = "user_id";
  std::string bot_alias = "bot_alias";
  std::string bot_name = "bot_name";

  void ConfigureLexConfiguration(LexConfiguration & lex_configuration)
  {
    lex_configuration.user_id = user_id;
    lex_configuration.bot_alias = bot_alias;
    lex_configuration.bot_name = bot_name;
  }
};

class LexRuntimeServiceClientMock
  : public Aws::LexRuntimeService::LexRuntimeServiceClient
{
public:
  virtual ~LexRuntimeServiceClientMock() = default;

  MOCK_CONST_METHOD1(PostContent_, Mover<PostContentOutcome>(const PostContentRequest & request));

  PostContentOutcome PostContent(const PostContentRequest & request) const
  {
    return std::move(PostContent_(request).get());
  }
};

class TestLexInteractor : public ::testing::Test
{
protected:
  std::shared_ptr<LexConfiguration> lex_configuration;
  std::shared_ptr<LexRuntimeServiceClientMock> lex_runtime_client;
  LexInteractor lex_interactor;
  LexRequest default_request;
  PostContentRequest default_expected_request;
  TestData test_data;
  Aws::LexRuntimeService::Model::PostContentResult result;

  void SetUp() override
  {
    lex_runtime_client = std::make_shared<LexRuntimeServiceClientMock>();
    lex_configuration = std::make_shared<LexConfiguration>();
    TestLexConfiguration configuration;
    configuration.ConfigureLexConfiguration(*lex_configuration);

    default_request.accept_type = "accept_type";
    default_request.content_type = "content_type";
    default_request.text_request = "";

    default_expected_request
    .WithBotAlias(lex_configuration->bot_alias.c_str())
    .WithBotName(lex_configuration->bot_name.c_str())
    .WithAccept(default_request.accept_type.c_str())
    .WithUserId(lex_configuration->user_id.c_str());
    default_expected_request.SetBotAlias(lex_configuration->bot_alias.c_str());
    default_expected_request.SetContentType(default_request.content_type.c_str());
    test_data.ConfigureExampleResult(result);
  }
};

TEST_F(TestLexInteractor, TestLexInteractorFailedConfiguration) {
  EXPECT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION,
    lex_interactor.ConfigureAwsLex(nullptr, nullptr));
  EXPECT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION,
    lex_interactor.ConfigureAwsLex(lex_configuration, nullptr));
  EXPECT_EQ(ErrorCode::INVALID_LEX_CONFIGURATION,
    lex_interactor.ConfigureAwsLex(nullptr, lex_runtime_client));
}

TEST_F(TestLexInteractor, TestLexInteractorPostContentText) {
  ASSERT_EQ(ErrorCode::SUCCESS, lex_interactor.ConfigureAwsLex(lex_configuration,
    lex_runtime_client));
  default_request.audio_request = std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7};
  auto lex_configuration = std::make_shared<LexConfiguration>();
  auto io_stream = std::make_shared<StringStream>();
  std::copy(default_request.audio_request.begin(),
    default_request.audio_request.end(), std::ostream_iterator<unsigned char>(*io_stream));
  default_expected_request.SetBody(io_stream);
  EXPECT_CALL(*lex_runtime_client, PostContent_(default_expected_request))
  .WillOnce(testing::Return(Movable(PostContentOutcome(std::move(result)))));
  // Aws::Client::AWSError<LexRuntimeServiceErrors>
  LexResponse response;
  ASSERT_EQ(ErrorCode::SUCCESS, lex_interactor.PostContent(default_request, response));
  test_data.ExpectEq(response);
}

TEST_F(TestLexInteractor, TestLexInteractorPostContentAudio) {
  default_request.audio_request = std::vector<uint8_t>{1, 2, 3, 4, 5, 6, 7};
  auto io_stream = std::make_shared<StringStream>();
  std::copy(default_request.audio_request.begin(),
    default_request.audio_request.end(), std::ostream_iterator<unsigned char>(*io_stream));
  default_expected_request.SetBody(io_stream);

  ASSERT_EQ(ErrorCode::SUCCESS, lex_interactor.ConfigureAwsLex(lex_configuration,
    lex_runtime_client));
  TestData test_data;
  PostContentResult result;
  test_data.ConfigureExampleResult(result);
  EXPECT_CALL(*lex_runtime_client, PostContent_(default_expected_request))
  .WillOnce(testing::Return(Movable(PostContentOutcome(std::move(result)))));

  LexResponse response;
  ASSERT_EQ(ErrorCode::SUCCESS, lex_interactor.PostContent(default_request, response));
  test_data.ExpectEq(response);
}

TEST_F(TestLexInteractor, TestLexInteractorPostContentFailed) {
  ASSERT_EQ(ErrorCode::SUCCESS, lex_interactor.ConfigureAwsLex(lex_configuration,
    lex_runtime_client));

  EXPECT_CALL(*lex_runtime_client, PostContent_(testing::_))
  .WillOnce(testing::Return(Movable(PostContentOutcome(
        Aws::Client::AWSError<LexRuntimeServiceErrors>(LexRuntimeServiceErrors::ACCESS_DENIED,
        false)))));
  LexResponse response;
  ASSERT_EQ(ErrorCode::FAILED_POST_CONTENT, lex_interactor.PostContent(default_request, response));
}

TEST_F(TestLexInteractor, TestLexInteractorPostContentRetry) {
  ASSERT_EQ(ErrorCode::SUCCESS, lex_interactor.ConfigureAwsLex(lex_configuration,
    lex_runtime_client));

  EXPECT_CALL(*lex_runtime_client, PostContent_(testing::_))
  .WillOnce(testing::Return(Movable(PostContentOutcome(
        Aws::Client::AWSError<LexRuntimeServiceErrors>(LexRuntimeServiceErrors::REQUEST_TIMEOUT,
        true)))));
  LexResponse response;
  ASSERT_EQ(ErrorCode::RETRY_POST_CONTENT, lex_interactor.PostContent(default_request, response));
}

}  // namespace Lex
}  // namespace Aws
