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

#ifndef LEX_COMMON_TEST__TEST_UTILS_H_
#define LEX_COMMON_TEST__TEST_UTILS_H_

#include <gmock/gmock.h>

#include <aws/core/utils/Outcome.h>
#include <aws/lex/LexRuntimeServiceClient.h>
#include <aws/core/utils/memory/AWSMemory.h>
#include <aws/lex/model/PostContentRequest.h>
#include <aws/core/utils/memory/stl/AWSStreamFwd.h>

#include <lex_common/lex_common.h>

#include <string>
#include <memory>
#include <vector>

namespace Aws
{
namespace Lex
{
class TestData
{
public:
  std::string content_type = "content_type";
  LexRuntimeService::Model::DialogState dialog_state =
    LexRuntimeService::Model::DialogState::ElicitSlot;
  std::string intent_name = "intent_name";
  std::string message = "message";
  std::string slot_to_elicit = "slot_to_elicit";
  LexRuntimeService::Model::MessageFormatType message_format =
    LexRuntimeService::Model::MessageFormatType::PlainText;
  std::string session_attributes = "session_attributes";

  inline void ConfigureExampleResult(
    Aws::LexRuntimeService::Model::PostContentResult & example_result)
  {
    example_result.SetContentType(content_type.c_str());
    example_result.SetDialogState(dialog_state);
    example_result.SetIntentName(intent_name.c_str());
    example_result.SetMessage(message.c_str());
    example_result.SetSlots("");
    example_result.SetSlotToElicit(slot_to_elicit.c_str());
    example_result.SetMessageFormat(message_format);
    example_result.SetSessionAttributes(session_attributes.c_str());
    auto io_stream = new Aws::StringStream();
    example_result.ReplaceBody(io_stream);
  }

  inline void ExpectEq(const LexResponse & response)
  {
    EXPECT_EQ(message, response.text_response);
    EXPECT_EQ(intent_name, response.intent_name);
    using Aws::LexRuntimeService::Model::DialogStateMapper::GetNameForDialogState;
    std::string dialog_state_name =
      GetNameForDialogState(dialog_state).c_str();
    EXPECT_EQ(dialog_state_name, response.dialog_state);
    using Aws::LexRuntimeService::Model::MessageFormatTypeMapper::GetNameForMessageFormatType;
    std::string message_format_name =
      GetNameForMessageFormatType(message_format).c_str();
    EXPECT_EQ(message_format_name, response.message_format_type);
    EXPECT_EQ(session_attributes, response.session_attributes);
    EXPECT_TRUE(response.slots.empty());
  }
};
}  // namespace Lex

namespace LexRuntimeService
{
namespace Model
{

using Aws::LexRuntimeService::Model::PostContentRequest;

inline std::vector<uint8_t> GetVectorFromStream(std::shared_ptr<Aws::IOStream> io_stream)
{
  std::streampos audio_size = io_stream->seekg(0, std::ios_base::end).tellg();
  std::vector<uint8_t> vec(audio_size);
  io_stream->seekg(0, std::ios_base::beg);
  io_stream->readsome(reinterpret_cast<char *>(&vec[0]), audio_size);
  io_stream->seekg(0, std::ios_base::beg);
  return vec;
}

inline bool operator==(const PostContentRequest & left, const PostContentRequest & right)
{
  std::cout << "Checking equals" << std::endl;
  bool is_equal = true;
  std::string left_user_id = left.GetUserId().c_str();
  std::string right_user_id = right.GetUserId().c_str();
  is_equal &= static_cast<bool>(left_user_id == right_user_id);
  is_equal &= static_cast<bool>(left.GetBotAlias() == right.GetBotAlias());
  is_equal &= static_cast<bool>(left.GetBotName() == right.GetBotName());
  is_equal &= static_cast<bool>(left.GetContentType() == right.GetContentType());
  is_equal &= static_cast<bool>(left.GetSessionAttributes() == right.GetSessionAttributes());
  is_equal &= static_cast<bool>(left.GetAccept() == right.GetAccept());
  EXPECT_THAT(GetVectorFromStream(left.GetBody()),
    testing::ElementsAreArray(GetVectorFromStream(right.GetBody())));
  return is_equal;
}

}  // namespace Model
}  // namespace LexRuntimeService
}  // namespace Aws

#endif  // LEX_COMMON_TEST__TEST_UTILS_H_
