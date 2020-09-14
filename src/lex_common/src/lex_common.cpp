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

#include <algorithm>
#include <string>
#include <memory>
#include <vector>

namespace Aws
{
namespace Lex
{
std::ostream & operator<<(
  std::ostream & os,
  const Aws::LexRuntimeService::Model::PostContentRequest & request)
{
  os << "Request: " << std::endl;
  os << "Bot Alias: " << request.GetBotAlias() << std::endl;
  os << "Bot Name : " << request.GetBotName() << std::endl;
  std::stringstream ss;
  ss << request.GetBody()->rdbuf();
  os << "Input data: " << ss.str() << std::endl;
  os << "User Id: " << request.GetUserId() << std::endl;
  os << "Accept Type: " << request.GetAccept() << std::endl;
  os << "Content Type: " << request.GetContentType() << std::endl;
  return os;
}

std::ostream & operator<<(
  std::ostream & os,
  const Aws::LexRuntimeService::Model::PostContentResult & result)
{
  os << "PostContentResult: " << std::endl;
  os << "Message: " << result.GetMessage() << std::endl;
  os << "Slot to elicit: " << result.GetSlotToElicit() << std::endl;
  using Aws::LexRuntimeService::Model::MessageFormatTypeMapper::GetNameForMessageFormatType;
  using Aws::LexRuntimeService::Model::DialogStateMapper::GetNameForDialogState;
  os << "Dialog State: " << GetNameForDialogState(result.GetDialogState()) << std::endl;
  os << "Message format type: " << GetNameForMessageFormatType(result.GetMessageFormat()) <<
    std::endl;
  os << "Slots: " << result.GetSlots() << std::endl;
  os << "Session Attributes: " << result.GetSessionAttributes() << std::endl;
  os << "Content Type: " << result.GetContentType() << std::endl;
  os << "Intent Name: " << result.GetIntentName() << std::endl;
  return os;
}

ErrorCode CopyResult(
  Aws::LexRuntimeService::Model::PostContentResult & result,
  LexResponse & response)
{
  using Aws::LexRuntimeService::Model::MessageFormatTypeMapper::GetNameForMessageFormatType;
  response.message_format_type = GetNameForMessageFormatType(result.GetMessageFormat()).c_str();
  response.text_response = result.GetMessage().c_str();
  
  // Copy audio stream into vector
  std::streampos audio_size = result.GetAudioStream().seekg(0, std::ios_base::end).tellg();
  response.audio_response = std::vector<uint8_t>(audio_size);
  result.GetAudioStream().seekg(0, std::ios_base::beg);
  result.GetAudioStream().read(reinterpret_cast<char *>(&response.audio_response[0]), audio_size);
  
  response.intent_name = result.GetIntentName().c_str();
  using Aws::LexRuntimeService::Model::DialogStateMapper::GetNameForDialogState;
  response.dialog_state = GetNameForDialogState(result.GetDialogState()).c_str();
  std::string session_attributes = result.GetSessionAttributes().c_str();
  
  // Strings are Base64 from lex, decode and store into a slot string
  auto slot_byte_buffer = Aws::Utils::HashingUtils::Base64Decode(result.GetSlots().c_str());
  Aws::String slot_string(reinterpret_cast<char *>(slot_byte_buffer.GetUnderlyingData()),
    slot_byte_buffer.GetLength());
  response.session_attributes = result.GetSessionAttributes().c_str();

  // Parse json into a map of slots
  if (!slot_string.empty()) {
    auto slot_json = Aws::Utils::Json::JsonValue(slot_string);
    if (slot_json.WasParseSuccessful()) {
      AWS_LOGSTREAM_DEBUG(__func__, "slot_json: " << slot_string);

      auto view = slot_json.View();
      for (auto & element : view.GetAllObjects()) {
        response.slots[element.first.c_str()] = element.second.AsString().c_str();
      }
    } else {
      AWS_LOGSTREAM_WARN(__func__, "Unable to parse slot string " << slot_string);
      return INVALID_RESULT;
    }
  }
  return SUCCESS;
}

ErrorCode LexInteractor::ConfigureAwsLex(
  std::shared_ptr<LexConfiguration> lex_configuration,
  std::shared_ptr<Aws::LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client)
{
  if (!(lex_configuration && lex_runtime_client)) {
    return ErrorCode::INVALID_LEX_CONFIGURATION;
  }
  lex_configuration_ = lex_configuration;
  lex_runtime_client_ = lex_runtime_client;
  return ErrorCode::SUCCESS;
}

ErrorCode LexInteractor::PostContent(
  const LexRequest & request,
  LexResponse & response)
{
  Aws::LexRuntimeService::Model::PostContentRequest post_content_request;
  post_content_request.WithBotAlias(lex_configuration_->bot_alias.c_str())
  .WithBotName(lex_configuration_->bot_name.c_str())
  .WithAccept(request.accept_type.c_str())
  .WithUserId(lex_configuration_->user_id.c_str());

  post_content_request.SetContentType(request.content_type.c_str());
  auto io_stream = Aws::MakeShared<Aws::StringStream>(kAllocationTag);

  if (!request.audio_request.empty()) {
    std::copy(request.audio_request.begin(),
      request.audio_request.end(), std::ostream_iterator<unsigned char>(*io_stream));
  } else {
    *io_stream << request.text_request;
  }
  post_content_request.SetBody(io_stream);
  AWS_LOGSTREAM_DEBUG(__func__, "PostContentRequest " << post_content_request);
  auto post_content_result = lex_runtime_client_->PostContent(post_content_request);
  ErrorCode result_code;
  if (post_content_result.IsSuccess()) {
    auto & result = post_content_result.GetResult();
    AWS_LOGSTREAM_DEBUG(__func__, "PostContentResult succeeded: " << result.GetMessage());
    result_code = CopyResult(result, response);

  } else {
    bool is_retryable = post_content_result.GetError().ShouldRetry();
    result_code = is_retryable ? RETRY_POST_CONTENT : FAILED_POST_CONTENT;
    AWS_LOGSTREAM_ERROR(__func__,
      "Aws Lex Error Has Occurred during LexRuntimeService->PostContent" << std::endl <<
      "PostContentResult failed: " << std::endl <<
      post_content_result.GetError() << std::endl <<
      "Request which caused error: " << std::endl <<
      post_content_request
    );
  }
  return result_code;
}

}  // namespace Lex
}  // namespace Aws
