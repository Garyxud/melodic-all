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

#ifndef LEX_COMMON__LEX_COMMON_H_
#define LEX_COMMON__LEX_COMMON_H_

#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_common/sdk_utils/parameter_reader.h>

#include <aws/lex/LexRuntimeServiceClient.h>
#include <aws/lex/model/PostTextRequest.h>
#include <aws/lex/model/PostTextResult.h>
#include <aws/lex/model/PostContentRequest.h>
#include <aws/lex/model/PostContentResult.h>
#include <aws/core/Aws.h>
#include <aws/core/utils/Outcome.h>
#include <aws/core/utils/memory/stl/AWSString.h>
#include <aws/core/utils/HashingUtils.h>
#include <aws/core/utils/json/JsonSerializer.h>
#include <aws/core/utils/logging/AWSLogging.h>
#include <aws/core/utils/logging/LogMacros.h>

#include <lex_common/error_codes.h>
#include <lex_common/lex_configuration.h>
#include <lex_common/lex_param_helper.h>

#include <algorithm>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <regex>
#include <unordered_map>
#include <vector>

namespace Aws
{
namespace Lex
{

/**
* Aws memory allocation tag.
*/
static const char * kAllocationTag = "lex";

/**
 * Lex request datastructure.
 */
struct LexRequest
{
  std::string accept_type;
  std::string text_request;
  std::vector<uint8_t> audio_request;
  std::string content_type;
};

/**
 * Lex response datastructure.
 */
struct LexResponse
{
  std::string text_response;
  std::string message_format_type;
  std::vector<uint8_t> audio_response;
  std::string intent_name;
  std::string dialog_state;
  std::unordered_map<std::string, std::string> slots;
  std::string session_attributes;
};

/**
 * Copy a result into an AudioTextConversionResponse.
 *
 * @param result to copy to the response
 * @param response [out] result copy
 * @return error code, SUCCESS if the result is copied
 */
/**
* @brief Copy the PostContentRestult to a LexResponse.
*
* Format the contents of a PostContentResult and insert them into the respective
* Lex Response Fields.
*
* @param result to copy to the response
* @param response [out] result copy
* @return error code, SUCCESS if the result is copied,
* INVALID_RESULT if unable to parse the Slots received
*/
ErrorCode CopyResult(
  Aws::LexRuntimeService::Model::PostContentResult & result,
  LexResponse & response);

/** \addtogroup OutputStreams */
/*\@{*/
std::ostream & operator<<(
  std::ostream & os,
  const Aws::LexRuntimeService::Model::PostContentRequest & request);

std::ostream & operator<<(
  std::ostream & os,
  const Aws::LexRuntimeService::Model::PostContentResult & result);
/*\@}*/

/**
 * Interface for posting content to lex.
 */
class PostContentInterface
{
public:
  /**
   * Consume a request and establish produce a lex response.
   *
   * @param request to process
   * @param response [out] result from lex
   * @return error code
   */
  virtual ErrorCode PostContent(
    const LexRequest & request,
    LexResponse & response) = 0;
};

/**
 * Class responsible for simplified interaction with Lex.
 */
class LexInteractor : public PostContentInterface
{
protected:
  /**
   * The Lex specific configuration for the amazon bot.
   */
  std::shared_ptr<LexConfiguration> lex_configuration_;

  /**
   * The lex runtime client to use for lex api calls.
   */
  std::shared_ptr<Aws::LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client_;

public:
  /**
   * @brief Configure the Lex Interactor.
   *
   * @param lex_configuration to configure lex calls
   * @param lex_runtime_client to facilitate lex connection
   * @return INVALID_LEX_CONFIGURATION if either arguments are null
   * SUCCESS otherwise
   */
  ErrorCode ConfigureAwsLex(
    std::shared_ptr<LexConfiguration> lex_configuration,
    std::shared_ptr<Aws::LexRuntimeService::LexRuntimeServiceClient> lex_runtime_client);

  /**
   * Post content to lex given an audio text conversation request and respond to it.
   * Configures the call with the lex configuration and lex_runtime_client.
   *
   * @param request to populate the lex call with
   * @param response to fill with data received by lex
   * @return error code, SUCCESS if all data is received
   */
  ErrorCode PostContent(
    const LexRequest & request,
    LexResponse & response) override;
};

/**
 * @brief Utility function to configure a LexInteractor
 *
 * Build a LexInteractor with the parameter reader specified.
 *
 * @param params to use in the lex interactor
 * @param lex_interactor [OUT] resulting simplified lex interface
 * @return SUCCESS if the lex interactor is built, error otherwise
 */
inline ErrorCode BuildLexInteractor(
  std::shared_ptr<Client::ParameterReaderInterface> params,
  LexInteractor & lex_interactor)
{
  ErrorCode result;
  auto lex_configuration = std::make_shared<LexConfiguration>();
  result = LoadLexParameters(*params, *lex_configuration);
  if (result != SUCCESS) {
    return result;
  }
  Client::ClientConfigurationProvider configuration_provider(params);
  auto lex_runtime_client = Aws::MakeShared<Aws::LexRuntimeService::LexRuntimeServiceClient>(
    kAllocationTag, configuration_provider.GetClientConfiguration());
  lex_interactor = LexInteractor();
  lex_interactor.ConfigureAwsLex(lex_configuration, lex_runtime_client);
  return result;
}

}  // namespace Lex
}  // namespace Aws

#endif  // LEX_COMMON__LEX_COMMON_H_
