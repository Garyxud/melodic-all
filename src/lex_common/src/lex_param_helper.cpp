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

#include <lex_common/lex_configuration.h>
#include <lex_common/lex_param_helper.h>
#include <aws/core/utils/logging/LogMacros.h>

namespace Aws
{
namespace Lex
{

ErrorCode LoadLexParameters(
  const Client::ParameterReaderInterface & parameter_interface,
  LexConfiguration & lex_configuration)
{
  bool is_invalid = false;
  is_invalid |= static_cast<bool>(parameter_interface.ReadParam(
      lex_configuration.kBotAliasKey_, lex_configuration.bot_alias));
  is_invalid |= static_cast<bool>(parameter_interface.ReadParam(
      lex_configuration.kBotNameKey_, lex_configuration.bot_name));
  is_invalid |= static_cast<bool>(parameter_interface.ReadParam(
      lex_configuration.kUserIdKey_, lex_configuration.user_id));
  if (is_invalid) {
    AWS_LOG_WARN(__func__, "Lex configuration not fully specified");
    return INVALID_LEX_CONFIGURATION;
  }
  return SUCCESS;
}

}  // namespace Lex
}  // namespace Aws
