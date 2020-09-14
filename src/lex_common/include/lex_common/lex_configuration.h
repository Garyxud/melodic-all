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

#ifndef LEX_COMMON__LEX_CONFIGURATION_H_
#define LEX_COMMON__LEX_CONFIGURATION_H_

#include <aws_common/sdk_utils/parameter_reader.h>

#include <string>
#include <vector>

namespace Aws
{
namespace Lex
{

using Client::ParameterPath;
/**
 * Lex configuration namespace.
 */
constexpr char kLexConfigurationNamespace[] = "lex_configuration";

/**
* Configuration to make calls to lex.
*/
class LexConfiguration
{
public:
  /**
   * \defgroup Formatted ROS parameter keys for lex configuration.
   */
  /**@{*/
  ParameterPath kUserIdKey_{kLexConfigurationNamespace, "user_id"};
  ParameterPath kBotNameKey_{kLexConfigurationNamespace, "bot_name"};
  ParameterPath kBotAliasKey_{kLexConfigurationNamespace, "bot_alias"};
  /** @}*/

  /**
   * The user id to call lex with. Unique per caller.
   */
  std::string user_id;

  /**
   * The lex bot to use.
   */
  std::string bot_name;

  /**
   * The lex alias of the bot to use.
   */
  std::string bot_alias;
};
}  // namespace Lex
}  // namespace Aws

#endif  // LEX_COMMON__LEX_CONFIGURATION_H_
