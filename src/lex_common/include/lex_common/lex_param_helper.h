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

#ifndef LEX_COMMON__LEX_PARAM_HELPER_H_
#define LEX_COMMON__LEX_PARAM_HELPER_H_

#include <aws_common/sdk_utils/parameter_reader.h>

#include <lex_common/lex_configuration.h>
#include <lex_common/error_codes.h>

namespace Aws
{
namespace Lex
{

/**
 * Load lex parameters from ros param server.
 *
 * @param parameter_interface to retrieve the parameters from.
 * @param lex_configuration to fill with the parameter data
 * @return SUCCESS if able to get all parameters, INVALID_LEX_CONFIGURATION otherwise
 */
ErrorCode LoadLexParameters(
  const Client::ParameterReaderInterface & parameter_interface,
  LexConfiguration & lex_configuration);

}  // namespace Lex
}  // namespace Aws

#endif  // LEX_COMMON__LEX_PARAM_HELPER_H_
