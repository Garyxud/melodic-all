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

#ifndef LEX_COMMON__ERROR_CODES_H_
#define LEX_COMMON__ERROR_CODES_H_

namespace Aws
{
namespace Lex
{

/**
 * Error codes for lex.
 */
enum ErrorCode
{
  SUCCESS = 0,
  INVALID_RESULT,
  FAILED_POST_CONTENT,
  RETRY_POST_CONTENT,
  INVALID_LEX_CONFIGURATION,
  INVALID_ARGUMENT,
};

}  // namespace Lex
}  // namespace Aws

#endif  // LEX_COMMON__ERROR_CODES_H_
