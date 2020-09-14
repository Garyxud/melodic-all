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

#ifndef LEX_COMMON_TEST__PARAMETER_READER_MOCK_H_
#define LEX_COMMON_TEST__PARAMETER_READER_MOCK_H_

#include <gmock/gmock.h>

#include <aws_common/sdk_utils/parameter_reader.h>

#include <map>
#include <vector>
#include <string>


namespace Aws
{
namespace Client
{

class ParameterReaderMock : public Aws::Client::ParameterReaderInterface
{
public:
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, std::vector<std::string>&));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, double &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, int &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, bool &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, Aws::String &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, std::string &));
  MOCK_CONST_METHOD2(ReadParam, Aws::AwsError(
      const ParameterPath &, std::map<std::string, std::string>&));
};

extern const ParameterPath user_id_key;
extern const ParameterPath bot_name_key;
extern const ParameterPath bot_alias_key;

/**
 * Set up a mock reader with the parameter paths as the key for the mock reader.
 * The return code for each of those ReadParams corresponds to the error param errors.
 *
 * @param user_id_error
 * @param bot_name_error
 * @param bot_alias_error
 * @param mock_reader [out] configured with EXPECT_CALL's for user_id, bot_name, and bot_alias
 */
void SetupMockReader(
  AwsError user_id_error,
  AwsError bot_name_error,
  AwsError bot_alias_error,
  ParameterReaderMock & mock_reader);

}  // namespace Client
}  // namespace Aws

#endif  // LEX_COMMON_TEST__PARAMETER_READER_MOCK_H_
