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
#include <gmock/gmock.h>
#include <aws_common/sdk_utils/parameter_reader.h>
#include <lex_common_test/parameter_reader_mock.h>

#include <string>

namespace Aws
{
namespace Client
{

const ParameterPath user_id_key{"lex_configuration", "user_id"};
const ParameterPath bot_name_key{"lex_configuration", "bot_name"};
const ParameterPath bot_alias_key{"lex_configuration", "bot_alias"};

void SetupMockReader(
  const AwsError user_id_error,
  const AwsError bot_name_error,
  const AwsError bot_alias_error,
  ParameterReaderMock & mock_reader)
{
  using testing::Return;
  using testing::_;
  using testing::Eq;
  EXPECT_CALL(mock_reader, ReadParam(Eq(user_id_key), testing::A<std::string&>()))
  .WillOnce(Return(user_id_error));
  EXPECT_CALL(mock_reader, ReadParam(Eq(bot_name_key), testing::A<std::string&>()))
  .WillOnce(Return(bot_name_error));
  EXPECT_CALL(mock_reader, ReadParam(Eq(bot_alias_key), testing::A<std::string&>()))
  .WillOnce(Return(bot_alias_error));
}

}  // namespace Client
}  // namespace Aws
