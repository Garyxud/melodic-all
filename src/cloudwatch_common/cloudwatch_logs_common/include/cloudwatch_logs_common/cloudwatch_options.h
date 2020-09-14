/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#pragma once

#include <dataflow_lite/cloudwatch/uploader_options.h>
#include <file_management/file_manager_options.h>

namespace Aws {
namespace CloudWatchLogs {

/** Contains all options for the file uploader and file management
 * in one options object.
 */
struct CloudWatchOptions {
  CloudWatchOptions() = default;

  CloudWatchOptions(const Aws::DataFlow::UploaderOptions & _uploader_options,
                    Aws::FileManagement::FileManagerStrategyOptions _strategy_options)
    : uploader_options(_uploader_options), file_manager_strategy_options(std::move(_strategy_options)) {}

  /**
   * All options for the FileUpload system
   */
  Aws::DataFlow::UploaderOptions uploader_options{};
  /**
   * All options for the FileManager system
   */
  Aws::FileManagement::FileManagerStrategyOptions file_manager_strategy_options;
};

static const CloudWatchOptions kDefaultCloudWatchOptions{
  Aws::DataFlow::kDefaultUploaderOptions,
  Aws::FileManagement::kDefaultFileManagerStrategyOptions
};

}  // namespace CloudWatchLogs
}  // namespace Aws
