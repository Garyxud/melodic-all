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
namespace CloudWatchMetrics {

/** Contains all options for the file uploader and file management
 * in one options object.
 */
struct CloudWatchOptions {
  /**
   * All options for the FileUpload system
   */
  Aws::DataFlow::UploaderOptions uploader_options{};
  /**
   * All options for the FileManager system
   */
  Aws::FileManagement::FileManagerStrategyOptions file_manager_strategy_options;
};

static const Aws::FileManagement::FileManagerStrategyOptions kDefaultMetricFileManagerStrategyOptions{
  "~/.ros/cwmetrics",
  "cwmetric",
  Aws::FileManagement::kDefaultFileManagerStrategyOptions.file_extension,
  Aws::FileManagement::kDefaultFileManagerStrategyOptions.maximum_file_size_in_kb,
  Aws::FileManagement::kDefaultFileManagerStrategyOptions.storage_limit_in_kb
};

static const CloudWatchOptions kDefaultCloudWatchOptions{
  Aws::DataFlow::kDefaultUploaderOptions,
  kDefaultMetricFileManagerStrategyOptions
};

}  // namespace CloudWatchMetrics
}  // namespace Aws
