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

#include <string>

namespace Aws {
namespace FileManagement {

struct TokenStoreOptions {
  /**
   * The directory the token store is backed up to upon system shutdown
   */
  std::string backup_directory;
};

struct FileManagerStrategyOptions {
  FileManagerStrategyOptions() = default;

  FileManagerStrategyOptions(
    std::string _storage_directory,
    std::string _file_prefix,
    std::string _file_extension,
    size_t _maximum_file_size,
    size_t _storage_limit)
    : storage_directory(std::move(_storage_directory)),
      file_prefix(std::move(_file_prefix)),
      file_extension(std::move(_file_extension)),
      maximum_file_size_in_kb(_maximum_file_size),
      storage_limit_in_kb(_storage_limit) {}

  /**
   * The path to the folder where all files are stored. Can be absolute or relative
   */
  std::string storage_directory;
  /**
   * The prefix appended to all files on disk
   */
  std::string file_prefix;
  /**
   * The extension of all storage files
   */
  std::string file_extension;
  /**
   * The maximum size of any single file in storage.
   * After this limit is reached the file will be rotated.
   */
  size_t maximum_file_size_in_kb{};
  /**
   * The maximum size of all files on disk.
   * After this limit is reached files will start to be deleted, oldest first.
   */
  size_t storage_limit_in_kb{};
};

static const FileManagerStrategyOptions kDefaultFileManagerStrategyOptions{"~/.ros/cwlogs", "cwlog", ".log", 1024, 1024*1024};

}  // namespace FileManagement
}  // namespace Aws
