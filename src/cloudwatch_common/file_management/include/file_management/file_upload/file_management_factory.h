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

#pragma once

#include <file_management/file_upload/file_manager.h>
#include <dataflow_lite/dataflow/status_monitor.h>
#include <file_management/file_upload/file_upload_streamer.h>
#include <dataflow_lite/dataflow/observed_queue.h>
#include <dataflow_lite/dataflow/queue_monitor.h>

namespace Aws {
namespace FileManagement {

/**
 * Create a file upload manager complete with a file status monitor attached to the file_manager,
 * and a task based queue.
 *
 * @tparam T the type of messages the file uploader will handle
 * @param file_manager to use as the source of these messages
 * @return a shared pointer to a configured file upload manager.
 * @throws Invalid Argument for the file manager
 */
template<
  typename T,
  typename O,
  class = typename std::enable_if<std::is_base_of<DataReader<T>, O>::value, O>::type>
std::shared_ptr<FileUploadStreamer<T>> createFileUploadStreamer(
  std::shared_ptr<O> file_manager,
  FileUploadStreamerOptions file_manager_options)
  {
  if (!file_manager) {
    throw "Invalid file_manager";
  }
  // Create a file upload manager to handle uploading a file.
  auto file_upload_manager =
      std::make_shared<Aws::FileManagement::FileUploadStreamer<T>>(
          file_manager,
          file_manager_options);
  return file_upload_manager;
}

}  // namespace FileManagement
}  // namespace Aws

