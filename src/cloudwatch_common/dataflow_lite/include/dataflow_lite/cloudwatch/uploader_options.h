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

#include <stdlib.h>
#include <stdint.h>

namespace Aws {
namespace DataFlow {

struct UploaderOptions {
  /**
   * The batch size used to upload files stored on disk.
   */
  size_t file_upload_batch_size;
  /**
   * Max number of elements in the queue for streaming / reading files on disk.
   */
  size_t file_max_queue_size;
  /**
   * Max queue size for data streamed into the service.
   */
  size_t batch_max_queue_size;
  /**
   * Max trigger size to publish: if set the streamed data is attempted to be published when this limit
   * has been reached.
   */
  size_t batch_trigger_publish_size;
  /**
   * Maximum number of items, each of size batch_max_queue_size, to store in memory to attempt to stream.
   */
  size_t stream_max_queue_size;
};

static constexpr UploaderOptions kDefaultUploaderOptions{50, 5, 1024, SIZE_MAX, 3};

}  // namespace DataFlow
}  // namespace AWS

