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
namespace Aws {
namespace DataFlow {

enum PriorityLevel : uint {
  LOWEST_PRIORITY = 0,
  LOW_PRIORITY,
  MEDIUM_PRIORITY,
  HIGH_PRIORITY,
  HIGHEST_PRIORITY
};

struct PriorityOptions {
  explicit PriorityOptions(PriorityLevel level = MEDIUM_PRIORITY) {
    priority_level = level;
  }
  PriorityLevel priority_level;

  inline bool operator > (const PriorityOptions &other) const {
    return priority_level > other.priority_level;
  }

  inline bool operator < (const PriorityOptions &other) const {
    return priority_level < other.priority_level;
  }
};

}  // namespace DataFlow
}  // namespace Aws
