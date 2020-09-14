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
#include <dataflow_lite/dataflow/sink.h>
#include <dataflow_lite/dataflow/source.h>

namespace Aws {
namespace DataFlow {

template <typename O>
class OutputStage;
template <typename I>
class InputStage;

template <typename O>
class OutputStage {
public:
  std::shared_ptr<Sink<O>> getSink() {
    return sink_;
  }

  template<
      class T>
  inline
  typename std::enable_if<std::is_base_of<Sink<O>, T>::value, std::shared_ptr<T>>::type
  setSink(std::shared_ptr<T> sink) {
    sink_ = sink;
    return sink;
  }
 private:
  std::shared_ptr<Sink<O>> sink_;
};

template <typename I>
class InputStage {
 public:
  inline std::shared_ptr<Source<I>> getSource() {
    return source_;
  }
  inline void setSource(std::shared_ptr<Source<I>> source) {
    source_ = source;
  }
 private:
  std::shared_ptr<Source<I>> source_;
};

}  // namespace DataFlow
}  // namespace Aws
