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

#include <chrono>
#include <memory>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <dataflow_lite/dataflow/observed_queue.h>
#include <dataflow_lite/dataflow/queue_monitor.h>

using namespace Aws::DataFlow;
using namespace ::testing;

class MockObservedQueue :
  public IObservedQueue<std::string>
{
public:
  MOCK_METHOD0(clear, void (void));
  MOCK_CONST_METHOD0(size, size_t (void));
  MOCK_CONST_METHOD0(empty, bool (void));
  MOCK_METHOD2(dequeue, bool (std::string& data,
    const std::chrono::microseconds& duration));
  MOCK_METHOD1(enqueue, bool (std::string& data));
  MOCK_METHOD2(tryEnqueue,
    bool (std::string& data,
    const std::chrono::microseconds &duration));
  inline bool enqueue(std::string&&) override {
    return false;
  }

  inline bool tryEnqueue(
      std::string&& value,
      const std::chrono::microseconds&) override
  {
    return enqueue(value);
  }

  /**
   * Set the observer for the queue.
   *
   * @param status_monitor
   */
  inline void setStatusMonitor(std::shared_ptr<StatusMonitor> status_monitor) override {
    status_monitor_ = status_monitor;
  }

   /**
   * The status monitor observer.
   */
  std::shared_ptr<StatusMonitor> status_monitor_;
};

bool dequeueFunc(
  std::string& data,
  std::string actual,
  const std::chrono::microseconds&)
{
  data = std::move(actual);
  return true;
}

using std::placeholders::_1;
using std::placeholders::_2;

TEST(queue_demux_test, Sanity) {
  ASSERT_TRUE(true);
}

TEST(queue_demux_test, single_source_test)
{
  auto observed_queue = std::make_shared<MockObservedQueue>();
  std::shared_ptr<StatusMonitor> monitor;
  std::string actual = "test_string";
  EXPECT_CALL(*observed_queue, dequeue(_, _))
    .WillOnce(Invoke([=](auto && arg1, auto && arg2) { return dequeueFunc(arg1, actual, arg2); }));
  QueueMonitor<std::string> queue_monitor;
  queue_monitor.addSource(observed_queue, PriorityOptions());
  std::string data;
  EXPECT_TRUE(queue_monitor.dequeue(data, std::chrono::microseconds(0)));
  EXPECT_EQ(actual, data);
}

TEST(queue_demux_test, multi_source_test)
{
  QueueMonitor<std::string> queue_monitor;
  auto low_priority_queue = std::make_shared<StrictMock<MockObservedQueue>>();
  EXPECT_CALL(*low_priority_queue, dequeue(_, _))
    .WillOnce(Invoke([=](auto && arg1, auto && arg2) { return dequeueFunc(arg1, "low_priority", arg2); }))
    .WillRepeatedly(Return(false));
  queue_monitor.addSource(low_priority_queue, PriorityOptions(LOWEST_PRIORITY));

  auto high_priority_observed_queue = std::make_shared<StrictMock<MockObservedQueue>>();
  std::shared_ptr<StatusMonitor> monitor;
  EXPECT_CALL(*high_priority_observed_queue, dequeue(_, _))
    .WillOnce(Invoke([=](auto && arg1, auto && arg2) { return dequeueFunc(arg1, "high_priority", arg2); }))
    .WillRepeatedly(Return(false));;
  queue_monitor.addSource(high_priority_observed_queue, PriorityOptions(HIGHEST_PRIORITY));
  std::string data;
  EXPECT_TRUE(queue_monitor.dequeue(data, std::chrono::microseconds(0)));
  EXPECT_EQ("high_priority", data);
  EXPECT_TRUE(queue_monitor.dequeue(data, std::chrono::microseconds(0)));
  EXPECT_EQ("low_priority", data);
  EXPECT_FALSE(queue_monitor.dequeue(data, std::chrono::microseconds(0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleMock(&argc, argv);
  return RUN_ALL_TESTS();
}
