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


#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <chrono>
#include <thread>
#include <dataflow_lite/utils/service.h>

/**
 * Simple extension of the RunnableService used for testing
 */
class HardWorker : public RunnableService
{
public:
    HardWorker() {
      has_worked_ = false;
    };

    ~HardWorker() override = default;

    bool shutdown() override {
      std::unique_lock <std::mutex> lck(this->test_mtx);
      this->test_cv.notify_all(); // stop blocking in the work thread
      return RunnableService::shutdown();
    }

    void work() override {
      this->has_worked_ = true;

      // manually wait for shutdown
      std::unique_lock <std::mutex> lck(this->test_mtx);
      this->test_cv.wait(lck);
    }

    bool getHasWorked() {
      return this->has_worked_;
    }
private:
    bool has_worked_;
    std::condition_variable test_cv;
    mutable std::mutex test_mtx;
};

/**
 * Test fixture used to execture the HardWorker RunnableService
 */
class RunnableServiceTest : public ::testing::Test {
public:
    void SetUp() override
    {
      hard_worker = std::make_shared<HardWorker>();
      EXPECT_EQ(ServiceState::CREATED, hard_worker->getState());
      EXPECT_FALSE(hard_worker->isRunning());
    }

    void TearDown() override
    {
      hard_worker->shutdown();
      hard_worker->join();
      EXPECT_EQ(ServiceState::SHUTDOWN, hard_worker->getState());
      EXPECT_FALSE(hard_worker->isRunning());
      hard_worker.reset();
    }

protected:
    std::shared_ptr<HardWorker> hard_worker;
};

TEST_F(RunnableServiceTest, Sanity) {
  ASSERT_TRUE(true);
}

/**
 * Exercise the RunnableService start and shutdown. Verify that it ran.
 */
TEST_F(RunnableServiceTest, Test) {
  EXPECT_EQ(false, hard_worker->getHasWorked());
  EXPECT_EQ(false, hard_worker->isRunning());

  //  start the worker
  EXPECT_EQ(true, hard_worker->start());
  EXPECT_EQ(ServiceState::STARTED, hard_worker->getState());
  // expect false on subsequent start
  EXPECT_EQ(false, hard_worker->start());

  //  wait to make sure the thread was started
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  EXPECT_EQ(true, hard_worker->isRunning());
  EXPECT_EQ(ServiceState::STARTED, hard_worker->getState());

  EXPECT_EQ(true, hard_worker->shutdown());
  EXPECT_EQ(false, hard_worker->shutdown());

  hard_worker->waitForShutdown(std::chrono::milliseconds(1000)); // wait with timeout so we don't block other tests

  // did we at least work?
  EXPECT_EQ(true, hard_worker->getHasWorked());
}