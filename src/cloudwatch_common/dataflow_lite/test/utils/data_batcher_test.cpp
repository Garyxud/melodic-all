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

#include <dataflow_lite/utils/data_batcher.h>
#include <memory>
#include <string>
#include <stdexcept>

/**
 * Simple extension to test basic functionality
 */
class TestBatcher : public DataBatcher<int> {
public:

  explicit TestBatcher(size_t max_allowable_batch_size = DataBatcher::kDefaultMaxBatchSize,
          size_t publish_trigger_size = DataBatcher::kDefaultTriggerSize) : DataBatcher(max_allowable_batch_size, publish_trigger_size) {
    pub_called = 0;
  }

  bool start() override {
    return true;
  }

  bool shutdown() override {
    this->resetBatchedData();
    return true;
  }

  bool publishBatchedData() override {
    std::lock_guard<std::recursive_mutex> lk(mtx);
    pub_called++;
    this->resetBatchedData();
    return true;
  }
  int pub_called;
};

/**
 * Test fixture
 */
class DataBatcherTest : public ::testing::Test {
public:
    void SetUp() override
    {
      test_batcher = std::make_shared<TestBatcher>();
      test_batcher->start();
    }

    void TearDown() override
    {
      test_batcher->shutdown();
    }

protected:
    std::shared_ptr<TestBatcher> test_batcher;
};

TEST_F(DataBatcherTest, Sanity) {
  ASSERT_TRUE(true);
}

TEST_F(DataBatcherTest, Init) {

  EXPECT_EQ((size_t) TestBatcher::kDefaultTriggerSize,  test_batcher->getTriggerBatchSize());
  EXPECT_EQ((size_t) TestBatcher::kDefaultMaxBatchSize, test_batcher->getMaxAllowableBatchSize());
  EXPECT_EQ(0u, test_batcher->getCurrentBatchSize());
}

TEST_F(DataBatcherTest, TestMaxSizeClear) {

  size_t new_max = 10;
  test_batcher->setMaxAllowableBatchSize(new_max);
  EXPECT_EQ(new_max, test_batcher->getMaxAllowableBatchSize());

  for(size_t i=0; i<new_max; i++) {
    test_batcher->batchData(i);
    EXPECT_EQ(i+1, test_batcher->getCurrentBatchSize());
  }

  test_batcher->batchData(42);
  EXPECT_EQ(0u, test_batcher->getCurrentBatchSize());
  EXPECT_EQ(0, test_batcher->pub_called);
}

TEST_F(DataBatcherTest, TestPublishTrigger) {

  size_t new_max = 10;
  test_batcher->setTriggerBatchSize(new_max);
  EXPECT_EQ(new_max, test_batcher->getTriggerBatchSize());

  for(size_t i=0; i<new_max-1; i++) {
  test_batcher->batchData(i);
  EXPECT_EQ(i+1, test_batcher->getCurrentBatchSize());
  }

  test_batcher->batchData(42);
  EXPECT_EQ(0u, test_batcher->getCurrentBatchSize());
  EXPECT_EQ(1, test_batcher->pub_called);
}

TEST_F(DataBatcherTest, TestValidateArguments) {

  EXPECT_THROW(TestBatcher::validateConfigurableSizes(0, 0), std::invalid_argument);
  EXPECT_THROW(TestBatcher::validateConfigurableSizes(1, 0), std::invalid_argument);
  EXPECT_THROW(TestBatcher::validateConfigurableSizes(0, 1), std::invalid_argument);
  EXPECT_THROW(TestBatcher::validateConfigurableSizes(1, 1), std::invalid_argument);
  EXPECT_THROW(TestBatcher::validateConfigurableSizes(1, 2), std::invalid_argument);

  EXPECT_NO_THROW(TestBatcher::validateConfigurableSizes(2, 1));
}

TEST_F(DataBatcherTest, TestBatcherArguments) {

  size_t max = 10;
  EXPECT_THROW(test_batcher->setMaxAllowableBatchSize(0), std::invalid_argument);
  EXPECT_NO_THROW(test_batcher->setMaxAllowableBatchSize(max));
  EXPECT_EQ(max, test_batcher->getMaxAllowableBatchSize());

  size_t trigger = 5;
  EXPECT_THROW(test_batcher->setTriggerBatchSize(0), std::invalid_argument);
  EXPECT_THROW(test_batcher->setTriggerBatchSize(trigger + max), std::invalid_argument);
  EXPECT_NO_THROW(test_batcher->setTriggerBatchSize(trigger));
  EXPECT_EQ(trigger, test_batcher->getTriggerBatchSize());

  EXPECT_THROW(TestBatcher(100, 200), std::invalid_argument);
}