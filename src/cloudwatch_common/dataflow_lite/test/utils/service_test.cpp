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

#include <dataflow_lite/utils/service.h>

/**
 * Simple fixture to test the Service class basics
 */
class ServiceTest : public ::testing::Test {
public:

    void SetUp() override {
      test_service = std::make_shared<Service>();
    }

    void TearDown() override {
      test_service.reset();
    }
protected:
    std::shared_ptr<Service> test_service;
};

TEST_F(ServiceTest, Sanity) {
  ASSERT_TRUE(true);
}

/**
 * Test the lifecycle (state mgmt) of a Service
 */
TEST_F(ServiceTest, TestLifecycle) {
  EXPECT_EQ(ServiceState::CREATED, test_service->getState());
  EXPECT_TRUE(test_service->start());
  EXPECT_EQ(ServiceState::STARTED, test_service->getState());
  EXPECT_TRUE(test_service->shutdown());
  EXPECT_EQ(ServiceState::SHUTDOWN, test_service->getState());
}
