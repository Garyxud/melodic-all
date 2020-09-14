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

#include <list>

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <experimental/filesystem>

#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <aws/logs/model/InputLogEvent.h>
#include <file_management/file_upload/file_manager_strategy.h>

using namespace Aws::FileManagement;

class MockFileManagerStrategy : public FileManagerStrategy {
public:
  explicit MockFileManagerStrategy(const FileManagerStrategyOptions &options) : FileManagerStrategy(options) {}

  std::string getFileToRead() {
    return FileManagerStrategy::getFileToRead();
  }

  std::string getActiveWriteFile() {
    return FileManagerStrategy::getActiveWriteFile();
  }
};

class FileManagerStrategyTest : public ::testing::Test {
public:
  void SetUp() override
  {
  }

  void TearDown() override
  {
    std::experimental::filesystem::path test_folder{"log_tests/"};
    std::experimental::filesystem::remove_all(test_folder);
  }

protected:
  std::string folder = "log_tests/";
  std::string prefix = "test";
  std::string extension = ".log";
  uint max_file_size = 1024;
  uint storage_limit = max_file_size * 10;
  FileManagerStrategyOptions options{folder, prefix, extension, max_file_size, storage_limit};
};


TEST_F(FileManagerStrategyTest, restart_without_token) {
  const std::string data1 = "test_data_1";
  const std::string data2 = "test_data_2";
  {
    FileManagerStrategy file_manager_strategy(options);
    EXPECT_NO_THROW(file_manager_strategy.start());
    file_manager_strategy.write(data1);
    file_manager_strategy.write(data2);
  }
  {
    FileManagerStrategy file_manager_strategy(options);
    EXPECT_NO_THROW(file_manager_strategy.start());
    std::string result1, result2;
    file_manager_strategy.read(result1);
    file_manager_strategy.read(result2);
    EXPECT_EQ(data1, result1);
    EXPECT_EQ(data2, result2);
  }
}

// @todo (rddesmon) preserve file of tokens on restart
//TEST_F(FileManagerStrategyTest, restart_with_token) {
//  const std::string data1 = "test_data_1";
//  const std::string data2 = "test_data_2";
//  {
//    FileManagerStrategy file_manager_strategy(options);
//    EXPECT_NO_THROW(file_manager_strategy.start());
//    file_manager_strategy.write(data1);
//    file_manager_strategy.write(data2);
//    std::string result1;
//    DataToken token1 = file_manager_strategy.read(result1);
//    EXPECT_EQ(data1, result1);
//  }
//  {
//    FileManagerStrategy file_manager_strategy(options);
//    EXPECT_NO_THROW(file_manager_strategy.start());
//    std::string result2;
//    DataToken token2 = file_manager_strategy.read(result2);
//    EXPECT_EQ(data2, result2);
//  }
//}

TEST_F(FileManagerStrategyTest, fail_token_restart_from_last_location) {
  const std::string data1 = "test_data_1";
  const std::string data2 = "test_data_2";
  FileManagerStrategy file_manager_strategy(options);
  EXPECT_NO_THROW(file_manager_strategy.start());
  file_manager_strategy.write(data1);
  file_manager_strategy.write(data2);
  std::string result1;
  DataToken token1 = file_manager_strategy.read(result1);
  EXPECT_EQ(data1, result1);
  file_manager_strategy.resolve(token1, true);
  std::string result2, result3, result4;
  DataToken token2 = file_manager_strategy.read(result2);
  EXPECT_EQ(data2, result2);

  file_manager_strategy.resolve(token2, false);
  // Token was failed, should be re-read.
  DataToken token3 = file_manager_strategy.read(result3);
  EXPECT_EQ(data2, result3);
  file_manager_strategy.resolve(token3, false);
  // Token was failed, should be re-read.
  EXPECT_TRUE(file_manager_strategy.isDataAvailable());
  DataToken token4 = file_manager_strategy.read(result4);
  EXPECT_EQ(data2, result4);
  EXPECT_FALSE(file_manager_strategy.isDataAvailable());
  file_manager_strategy.resolve(token4, true);
  EXPECT_FALSE(file_manager_strategy.isDataAvailable());
}

/**
 * Test that the upload complete with CW Failure goes to a file.
 */
TEST_F(FileManagerStrategyTest, start_success) {
  FileManagerStrategy file_manager_strategy(options);
  EXPECT_NO_THROW(file_manager_strategy.start());
}

TEST_F(FileManagerStrategyTest, discover_stored_files) {
  const std::string test_data = "test_data";
  {
    FileManagerStrategy file_manager_strategy(options);
    EXPECT_NO_THROW(file_manager_strategy.start());
    file_manager_strategy.write(test_data);
  }
  {
    FileManagerStrategy file_manager_strategy(options);
    EXPECT_NO_THROW(file_manager_strategy.start());
    EXPECT_TRUE(file_manager_strategy.isDataAvailable());
    std::string result;
    DataToken token = file_manager_strategy.read(result);
    EXPECT_EQ(test_data, result);
    file_manager_strategy.resolve(token, true);
  }
}

TEST_F(FileManagerStrategyTest, get_file_to_read_gets_newest) {
  namespace fs = std::experimental::filesystem;
  const uint max_file_size_in_kb = 25;
  options.maximum_file_size_in_kb = max_file_size_in_kb;
  {
    MockFileManagerStrategy file_manager_strategy(options);
    file_manager_strategy.start();
    std::ostringstream ss_25_kb;
    for (int i = 0; i < 1024; i++) {
      ss_25_kb << "This is 25 bytes of data.";
    }
    std::string string_25_kb = ss_25_kb.str();

    for (int i = 0; i < 5; i++) {
      file_manager_strategy.write(string_25_kb);
    }

    long file_count = std::distance(fs::directory_iterator(folder), fs::directory_iterator{});
    EXPECT_EQ(5, file_count);

    std::vector<std::string> file_paths;
    for (const auto &entry : fs::directory_iterator(folder)) {
      const fs::path &path = entry.path();
      file_paths.push_back(path);
    }

    std::sort(file_paths.begin(), file_paths.end());
    const std::string expected_active_write_file = file_paths.end()[-1];
    const std::string expected_file_to_be_read = file_paths.end()[-2];

    EXPECT_EQ(expected_active_write_file, file_manager_strategy.getActiveWriteFile());
    EXPECT_EQ(expected_file_to_be_read, file_manager_strategy.getFileToRead());
  }
}


TEST_F(FileManagerStrategyTest, rotate_large_files) {
  namespace fs = std::experimental::filesystem;
  const uint max_file_size_in_kb = 10;
  options.maximum_file_size_in_kb = max_file_size_in_kb;
  {
    FileManagerStrategy file_manager_strategy(options);
    file_manager_strategy.start();
    std::ostringstream data1ss;
    for (int i = 0; i < 1024; i++) {
      data1ss << "This is some long data that is longer than 10 bytes";
    }
    std::string data1 = data1ss.str();
    file_manager_strategy.write(data1);
    long file_count = std::distance(fs::directory_iterator(folder), fs::directory_iterator{});
    EXPECT_EQ(1, file_count);
    std::ostringstream data2ss;
    for (int i = 0; i < 1024; i++) {
      data2ss << "This is some additional data that is also longer than 10 bytes";
    }
    std::string data2 = data2ss.str();
    file_manager_strategy.write(data2);
    file_count = std::distance(fs::directory_iterator(folder), fs::directory_iterator{});
    EXPECT_EQ(2, file_count);
  }
}

TEST_F(FileManagerStrategyTest, resolve_token_deletes_file) {
  const std::string test_data = "test_data";
  {
    FileManagerStrategy file_manager_strategy(options);
    file_manager_strategy.start();
    EXPECT_FALSE(file_manager_strategy.isDataAvailable());
    file_manager_strategy.write(test_data);
    EXPECT_TRUE(file_manager_strategy.isDataAvailable());
    std::string result;
    DataToken token = file_manager_strategy.read(result);
    file_manager_strategy.resolve(token, true);
  }
  {
    FileManagerStrategy file_manager_strategy(options);
    file_manager_strategy.start();
    EXPECT_FALSE(file_manager_strategy.isDataAvailable());
  }
}

TEST_F(FileManagerStrategyTest, on_storage_limit_delete_oldest_file) {
  namespace fs = std::experimental::filesystem;
  const uint max_file_size_in_kb = 50;
  const uint storage_limit = 150;
  options.maximum_file_size_in_kb = max_file_size_in_kb;
  options.storage_limit_in_kb = storage_limit;
  {
    FileManagerStrategy file_manager_strategy(options);
    file_manager_strategy.start();
    std::ostringstream ss_25_kb;
    for (int i = 0; i < 1024; i++) {
      ss_25_kb << "This is 25 bytes of data.";
    }
    std::string string_25_kb = ss_25_kb.str();
    file_manager_strategy.write(string_25_kb);
    long file_count = std::distance(fs::directory_iterator(folder), fs::directory_iterator{});
    EXPECT_EQ(1, file_count);

    for (int i = 0; i < 5; i++) {
      file_manager_strategy.write(string_25_kb);
    }

    file_count = std::distance(fs::directory_iterator(folder), fs::directory_iterator{});
    EXPECT_EQ(3, file_count);

    std::vector<std::string> file_paths;
    for (const auto &entry : fs::directory_iterator(folder)) {
      const fs::path &path = entry.path();
      file_paths.push_back(path);
    }

    std::sort(file_paths.begin(), file_paths.end());
    const std::string file_to_be_deleted = file_paths[0];

    file_manager_strategy.write(string_25_kb);
    file_count = std::distance(fs::directory_iterator(folder), fs::directory_iterator{});
    EXPECT_EQ(3, file_count);

    for (const auto &entry : fs::directory_iterator(folder)) {
      const std::string file_path = entry.path();
      EXPECT_TRUE(file_path != file_to_be_deleted);
    }
  }
}

class SanitizePathTest : public ::testing::Test {
public:
  void SetUp() override
  {
  }

  void TearDown() override
  {
  }

};

TEST_F(SanitizePathTest, sanitizePath_home_set) {
  std::string test_path = "~/dir/";
  char * original_home = getenv("HOME");
  setenv("HOME", "/home", 1);

  sanitizePath(test_path);

  // Cleanup before test
  if (nullptr != original_home) {
    setenv("HOME", original_home, 1);
  } else {
    unsetenv("HOME");
  }

  EXPECT_STREQ(test_path.c_str(), "/home/dir/");
}

TEST_F(SanitizePathTest, sanitizePath_home_not_set_roshome_set) {
  std::string test_path = "~/dir/";
  char * original_home = getenv("HOME");
  char * original_ros_home = getenv("ROS_HOME");
  unsetenv("HOME");
  setenv("ROS_HOME", "/ros_home", 1);

  sanitizePath(test_path);

  // Cleanup before test
  if (nullptr != original_home) {
    setenv("HOME", original_home, 1);
  }
  if (nullptr != original_ros_home) {
    setenv("ROS_HOME", original_ros_home, 1);
  } else {
    unsetenv("ROS_HOME");
  }

  EXPECT_STREQ(test_path.c_str(), "/ros_home/dir/");
}


TEST_F(SanitizePathTest, sanitizePath_home_not_set_roshome_not_set) {
  std::string test_path = "~/dir/";
  char * original_home = getenv("HOME");
  char * original_ros_home = getenv("ROS_HOME");
  unsetenv("HOME");
  unsetenv("ROS_HOME");


  ASSERT_THROW(sanitizePath(test_path), std::runtime_error);

  // Cleanup before test
  if (nullptr != original_home) {
    setenv("HOME", original_home, 1);
  }
  if (nullptr != original_ros_home) {
    setenv("ROS_HOME", original_ros_home, 1);
  }

}

TEST_F(SanitizePathTest, sanitizePath_adds_trailing_slash) {
  std::string test_path = "/test/path";
  sanitizePath(test_path);
  EXPECT_STREQ(test_path.c_str(), "/test/path/");
}
