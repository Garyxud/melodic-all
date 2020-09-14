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

#include <aws/core/utils/logging/LogMacros.h>
#include <aws/core/utils/logging/ConsoleLogSystem.h>

#include <file_management/file_upload/file_manager_strategy.h>

using namespace Aws::FileManagement;

static const FileTokenInfo kTestToken1("fake_file", 0, false);
static const FileTokenInfo kTestToken2("fake_file", 10, true);

class TokenTest : public ::testing::Test {
public:
  void SetUp() override
  {
  }

  void TearDown() override
  {
    std::experimental::filesystem::path test_folder{"token_backups/"};
    if (std::experimental::filesystem::exists(test_folder)) {
      std::experimental::filesystem::remove_all(test_folder);
    }
  }

protected:
  std::string backup_folder = "token_backups/";
  std::string kBackupFilename = "token_store.info";
  TokenStoreOptions options{backup_folder};
};

TEST(token_test, fail_unknown_token) {
  TokenStore token_store;
  EXPECT_THROW(token_store.fail(0), std::runtime_error);
}

TEST(token_test, fail_token_twice) {
  TokenStore token_store;
  FileTokenInfo kTestToken1("fake_file", 0, false);
  auto token = token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.fail(token);
  EXPECT_THROW(token_store.fail(token), std::runtime_error);
}

TEST(token_test, resolve_then_fail_token) {
  TokenStore token_store;
  auto token = token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.resolve(token);
  EXPECT_THROW(token_store.fail(token), std::runtime_error);
}

TEST(token_test, fail_then_recover_token) {
  TokenStore token_store;
  auto token = token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.fail(token);
  EXPECT_TRUE(token_store.isTokenAvailable(kTestToken1.file_path_));
  auto popped_token = token_store.popAvailableToken(kTestToken1.file_path_);
  EXPECT_EQ(kTestToken1, popped_token);
}

TEST(token_test, resolve_unknown_token) {
  TokenStore token_store;
  EXPECT_THROW(token_store.resolve(0), std::runtime_error);
}

TEST(token_test, resolve_token) {
  TokenStore token_store;
  auto token = token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  auto resolved_token = token_store.resolve(token);
  EXPECT_EQ(kTestToken1, resolved_token);
}

TEST(token_test, resolve_token_twice) {
  TokenStore token_store;
  auto token = token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.resolve(token);
  EXPECT_THROW(token_store.resolve(token), std::runtime_error);
}

TEST(token_test, test_backup) {
  TokenStore token_store;
  token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.createToken(kTestToken2.file_path_, kTestToken2.position_, kTestToken2.eof_);
  auto backup = token_store.backup();

  EXPECT_THAT(backup, testing::ElementsAre(kTestToken1));
}

TEST(token_test, test_backup_failed_file) {
  TokenStore token_store;

  auto token1 = token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.createToken(kTestToken2.file_path_, kTestToken2.position_, kTestToken2.eof_);
  token_store.fail(token1);
  auto backup = token_store.backup();

  EXPECT_THAT(backup, testing::ElementsAre(kTestToken1));
}

TEST(token_test, test_token_restore) {
  std::vector<FileTokenInfo> backup;
  {
    TokenStore token_store;
    token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
    backup = token_store.backup();
    EXPECT_THAT(backup, testing::ElementsAre(kTestToken1));
  }
  {
    TokenStore token_store;
    token_store.restore(backup);
    EXPECT_TRUE(token_store.isTokenAvailable(kTestToken1.file_path_));
    EXPECT_EQ(kTestToken1, token_store.popAvailableToken(kTestToken1.file_path_));
  }
}

TEST(token_test, test_backup_two_files) {
  TokenStore token_store;
  FileTokenInfo test_token_2("different_file", 10, true);
  token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.createToken(test_token_2.file_path_, test_token_2.position_, test_token_2.eof_);
  auto backup = token_store.backup();
  EXPECT_THAT(backup, testing::UnorderedElementsAre(kTestToken1, test_token_2));
}

TEST_F(TokenTest, test_backup_to_disk) {
  TokenStore token_store(options);
  FileTokenInfo test_token_2("different_file", 10, true);
  token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
  token_store.createToken(test_token_2.file_path_, test_token_2.position_, test_token_2.eof_);
  EXPECT_NO_THROW(token_store.backupToDisk());
  std::string backup_file = options.backup_directory + kBackupFilename;
  EXPECT_TRUE(std::experimental::filesystem::exists(backup_file));
}

TEST_F(TokenTest, test_restore_from_disk) {
  FileTokenInfo test_token_2("different_file", 10, true);
  std::string backup_file = options.backup_directory + kBackupFilename;
  {
    TokenStore token_store(options);
    token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
    token_store.createToken(test_token_2.file_path_, test_token_2.position_, test_token_2.eof_);
    EXPECT_NO_THROW(token_store.backupToDisk());
    EXPECT_TRUE(std::experimental::filesystem::exists(backup_file));
  }
  {
    TokenStore token_store(options);
    token_store.restoreFromDisk();
    EXPECT_THAT(token_store.backup(), testing::UnorderedElementsAre(kTestToken1, test_token_2));
  }
}

TEST_F(TokenTest, test_restoring_from_disk_with_invalid_json) {
  FileTokenInfo test_token_2("different_file", 10, true);
  std::string backup_file = options.backup_directory + kBackupFilename;
  {
    TokenStore token_store(options);
    token_store.createToken(kTestToken1.file_path_, kTestToken1.position_, kTestToken1.eof_);
    token_store.createToken(test_token_2.file_path_, test_token_2.position_, test_token_2.eof_);
    EXPECT_NO_THROW(token_store.backupToDisk());
    EXPECT_TRUE(std::experimental::filesystem::exists(backup_file));
  }
  {
    std::ofstream token_store_file(backup_file, std::ios_base::app);
    token_store_file << "some other stuff" << std::endl;
    token_store_file.close();
  }
  {
    TokenStore token_store(options);
    EXPECT_NO_THROW(token_store.restoreFromDisk());
    EXPECT_THAT(token_store.backup(), testing::UnorderedElementsAre(kTestToken1, test_token_2));
  }
}



int main(int argc, char** argv)
{
  Aws::Utils::Logging::InitializeAWSLogging(
      Aws::MakeShared<Aws::Utils::Logging::ConsoleLogSystem>(
          "RunUnitTests", Aws::Utils::Logging::LogLevel::Trace));
  ::testing::InitGoogleMock(&argc, argv);
  int exitCode = RUN_ALL_TESTS();
  Aws::Utils::Logging::ShutdownAWSLogging();
  return exitCode;
}
