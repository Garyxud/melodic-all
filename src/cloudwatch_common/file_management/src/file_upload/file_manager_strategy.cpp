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
#include <iostream>
#include <regex>
#include <fstream>
#include <file_management/file_upload/file_manager.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <iomanip>
#include "file_management/file_upload/file_manager_strategy.h"

#define KB_TO_BYTES(x) (static_cast<size_t>(x) << 10u)

namespace fs = std::experimental::filesystem;

namespace Aws {
namespace FileManagement {

static const std::string kTokenStoreFile("token_store.info");

void sanitizePath(std::string & path) {
  if (path.back() != '/') {
    path += '/';
  }
  if (path.front() == '~') {
    char * home = getenv("HOME");
    if (nullptr == home) {
      AWS_LOG_WARN(__func__, "No HOME environment variable set. Attempting to use ROS_HOME instead.");
      home = getenv("ROS_HOME");
    }
    if (nullptr != home) {
      path.replace(0, 1, home);
    } else {
      throw std::runtime_error("The storage directory path uses '~' but no HOME environment variable set.");
    }
  }
}

TokenStore::TokenStore(const TokenStoreOptions &options) : options_{options}{
  validateOptions();
  initializeBackupDirectory();
}

void TokenStore::validateOptions() {
  sanitizePath(options_.backup_directory);
}

void TokenStore::initializeBackupDirectory() {
  auto backup_directory = std::experimental::filesystem::path(options_.backup_directory);
  if (!std::experimental::filesystem::exists(backup_directory)) {
    AWS_LOG_INFO(__func__, "TokenStore backup directory %s does not exist, creating.", backup_directory.c_str());
    std::experimental::filesystem::create_directories(backup_directory);
  }
}

bool TokenStore::isTokenAvailable(const std::string &file_name) const {
  return !(staged_tokens_.find(file_name) == staged_tokens_.end());
}

FileTokenInfo TokenStore::popAvailableToken(const std::string &file_name) {
  auto file_token_info = staged_tokens_[file_name];
  staged_tokens_.erase(file_name);
  return file_token_info;
}

void printCache(std::unordered_map<DataToken, FileTokenInfo> token_store,
  std::unordered_map<std::string, std::list<DataToken>> file_tokens,
  std::unordered_map<std::string, FileTokenInfo> staged_tokens_) {
  {
  std::stringstream ss;
  for (auto& token_info : token_store) {
    ss << token_info.first << ": " << token_info.second.file_path_ << ", " << token_info.second.position_ << std::endl;
  }
  AWS_LOG_DEBUG(__func__,
               "Cache Info: token_store \n %s", ss.str().c_str());
  }
  {
    std::stringstream ss;
    for (auto &file_token : file_tokens) {
      ss << file_token.first << ": ";
      for (auto &tokens : file_token.second) {
        ss << tokens;
      }
      ss << std::endl;
    }
    AWS_LOG_DEBUG(__func__,
                 "Cache Info: file_tokens \n %s", ss.str().c_str());
  }
  std::stringstream ss;
  for (auto& token_info : staged_tokens_) {
    ss << token_info.first << ": " << token_info.second.file_path_ << ", " << token_info.second.position_ << std::endl;
  }
  AWS_LOG_DEBUG(__func__,
               "Cache Info: staged_tokens \n %s", ss.str().c_str());
}

// NOLINTNEXTLINE(google-runtime-int)
DataToken TokenStore::createToken(const std::string &file_name, const long streampos, bool is_eof) {
  AWS_LOG_DEBUG(__func__, "Creating token");
  std::mt19937_64 rand( rand_device() );
  DataToken token = rand();
  token_store_.emplace(token, FileTokenInfo(file_name, streampos, is_eof));
  if (file_tokens_.find(file_name) == file_tokens_.end()) {
    file_tokens_[file_name] = std::list<DataToken>();
  }
  file_tokens_[file_name].push_back(token);
  return token;
}

FileTokenInfo TokenStore::fail(const DataToken &token) {
  AWS_LOG_DEBUG(__func__, "Marking token %i as failed (data did not upload successfully)", token);
  if (token_store_.find(token) == token_store_.end()) {
    throw std::runtime_error("DataToken not found");
  }
  FileTokenInfo token_info = token_store_[token];
  token_store_.erase(token);
  if (file_tokens_.find(token_info.file_path_) != file_tokens_.end()) {
    const std::string &file_path = token_info.file_path_;
    staged_tokens_[file_path] = token_info;
    file_tokens_.erase(file_path);
  }
  return token_info;
}

FileTokenInfo TokenStore::resolve(const DataToken &token) {
  AWS_LOG_DEBUG(__func__,
               "Resolving token %i", token);

  if (token_store_.find(token) == token_store_.end()) {
    throw std::runtime_error("DataToken not found");
  }
  FileTokenInfo token_info = token_store_[token];
  const std::string &file_path = token_info.file_path_;

  if (file_tokens_.find(file_path) == file_tokens_.end()) {
    throw std::runtime_error("Could not find token set for file: " + file_path);
  }
  // this find should be O(1), as we expect data to be resolved in order
  auto list = file_tokens_[file_path];
  list.erase(std::find(list.begin(), list.end(), token));

  if (file_tokens_[file_path].empty()) {
    file_tokens_.erase(file_path);
  }
  token_store_.erase(token);
//  printCache(token_store_, file_tokens_, staged_tokens_);
  return token_info;
}

std::vector<FileTokenInfo> TokenStore::backup() {
  auto vector_size = file_tokens_.size() + staged_tokens_.size();
  std::vector<FileTokenInfo> token_backup(vector_size);
  auto it = token_backup.begin();
  for (auto& token : staged_tokens_) {
    *it++ = token.second;
  }
  for (auto& token : file_tokens_) {
    *it++ = token_store_[*token.second.begin()];
  }
  return token_backup;
}

void TokenStore::backupToDisk() {
  auto file_path = std::experimental::filesystem::path(options_.backup_directory + kTokenStoreFile);
  std::vector<FileTokenInfo> token_store_backup = backup();
  if (std::experimental::filesystem::exists(file_path)) {
    std::experimental::filesystem::remove(file_path);
  }
  std::ofstream token_store_file;
  token_store_file.open(file_path);
  if (token_store_file.bad()) {
    AWS_LOG_WARN(__func__, "Unable to open file %s to backup the token store", file_path.c_str());
    return;
  }
  for (const FileTokenInfo &token_info : token_store_backup) {
    token_store_file << token_info.serialize() << std::endl;
  }
  token_store_file.close();
}

void TokenStore::restore(const std::vector<FileTokenInfo> &file_tokens) {
  for (auto& file_token: file_tokens) {
    staged_tokens_[file_token.file_path_] = file_token;
  }
}

void TokenStore::restoreFromDisk() {
  // read through each line.
  // For each line the first 4 bytes are position, next byte is eof, the remainder are a string of file path
  // Will this change depending on OS / platform? Will that matter? Should I use another serialization library.
  auto file_path = std::experimental::filesystem::path(options_.backup_directory + kTokenStoreFile);
  if (!std::experimental::filesystem::exists(file_path)) {
    return;
  }
  AWS_LOG_INFO(__func__, "Loading existing token store from: %s", file_path.c_str());
  std::ifstream token_store_read_stream = std::ifstream(file_path);
  std::vector<FileTokenInfo> file_tokens;
  std::string line;
  while (!token_store_read_stream.eof()) {
    std::getline(token_store_read_stream, line);
    if (!line.empty()) {
      FileTokenInfo token_info;
      try {
        token_info.deserialize(line);
      } catch (const std::runtime_error & e) {
        AWS_LOG_ERROR(__func__, "Unable to parse token backup line: %s. Skipping.", line.c_str());
        continue;
      }
      file_tokens.push_back(token_info);
    }
  }
  token_store_read_stream.close();
  restore(file_tokens);
  std::experimental::filesystem::remove(file_path);
}


FileManagerStrategy::FileManagerStrategy(const FileManagerStrategyOptions &options) {
  stored_files_size_ = 0;
  active_write_file_size_ = 0;
  options_ = options;
  validateOptions();
}

bool FileManagerStrategy::start() {
  initializeStorage();
  initializeTokenStore();
  discoverStoredFiles();
  rotateWriteFile();
  return Service::start();
}

void FileManagerStrategy::validateOptions() {
  sanitizePath(options_.storage_directory);
}

void FileManagerStrategy::initializeStorage() {
  AWS_LOG_DEBUG(__func__, "Initializing offline file storage in directory %s", options_.storage_directory.c_str());
  auto storage = std::experimental::filesystem::path(options_.storage_directory);
  if (!std::experimental::filesystem::exists(storage)) {
    AWS_LOG_INFO(__func__, "File storage directory %s does not exist, creating.", storage.c_str());
    std::experimental::filesystem::create_directories(storage);
    stored_files_size_ = 0;
  }
}

void FileManagerStrategy::initializeTokenStore() {
  AWS_LOG_DEBUG(__func__, "Initializing token store in directory %s", options_.storage_directory.c_str());
  TokenStoreOptions options{options_.storage_directory};
  token_store_ = std::make_unique<TokenStore>(options);
  token_store_->restoreFromDisk();
}

bool FileManagerStrategy::isDataAvailable() {
  AWS_LOG_DEBUG(__func__,
               "Is Data Available: %s, %s %s",
               !active_read_file_.empty() ? "true" : "false",
               !stored_files_.empty() ? "true" : "false",
               active_write_file_size_ > 0 ? "true" : "false");
  return !active_read_file_.empty() || !stored_files_.empty() || active_write_file_size_ > 0;
}

void FileManagerStrategy::write(const std::string &data) {
  try {
    checkIfWriteFileShouldRotate(data.size());
    checkIfStorageLimitHasBeenReached(data.size());

    std::lock_guard<std::mutex> write_lock(active_write_file_mutex_);
    std::ofstream log_file;
    AWS_LOG_DEBUG(__func__, "Writing data to file: %s", active_write_file_.c_str())
    log_file.open(active_write_file_, std::ios_base::app);
    if (log_file.bad()) {
      AWS_LOG_WARN(__func__, "Unable to open file: %s", active_write_file_.c_str());
    }
    log_file << data << std::endl;
    log_file.close();
    active_write_file_size_ += data.size();
  } catch(const std::ios_base::failure& e) {
    AWS_LOG_WARN(__func__, "FileManagerStrategy::write caught std::ios_base::failure");
  }
}

DataToken FileManagerStrategy::read(std::string &data) {
  std::lock_guard<std::mutex> read_lock(active_read_file_mutex_);
  if (active_read_file_.empty()) {
    active_read_file_ = getFileToRead();
    // if the file is still empty, return an empty token.
    if (active_read_file_.empty()) {
      return 0;
    }
    active_read_file_stream_ = std::make_unique<std::ifstream>(active_read_file_);
  }
  AWS_LOG_DEBUG(__func__, "Reading from active log file: %s", active_read_file_.c_str());
  DataToken token;
  if (token_store_->isTokenAvailable(active_read_file_)) {
    FileTokenInfo file_token = token_store_->popAvailableToken(active_read_file_);
    active_read_file_stream_->seekg(file_token.position_);
  }
  int position = active_read_file_stream_->tellg();
  auto file_size = active_read_file_stream_->seekg(0, std::ifstream::end).tellg();
  active_read_file_stream_->seekg(position, std::ifstream::beg);
  std::getline(*active_read_file_stream_, data);
  int next_position = active_read_file_stream_->tellg();
  token = token_store_->createToken(active_read_file_, position, next_position >= file_size);

  if (next_position >= file_size) {
    auto file_loc = std::find(stored_files_.begin(), stored_files_.end(), active_read_file_);
    if (file_loc != stored_files_.end()) {
      stored_files_.erase(file_loc);
    }
    active_read_file_.clear();
    active_read_file_stream_ = nullptr;

  }
  return token;
}

void FileManagerStrategy::resolve(const DataToken &token, bool is_success) {
  if (is_success) {
    try {
      auto file_info = token_store_->resolve(token);
      if (file_info.eof_) {
        deleteFile(file_info.file_path_);
      }
    } catch(std::runtime_error& exception) {
      AWS_LOG_WARN(__func__,
                   "FileManagerStrategy resolve caught runtime_error attempting to resolve token %i",
                   token);
    }
  } else {
    try {
      auto file_info = token_store_->fail(token);
      if (file_info.eof_) {
        AWS_LOG_DEBUG(__func__,
                      "Failed last token %d, pushing file to stored: %s", token, file_info.file_path_.c_str());
        stored_files_.push_back(file_info.file_path_);
      }
    } catch(std::runtime_error& exception) {
      AWS_LOG_WARN(__func__,
                   "FileManagerStrategy resolve caught runtime_error attempting to resolve token %i",
                   token);
    }
  }
}

bool FileManagerStrategy::shutdown() {
  bool b = Service::shutdown();
  token_store_->backupToDisk();
  return b;
}


void FileManagerStrategy::discoverStoredFiles() {
  for (const auto &entry : fs::directory_iterator(options_.storage_directory)) {
    const fs::path &path = entry.path();
    std::regex name_expr(
      options_.file_prefix +
      "[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}-[0-9]{1}" +
      options_.file_extension);
    if (std::regex_match(path.filename().string(), name_expr)) {
      addFilePathToStorage(path);
    }
  }
}

void FileManagerStrategy::deleteFile(const std::string &file_path) {
  AWS_LOG_DEBUG(__func__, "Deleting file: %s", file_path.c_str());
  const uintmax_t file_size = fs::file_size(file_path);
  fs::remove(file_path);
  stored_files_size_ -= file_size;
}

std::string FileManagerStrategy::getFileToRead() {
  // if we have stored files, pop from the end of the list and return that filename
  // if we do not have stored files, and the active file has data, switch active file and return the existing active file.
  if (!stored_files_.empty()) {
    stored_files_.sort();
    const std::string newest_file = stored_files_.back();
    stored_files_.pop_back();
    return newest_file;
  }

  std::lock_guard<std::mutex> write_lock(active_write_file_mutex_);
  if (active_write_file_size_ > 0) {
    const std::string file_path = active_write_file_;
    rotateWriteFile();
    return file_path;
  }

  throw std::runtime_error("No files available for reading");
}

void FileManagerStrategy::addFilePathToStorage(const fs::path &file_path) {
  stored_files_.push_back(file_path);
  stored_files_size_ += fs::file_size(file_path);
}

void FileManagerStrategy::rotateWriteFile() {
  AWS_LOG_DEBUG(__func__, "Rotating offline storage file");
  using std::chrono::system_clock;
  time_t tt = system_clock::to_time_t (system_clock::now());
  std::ostringstream oss;
  auto tm = *std::localtime(&tt);
  oss << std::put_time(&tm, "%F_%H-%M-%S");
  uint count = 0;
  std::string original_file_name = options_.file_prefix + oss.str();
  std::string file_name = original_file_name + "-" + std::to_string(count);
  std::string file_path = options_.storage_directory + file_name + options_.file_extension;
  while (fs::exists(file_path)) {
    ++count;
    file_name = original_file_name + "-" + std::to_string(count);
    file_path = options_.storage_directory + file_name + options_.file_extension;
  }

  if (!active_write_file_.empty()) {
    stored_files_.push_back(active_write_file_);
    stored_files_size_ += active_write_file_size_;
  }

  AWS_LOG_DEBUG(__func__, "New active offline storage file is: %s", file_path.c_str());
  active_write_file_ = file_path;
  active_write_file_size_ = 0;
}

void FileManagerStrategy::checkIfWriteFileShouldRotate(const uintmax_t &new_data_size) {
  std::lock_guard<std::mutex> write_lock(active_write_file_mutex_);
  const uintmax_t new_file_size = active_write_file_size_ + new_data_size;
  const uintmax_t max_file_size_in_bytes = KB_TO_BYTES(options_.maximum_file_size_in_kb);
  if (new_file_size > max_file_size_in_bytes) {
    AWS_LOG_DEBUG(__func__, "New file size %d is larger than max file size %d", new_file_size, max_file_size_in_bytes);
    rotateWriteFile();
  }
}

void FileManagerStrategy::checkIfStorageLimitHasBeenReached(const uintmax_t &new_data_size) {
  const uintmax_t new_storage_size = stored_files_size_ + active_write_file_size_ + new_data_size;
  const uintmax_t max_storage_size_in_bytes = KB_TO_BYTES(options_.storage_limit_in_kb);
  if (new_storage_size > max_storage_size_in_bytes) {
    AWS_LOG_WARN(__func__, "Maximum offline storage limit has been reached. Deleting oldest log file.");
    deleteOldestFile();
  }
}

void FileManagerStrategy::deleteOldestFile() {
  if (!stored_files_.empty()) {
    std::lock_guard<std::mutex> read_lock(active_read_file_mutex_);
    stored_files_.sort();
    const std::string oldest_file = stored_files_.front();
    if (oldest_file == active_read_file_) {
      active_read_file_.clear();
      active_read_file_stream_ = nullptr;
    }
    stored_files_.pop_front();
    AWS_LOG_INFO(__func__, "Deleting oldest file: %s", oldest_file.c_str());
    deleteFile(oldest_file);
  }
}

}  // namespace FileManagement
}  // namespace Aws
