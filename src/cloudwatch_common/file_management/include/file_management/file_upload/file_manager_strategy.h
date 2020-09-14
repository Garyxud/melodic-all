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

#include <iostream>
#include <fstream>
#include <list>
#include <unordered_map>
#include <set>
#include <memory>
#include <random>
#include <mutex>
#include <experimental/filesystem>
#include <dataflow_lite/utils/service.h>
#include <file_management/file_manager_options.h>
#include <aws/core/utils/json/JsonSerializer.h>

namespace Aws {
namespace FileManagement {

enum TokenStatus {
  ACTIVE,
  INACTIVE
};

using DataToken = uint64_t;

static constexpr const char* kPositionKey = "position";
static constexpr const char* kEofKey = "eof";
static constexpr const char* kFilePathKey = "file_path";

/**
 * Sanitizes the path provided by making sure it ends with a '/' character
 * and also replacing the '~', if it's at the front of the string, with the
 * value of the $HOME environment variable. If $HOME is not set it attempts 
 * to use $ROS_HOME. If niether is set then it throws a runtime exception.
 */
void sanitizePath(std::string & path);

/**
 * Stores file token information for the purpose of tracking read locations.
 */
class FileTokenInfo {
public:

  FileTokenInfo() = default;

  // NOLINTNEXTLINE(google-runtime-int)
  explicit FileTokenInfo(const std::string &file_path, const long position, const bool eof) :
  file_path_{file_path},
  position_(position),
  eof_(eof)
  {

  };

  // NOLINTNEXTLINE(google-runtime-int)
  explicit FileTokenInfo(std::string &&file_path, const long position, const bool eof) :
      file_path_{std::move(file_path)},
      position_(position),
      eof_(eof)
  {

  };

  FileTokenInfo(const FileTokenInfo &info) = default;

  FileTokenInfo & operator=(const FileTokenInfo & other) = default;

  ~FileTokenInfo() = default;

  /**
   * Serializes a tokens information into a JSON string
   * @return A JSON representation of the token
   */
  std::string serialize() const {
    Aws::Utils::Json::JsonValue json_value;
    const Aws::String file_path(file_path_.c_str());
    json_value
        .WithInt64(kPositionKey, position_)
        .WithBool(kEofKey, eof_)
        .WithString(kFilePathKey, file_path);
    return std::string(json_value.View().WriteCompact().c_str());
  }

  /** 
   * Takes a Token JSON string and sets this tokens
   * values based on that. 
   */
  void deserialize(const std::string& token_info_json) {
    const Aws::String aws_str(token_info_json.c_str());
    const Aws::Utils::Json::JsonValue json_value(aws_str);
    if (!json_value.WasParseSuccessful()) {
      throw std::runtime_error("Unable to parse JSON");
    }
    auto view = json_value.View();
    position_ = view.GetInt64(kPositionKey);
    eof_ = view.GetBool(kEofKey);
    file_path_ = view.GetString(kFilePathKey).c_str();
  }

  /**
   * The file that this token corrosponds to
   */
  std::string file_path_;

  /** 
   * The position in the file that this token corrosponds to
   */
  int64_t position_ = 0;

  /**
   * Set to true if this is the last token for the file
   */
  bool eof_{};
};

inline bool operator==(const FileTokenInfo& lhs, const FileTokenInfo& rhs){
  return lhs.eof_ == rhs.eof_ && lhs.position_ == rhs.position_ && lhs.file_path_ == rhs.file_path_;
}

inline bool operator!=(const FileTokenInfo& lhs, const FileTokenInfo& rhs){ return !(lhs == rhs); }

class DataManagerStrategy : public Service {
public:
  DataManagerStrategy() = default;
  ~DataManagerStrategy() override = default;

  virtual bool isDataAvailable() = 0;

  virtual DataToken read(std::string &data) = 0;

  virtual void write(const std::string &data) = 0;

  /**
   * Mark a token as 'done' so the DataManager knows the piece of
   * data associated with that token can be cleaned up.
   * @param token
   * @throws std::runtime_error for token not found
   */
  virtual void resolve(const DataToken &token, bool is_success) = 0;
};

/**
 * Stores all tokens and manages failed or loaded tokens.
 */
class TokenStore {
public:

  TokenStore() = default;

  explicit TokenStore(const TokenStoreOptions &options);

  /**
   * Checks if a token is availa for a specific file
   * @param file_name to lookup
   * @return true if a staged token is available to read for that file
   */
  bool isTokenAvailable(const std::string &file_name) const;

  /**
   * @param file_name to lookup
   * @return the file token for that file
   */
  FileTokenInfo popAvailableToken(const std::string &file_name);

  /**
   * Create a token with the file name, stream position, and whether or not this is the last token in the file.
   *
   * @param file_name
   * @param streampos
   * @param is_eof
   * @return
   */
  // NOLINTNEXTLINE(google-runtime-int)
  DataToken createToken(const std::string &file_name, const long streampos, bool is_eof);

  /**
   * Mark a token as failed so the FileManagerStrategy knows to keep the 
   * data around. 
   *
   * @param token to fail
   * @return token info that was failed
   * @throws std::runtime_error if token not found
   */
  FileTokenInfo fail(const DataToken &token);

  /**
   * Resolve a token, marking it as complete so the TokenStore can forget 
   * about it and its related data.  
   * @param token
   * @return token info which was resolved
   * @throws std::runtime_error if token not found
   */
  FileTokenInfo resolve(const DataToken &token);

  /**
   * Backup the first unacked token and all failed tokens into a vector.
   * @return vector to tokens
   */
  std::vector<FileTokenInfo> backup();

  /**
   * Backup the token store to a file on disk
   */
  void backupToDisk();

  /**
   * Restore tokens from a vector
   * @param std::vector<FileTokenInfo>
   */
  void restore(const std::vector<FileTokenInfo> &file_tokens);

  /**
   * Restore the token store from a file saved to disk
   */
  void restoreFromDisk();



private:
  /**
   * Ensure that all options are valid. Also sanitizes options 
   * that are slightly incorrect. 
   */
  void validateOptions();
  
  /** 
   * Creates the directory where the token store is backed up 
   * to upon shutdown. 
   */
  void initializeBackupDirectory();

  /**
   * A map of tokens to File Path and Position that the token corrosponds to
   */
  std::unordered_map<DataToken, FileTokenInfo> token_store_;

  /**
   * A map of file paths to the tokens that corrospond to that file. So that when
   * all tokens for a file have been resolved the system knows that the file 
   * can be deleted
   */
  std::unordered_map<std::string, std::list<DataToken>> file_tokens_;

  /**
   * A map of file paths to the File path and position where the reader is up to.
   * When a token fails to resolve (thus the data failed to upload), it is added
   * to this map. 
   * Then in the read function if there is anything in this map it will go to that 
   * location in the file and attempt to send it again. It does this first before
   * reading any other data. 
   */
  std::unordered_map<std::string, FileTokenInfo> staged_tokens_;

  /** 
   * A struct of options to initialize the token store with
   */
  TokenStoreOptions options_;

  /**
   * A random number generator, used to generate tokens (which are just uint64's)
   */
  std::random_device rand_device;
};

/**
 * Manages how files are split up, which files to write to and read when requested.
 */
class FileManagerStrategy : public DataManagerStrategy {
public:
  explicit FileManagerStrategy(const FileManagerStrategyOptions &options);

  ~FileManagerStrategy() override = default;

  /**
   * Starts the FileManagerStrategy, this does any initialization I/O tasks required.
   * It is separate from the constructor to keep the construction of this class lightweight.
   * It checks if there are any existing offline storage files on disk, initializes the 
   * token store, and ensures there is a new file ready to write to if uploads fail. 
   * @return bool whether it started successfully or not
   */
  bool start() override;

  /**
   * Returns true if there is offline data on disk awaiting to be uploaded, false otherwise. 
   * @return bool if there is data available
   */
  bool isDataAvailable() override;

  /**
   * Reads a line of data from file storage. The most recent data in storage is read first. 
   * This returns a DataToken which should be passed to resolve() when the caller is done 
   * with it.
   * @param std::string a string of data which is read from storage
   * @return DataToken that should be passed to resolve() when done
   */
  DataToken read(std::string &data) override;

  /**
   * Write a line of data to to file storage.
   * @param std::string a string of data to write to storage
   */
  void write(const std::string &data) override;

  /** 
   * Resolves a DataToken. If the data associated with the token was successfully sent 
   * then the FileManagerStrategy knows it now longer needs to hold onto it. 
   * If the data was not successfully sent then the FileManagerStrategy knows it needs
   * to hold onto it and re-send that data in the next read. 
   * @param DataToken the token to be resolved
   * @param is_success whether the data was successfully sent or not
   */
  void resolve(const DataToken &token, bool is_success) override;

  /**
   * Saves all in flight tokens and information out to disk so that on startup the 
   * FileManagerStrategy can start reading from the same file location that it was up 
   * to previously. 
   * @return bool whether it shutdown successfully
   */
  bool shutdown() override;

protected: // functions exposed for testing
  /**
   * Returns the next file from storage that should be read from. Prioritizes newest 
   * files first. 
   * @return a path to the file to be read
   * @throws std::runtime_error If there are no files available to read
   */
  std::string getFileToRead();

  /**
   * Returns the active write file. Getter used for testing.
   * @return the file the system is actively writing to
   */
  std::string getActiveWriteFile() {
    return active_write_file_;
  }

private:
  /**
   * Validates options that were passed to the constructor of this class. Also
   * sanitizes any paths or slightly incorrect options.
   */
  void validateOptions();

  /**
   * Initializes the folder that offline data will be saved to, creating it if 
   * it doesn't already exist.
   */ 
  void initializeStorage();

  /**
   * Creates the TokenStore and runs its initialize method
   */
  void initializeTokenStore();

  /**
   * Finds any offline storage files on disk that are awaiting upload.
   */
  void discoverStoredFiles();

  /**
   * Delete a file at this path. Used when all data from a file has been uploaded 
   * successfully or when the offline file storage has reached storage_limit_in_kb
   * @param file_path path to the file to be deleted
   */
  void deleteFile(const std::string &file_path);

  /**
   * Checks if the active_write_file_ size is equal to or greater than 
   * maximum_file_size_in_kb in size. If it is then it calls rotateWriteFile. 
   * @param new_data_size the size of the active_write_file in bytes
   */
  void checkIfWriteFileShouldRotate(const uintmax_t &new_data_size);

  /** 
   * Moves the active_write_file into stored_files list and creates a new file
   * setting the active_write_file_ to that new file. 
   */
  void rotateWriteFile();

  /**
   * Checks if all the space taken up by offline storage exceeds the setting 
   * storage_limit_in_kb. If it has reached that amount it deletes the oldest 
   * file from disk.  
   * @param new_data_size the size of all files on disk in bytes
   */
  void checkIfStorageLimitHasBeenReached(const uintmax_t &new_data_size);

  /**
   * Deletes the oldest file in offline storage. Cannot delete the active_write_file_
   */
  void deleteOldestFile();

  /**
   * Adds a file path to the stored_files_ list 
   * @param file_path the path to the file
   */
  void addFilePathToStorage(const std::experimental::filesystem::path &file_path);

  /**
   * Stored files to read from in order from most recent to oldest.
   */
  std::list<std::string> stored_files_;

  /**
   * Disk space used by all stored files. Does not include active_write_file_size_.
   */
  std::atomic<size_t> stored_files_size_{};

  /**
   * Path to the file we're currently writing to. This file cannot be read from or deleted. 
   */
  std::string active_write_file_;

  /** 
   * The size of the active_write_file. Stored here to minimize disk reads. 
   */
  std::atomic<size_t> active_write_file_size_{};

  /** 
   * A lock on the active write file, to ensure that when it's rotated nothing is writing 
   * to it at the same time. 
   */
  std::mutex active_write_file_mutex_;

  /**
   * The file we are currently reading from. The FileManagerStrategy only reads from one file 
   * at a time.
   */
  std::string active_read_file_;

  /**
   * An IOStream for the active_read_file. Kept open so that it's easy to move to the next 
   * point in the file with each read instead of continually opening a new stream each time.
   */
  std::unique_ptr<std::ifstream> active_read_file_stream_;

  /**
   * A lock on the active read file, to ensure that it does not get deleted while it is being
   * read from. 
   */
  std::mutex active_read_file_mutex_;

  /**
   * Options for how and where to store files, and maximum file sizes.
   */
  FileManagerStrategyOptions options_;

  /**
   * Stores which tokens to read from.
   */
  std::unique_ptr<TokenStore> token_store_;
};

}  // namespace FileManagement
}  // namespace Aws
