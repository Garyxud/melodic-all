/*
 *  Author(s):
 *  - Joël Lamotte <jlamotte@aldebaran.com>
 *
 *  Copyright (c) 2014 Aldebaran. All rights reserved.
 */

#include <gtest/gtest.h>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem.hpp>
#include <atomic>

#include <qicore/file.hpp>
#include <qi/path.hpp>
#include <qi/application.hpp>
#include <qi/type/dynamicobjectbuilder.hpp>
#include <testsession/testsessionpair.hpp>
#include <testsession/testsession.hpp>

qiLogCategory("qimessaging.testFile");

namespace
{
struct TemporaryDir
{
  const qi::Path PATH;
  TemporaryDir()
    : PATH(qi::os::mktmpdir("qiCoreTestFile"))
  {
  }
  ~TemporaryDir()
  {
    boost::system::error_code err;
    const auto path = boost::filesystem::system_complete(PATH);
    boost::filesystem::remove_all(path, err);
    if (err)
    {
      qiLogError() << "Failed to remove temporary directory '" << PATH << "' : " << err.message();
    }
  }
} const TEMPORARY_DIR;

const qi::Path SMALL_TEST_FILE_PATH{TEMPORARY_DIR.PATH / "\xED\x95\x9C/testfile.data"};
qi::Path BIG_TEST_FILE_PATH;

const std::string TESTFILE_CONTENT = "abcdefghijklmnopqrstuvwxyz";
const std::streamoff TESTFILE_PARTIAL_BEGIN_POSITION = 23;
const std::streamoff TESTFILE_PARTIAL_SIZE = 3;

const std::streamoff TESTFILE_MIDDLE_BEGIN_POSITION = TESTFILE_CONTENT.size() / 3;
const std::streamoff TESTFILE_MIDDLE_SIZE = TESTFILE_CONTENT.size() / 3;

void makeSmallTestFile()
{
  boost::filesystem::remove(SMALL_TEST_FILE_PATH);
  boost::filesystem::create_directories(SMALL_TEST_FILE_PATH.parent());
  boost::filesystem::ofstream fileOutput(SMALL_TEST_FILE_PATH, std::ios::out | std::ios::binary);
  assert(fileOutput.is_open());
  fileOutput << TESTFILE_CONTENT;
  fileOutput.flush();
}

void checkIsTestFileContent(const qi::Buffer& buffer, std::streamoff beginOffset, std::streamsize bytesCount)
{
  EXPECT_EQ(static_cast<std::streamsize>(buffer.size()), bytesCount);
  for (size_t idx = 0; idx < buffer.totalSize(); ++idx)
  {
    EXPECT_EQ(static_cast<const char*>(buffer.data())[idx], TESTFILE_CONTENT[beginOffset + idx]);
  }
}

void checkIsTestFileMiddleContent(const qi::Buffer& buffer)
{
  return checkIsTestFileContent(buffer, TESTFILE_MIDDLE_BEGIN_POSITION, TESTFILE_MIDDLE_SIZE);
}

void checkIsTestFilePartialContent(const qi::Buffer& buffer)
{
  return checkIsTestFileContent(buffer, TESTFILE_PARTIAL_BEGIN_POSITION, TESTFILE_PARTIAL_SIZE);
}

void checkIsTestFileContent(const qi::Buffer& buffer)
{
  for (size_t idx = 0; idx < buffer.totalSize(); ++idx)
  {
    EXPECT_EQ(static_cast<const char*>(buffer.data())[idx], TESTFILE_CONTENT[idx]);
  }
}

void checkIsTestFileContent(qi::File& file)
{
  ASSERT_TRUE(file.isOpen());
  EXPECT_EQ(static_cast<std::streamsize>(TESTFILE_CONTENT.size()), file.size());
  file.seek(0);
  const qi::Buffer allFileData = file.read(file.size());
  EXPECT_EQ(TESTFILE_CONTENT.size(), allFileData.totalSize());
  checkIsTestFileContent(allFileData);
}

// Compare contents of the files and report any difference explicitly
void checkSameFilesContent(qi::File& leftFile, qi::File& rightFile)
{
  ASSERT_TRUE(leftFile.isOpen());
  ASSERT_TRUE(rightFile.isOpen());
  ASSERT_EQ(leftFile.size(), rightFile.size());
  static const std::streamsize BYTES_STEP = 1024 * 64;

  for (std::streamoff byteOffset = 0; byteOffset < rightFile.size(); byteOffset += BYTES_STEP)
  {
    qi::Buffer leftBytes = leftFile.read(byteOffset, BYTES_STEP);
    qi::Buffer rightBytes = rightFile.read(byteOffset, BYTES_STEP);
    EXPECT_GE(BYTES_STEP, leftBytes.totalSize());
    EXPECT_GE(BYTES_STEP, rightBytes.totalSize());
    EXPECT_EQ(leftBytes.totalSize(), rightBytes.totalSize());

    ASSERT_TRUE(std::equal(static_cast<char*>(leftBytes.data()),
                           static_cast<char*>(leftBytes.data()) + leftBytes.totalSize(),
                           static_cast<char*>(rightBytes.data())));
  }
}
}

TEST(TestFile, cannotReadUnknownFile)
{
  EXPECT_THROW(
      {
        qi::FilePtr file = qi::openLocalFile("file/that/doesnt/exists.atall");
      },
      std::runtime_error);
}

TEST(TestFile, cannotReadClosedFile)
{
  qi::FilePtr file = qi::openLocalFile(SMALL_TEST_FILE_PATH);
  EXPECT_TRUE(file->isOpen());

  file->close();
  EXPECT_FALSE(file->isOpen());
  EXPECT_EQ(0u, file->size());
  EXPECT_THROW(
      {
        file->read(42);
      },
      std::runtime_error);
  EXPECT_THROW(
      {
        file->seek(42);
      },
      std::runtime_error);
}

TEST(TestFile, readLocalFile)
{
  qi::FilePtr testFile = qi::openLocalFile(SMALL_TEST_FILE_PATH);
  EXPECT_TRUE(testFile->isOpen());
  EXPECT_EQ(std::streamsize(TESTFILE_CONTENT.size()), testFile->size());

  static const size_t COUNT_BYTES_TO_READ = 20;

  qi::Buffer buffer = testFile->read(COUNT_BYTES_TO_READ);
  ASSERT_EQ(COUNT_BYTES_TO_READ, buffer.totalSize());
  checkIsTestFileContent(buffer);
}

TEST(TestFile, cannotReadPastEnd)
{
  qi::FilePtr testFile = qi::openLocalFile(SMALL_TEST_FILE_PATH);
  EXPECT_TRUE(testFile->isOpen());
  EXPECT_EQ(std::streamsize(TESTFILE_CONTENT.size()), testFile->size());

  testFile->read(testFile->size());
  qi::Buffer buffer = testFile->read(1);
  EXPECT_EQ(0u, buffer.totalSize());
}

TEST(TestFile, localCopy)
{
  qi::FilePtr testFile = qi::openLocalFile(SMALL_TEST_FILE_PATH);
  EXPECT_TRUE(testFile->isOpen());
  EXPECT_EQ(std::streamsize(TESTFILE_CONTENT.size()), testFile->size());

  static const qi::Path LOCAL_COPY_PATH("if_this_is_not_removed_test_file_failed.txt");
  boost::filesystem::remove(LOCAL_COPY_PATH);
  {
    copyToLocal(testFile, LOCAL_COPY_PATH);

    qi::FilePtr copiedFile = qi::openLocalFile(SMALL_TEST_FILE_PATH);
    EXPECT_TRUE(copiedFile->isOpen());
    checkSameFilesContent(*testFile, *copiedFile);
  }
  boost::filesystem::remove(LOCAL_COPY_PATH);
}

namespace
{
qi::FilePtr getTestFile(const qi::Path& filePath)
{
  qi::FilePtr testFile = qi::openLocalFile(filePath);
  EXPECT_TRUE(testFile->isOpen());
  return testFile;
}

std::atomic<bool> printProgressHaveBeenCalled(false);
void printTranferProgress(double progress)
{
  printProgressHaveBeenCalled = true;
  qiLogInfo() << "#### File Transfer Progress = " << (progress * 100.0) << "%";
}

class Test_ReadRemoteFile : public ::testing::Test
{
public:
  void SetUp()
  {
    qi::DynamicObjectBuilder objectBuilder;
    objectBuilder.advertiseMethod("getTestFile", &getTestFile);
    service = objectBuilder.object();

    qi::SessionPtr serverSession = sessionPair.server();
    serverSession->registerService("service", service);
  }

  qi::FilePtr clientAcquireTestFile(const qi::Path& path)
  {
    qi::AnyObject service = sessionPair.client()->service("service");
    qi::FilePtr testFile = service.call<qi::FilePtr>("getTestFile", path);
    EXPECT_TRUE(testFile->isRemote());
    return testFile;
  }

private:
  TestSessionPair sessionPair;
  qi::AnyObject service;
};
}

TEST_F(Test_ReadRemoteFile, isLocalOrRemote)
{
  {
    qi::FilePtr testFile = openLocalFile(SMALL_TEST_FILE_PATH);
    EXPECT_FALSE(testFile->isRemote());
  }
  {
    qi::FilePtr testFile = clientAcquireTestFile(SMALL_TEST_FILE_PATH);
    EXPECT_TRUE(testFile->isRemote());
  }
}

TEST_F(Test_ReadRemoteFile, someReading)
{
  qi::FilePtr testFile = clientAcquireTestFile(SMALL_TEST_FILE_PATH);

  static const size_t COUNT_BYTES_TO_READ = 20;

  qi::Buffer buffer = testFile->read(COUNT_BYTES_TO_READ);
  ASSERT_EQ(COUNT_BYTES_TO_READ, buffer.totalSize());
  checkIsTestFileContent(buffer);
}

TEST_F(Test_ReadRemoteFile, readAll)
{
  qi::FilePtr testFile = clientAcquireTestFile(SMALL_TEST_FILE_PATH);

  static const size_t COUNT_BYTES_TO_READ_PER_CYCLE = 10;

  qi::Buffer buffer;
  qi::Buffer cycleBuffer;
  while (true)
  {
    cycleBuffer = testFile->read(COUNT_BYTES_TO_READ_PER_CYCLE);
    buffer.write(cycleBuffer.data(), cycleBuffer.totalSize());
    if (cycleBuffer.totalSize() < COUNT_BYTES_TO_READ_PER_CYCLE)
      break;
  }

  EXPECT_EQ(TESTFILE_CONTENT.size(), buffer.totalSize());
  checkIsTestFileContent(buffer);
}

TEST_F(Test_ReadRemoteFile, readAllOnce)
{
  qi::FilePtr testFile = clientAcquireTestFile(SMALL_TEST_FILE_PATH);

  const std::streamsize fileSize = testFile->size();
  EXPECT_EQ(std::streamsize(TESTFILE_CONTENT.size()), fileSize);

  qi::Buffer buffer = testFile->read(fileSize);
  EXPECT_EQ(TESTFILE_CONTENT.size(), buffer.totalSize());
  checkIsTestFileContent(buffer);
}

TEST_F(Test_ReadRemoteFile, filetransfert)
{
  static const qi::Path LOCAL_PATH_TO_RECEIVE_FILE_IN = TEMPORARY_DIR.PATH / "file.data";
  boost::filesystem::remove(LOCAL_PATH_TO_RECEIVE_FILE_IN);

  {
    qi::FilePtr testFile = clientAcquireTestFile(SMALL_TEST_FILE_PATH);

    copyToLocal(testFile, LOCAL_PATH_TO_RECEIVE_FILE_IN);
  }
  {
    qi::FilePtr localFileCopy = qi::openLocalFile(LOCAL_PATH_TO_RECEIVE_FILE_IN);
    EXPECT_TRUE(localFileCopy->isOpen());
    checkIsTestFileContent(*localFileCopy);
  }

  boost::filesystem::remove(LOCAL_PATH_TO_RECEIVE_FILE_IN);
}

TEST_F(Test_ReadRemoteFile, readInTheMiddle)
{
  qi::FilePtr testFile = clientAcquireTestFile(SMALL_TEST_FILE_PATH);

  qi::Buffer bufferPartial = testFile->read(TESTFILE_PARTIAL_BEGIN_POSITION, TESTFILE_PARTIAL_SIZE);
  checkIsTestFilePartialContent(bufferPartial);

  qi::Buffer bufferMiddle = testFile->read(TESTFILE_MIDDLE_BEGIN_POSITION, TESTFILE_MIDDLE_SIZE);
  checkIsTestFileMiddleContent(bufferMiddle);
}

TEST_F(Test_ReadRemoteFile, bigFiletransfert)
{
  static const qi::Path LOCAL_PATH_TO_RECEIVE_FILE_IN = TEMPORARY_DIR.PATH / "bigfile.data";
  boost::filesystem::remove(LOCAL_PATH_TO_RECEIVE_FILE_IN);

  {
    qi::FilePtr testFile = clientAcquireTestFile(BIG_TEST_FILE_PATH);

    qi::FileCopyToLocal fileCopy{testFile, LOCAL_PATH_TO_RECEIVE_FILE_IN};
    fileCopy.notifier()->progress.connect(&printTranferProgress);
    fileCopy.start().wait();

    EXPECT_TRUE(printProgressHaveBeenCalled.load());
  }
  {
    qi::FilePtr originalFile = qi::openLocalFile(BIG_TEST_FILE_PATH);
    qi::FilePtr localFileCopy = qi::openLocalFile(LOCAL_PATH_TO_RECEIVE_FILE_IN);
    checkSameFilesContent(*originalFile, *localFileCopy);
  }

  boost::filesystem::remove(LOCAL_PATH_TO_RECEIVE_FILE_IN);
}

TEST_F(Test_ReadRemoteFile, cancelFileTransfer)
{
  static const qi::Path LOCAL_PATH_TO_RECEIVE_FILE_IN = TEMPORARY_DIR.PATH / "bigfile.data";
  boost::filesystem::remove(LOCAL_PATH_TO_RECEIVE_FILE_IN);

  {
    qi::FilePtr testFile = clientAcquireTestFile(BIG_TEST_FILE_PATH);

    qi::FileCopyToLocal fileOp{ testFile, LOCAL_PATH_TO_RECEIVE_FILE_IN };
    auto fileOpNotifier = fileOp.notifier();
    fileOpNotifier->status.connect([&](qi::ProgressNotifier::Status status){
      if (status == qi::ProgressNotifier::Status_Running)
      {
        fileOpNotifier->waitForFinished().cancel();
      }
    });
    qi::Future<void> copyOpFt = fileOp.start();
    copyOpFt.wait();
    EXPECT_TRUE(copyOpFt.isCanceled());
  }

  EXPECT_FALSE(boost::filesystem::exists(LOCAL_PATH_TO_RECEIVE_FILE_IN));
  boost::filesystem::remove(LOCAL_PATH_TO_RECEIVE_FILE_IN);
}

int main(int argc, char** argv)
{
  ::TestMode::forceTestMode(TestMode::Mode_SD);
  ::testing::InitGoogleTest(&argc, argv);
  qi::Application app(argc, argv);
  BIG_TEST_FILE_PATH = qi::path::findLib("qi");
  makeSmallTestFile();
  const int result = RUN_ALL_TESTS();
  return result;
}
