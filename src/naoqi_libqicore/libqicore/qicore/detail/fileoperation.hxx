#pragma once
#ifndef _QICORE_FILEOPERATION_HPP_
#define _QICORE_FILEOPERATION_HPP_

#include <iostream>
#include <memory>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/operations.hpp>
#include <qi/detail/warn_push_ignore_deprecated.hpp>

namespace qi
{
  /** Base type for file operation exposing information about its progress state.
      Exposes a ProgressNotifier, associated to the operation.

      @includename{qicore/file.hpp}
  **/
  class FileOperation
  {
  public:
    /** Destructor.
        Cancel the operations's task if is still running and this object is valid.
    **/
    virtual ~FileOperation()
    {
      auto task = std::move(_task);
      if (task)
      {
        task->promise.future().cancel();
      }
    }

    // Move only
    FileOperation(const FileOperation&) = delete;
    FileOperation& operator=(const FileOperation&) = delete;

    /** Move construction.
        @param other  Object that will be moved-from. Will be in invalid state after this call,
                      until being assigned to a vallid state.
    */
    FileOperation(FileOperation&& other) // TODO VS2015 C++11: = default;
      : _task(std::move(other._task))
    {}

    /** Move assignation.
        @param other  Object that will be moved-from. Will be in invalid state after this call,
                      until being assigned to a vallid state.
    */
    FileOperation& operator=(FileOperation&& other) // TODO VS2015 C++11: = default;
    {
      _task = std::move(other._task);
      return *this;
    }

    /** Starts the operation's task.
        This function must be called only once.
        Throws a std::runtime_error if start() has already been called before at least once
        or if this object is in an invalid state.

        @return A future corresponding to the end of the operation.
    **/
    qi::Future<void> start()
    {
      if (!_task)
      {
        throw std::runtime_error{ "Tried to start an invalid FileOperation" };
      }

      if (_task->isLaunched.swap(true))
      {
        throw std::runtime_error{ "Called FileOperation::start() more than once!" };
      }

      return _task->run();
    }

    /** Detach the running operation from this object.
        Useful to dissociate the on-going operation from the lifetime of the object,
        in order to allow its continuation after object destruction.
        The object destructor will cancel any still running operation if not dissociated beforehand.

        Once called, this object will be in invalid state.
        The task must have been started before calling this function,
        otherwise a std::runtime_exception will be thrown.

        @return A future corresponding to the end of the operation.
    **/
    qi::Future<void> detach()
    {
      boost::shared_ptr<Task> sharedTask = std::move(_task);

      if (!sharedTask)
      {
        throw std::runtime_error("Called FileOperation::detach() but no task is owned!");
      }

      if (!sharedTask->isLaunched._value)
      {
        throw std::runtime_error("Called FileOperation::detach() but task was not started!");
      }

      auto future = sharedTask->promise.future();
      future.connect([sharedTask](const Future<void>&){}); // keep the task alive until it ends
      return future;
    }

    /// Call operator: calls start()
    auto operator()() -> decltype(start()) { return start(); }

    /** @returns A progress notifier associated to the operation if the operation's task is owned
                 and this object is valid, null otherwise.
    **/
    ProgressNotifierPtr notifier() const { return _task ? _task->localNotifier : ProgressNotifierPtr{}; }

    /** @returns True if this object is in a valid state, false otherwise.
                 In an invalid state, all of this object's member function calls will result in exception thrown
                 except validity checks functions and move-assignation.
                 An invalid object can be re-assigned to a valid state.
    **/
    bool isValid() const { return _task ? true : false; }

    /// @returns True if this object owns the operation's task, false otherwise.
    explicit operator bool() const { return isValid(); }

  protected:
    struct Task
      : public boost::enable_shared_from_this<Task>
    {
      Task(FilePtr file)
        : sourceFile{ std::move(file) }
        , fileSize{ sourceFile->size() }
        , promise{ PromiseNoop<void> }
        , localNotifier{ createProgressNotifier(promise.future()) }
        , remoteNotifier{ sourceFile->operationProgress() }
        , isRemoteDeprecated(sourceFile.metaObject().findMethod("read").empty())
      {
      }

      virtual ~Task() = default;

      qi::Future<void> run()
      {
        localNotifier->reset();
        isRemoteDeprecated ? remoteNotifier->_reset() : remoteNotifier->reset();
        localNotifier->notifyRunning();
        isRemoteDeprecated ? remoteNotifier->_notifyRunning() : remoteNotifier->notifyRunning();
        start();
        return promise.future();
      }

      void finish()
      {
        promise.setValue(0);
        localNotifier->notifyFinished();
        isRemoteDeprecated ? remoteNotifier->_notifyFinished() : remoteNotifier->notifyFinished();
      }

      void fail(const std::string& errorMessage)
      {
        promise.setError(errorMessage);
        localNotifier->notifyFailed();
        isRemoteDeprecated ? remoteNotifier->_notifyFailed() : remoteNotifier->notifyFailed();
      }

      void cancel()
      {
        promise.setCanceled();
        localNotifier->notifyCanceled();
        isRemoteDeprecated ? remoteNotifier->_notifyCanceled() : remoteNotifier->notifyCanceled();
      }

      void notifyProgressed(double newProgress)
      {
        localNotifier->notifyProgressed(newProgress);
        isRemoteDeprecated ? remoteNotifier->_notifyProgressed(newProgress) : remoteNotifier->notifyProgressed(newProgress);
      }

      virtual void start() = 0;

      qi::Atomic<bool> isLaunched{ false };
      const FilePtr sourceFile;
      const std::streamsize fileSize;
      Promise<void> promise;
      const ProgressNotifierPtr localNotifier;
      const ProgressNotifierPtr remoteNotifier;
      const bool isRemoteDeprecated;
    };

    using TaskPtr = boost::shared_ptr<Task>;

    explicit FileOperation(TaskPtr task)
      : _task{ std::move(task) }
    {
      if (!_task)
        throw std::runtime_error("FileOperation requires a non-null task on constrution.");
    }

  private:
    TaskPtr _task;
  };

  /////////////////////////////////////////////////////////////////////////////////

  /// Pointer to a file operation with sharing semantic.
  using FileOperationPtr = Object<FileOperation>;

  /** Copies a potentially remote file to the local file system. */
  class FileCopyToLocal
    : public FileOperation
  {
  public:
    /** Constructor.
        @param file        Access to a potentially remote file to copy to the local file system.
        @param localPath   Local file system location where the specified file will be copied.
                           No file or directory should be located at this path otherwise
                           the operation will fail.
    **/
    FileCopyToLocal(qi::FilePtr file, qi::Path localPath)
      : FileOperation(boost::make_shared<Task>(std::move(file), std::move(localPath)))
    {
    }

  private:
    class Task
      : public FileOperation::Task
    {
    public:
      Task(FilePtr sourceFile, qi::Path localFilePath)
        : FileOperation::Task(std::move(sourceFile))
        , localPath(std::move(localFilePath))
      {
      }

      void start() override
      {
        if (makeLocalFile())
        {
          fetchData();
        }
      }

      void stop()
      {
        localFile.close();
        finish();
      }

      bool makeLocalFile()
      {
        if (localPath.isEmpty()) {
          return true;
        }

        localFile.open(localPath.bfsPath(), std::ios::out | std::ios::binary);
        if (!localFile.is_open())
        {
          fail("Failed to create local file copy.");
          return false;
        }
        return true;
      }

      void write(Buffer buffer)
      {
        if (localFile.is_open())
          localFile.write(static_cast<const char*>(buffer.data()), buffer.totalSize());
        else
          std::cout.write(static_cast<const char*>(buffer.data()), buffer.totalSize());
        bytesWritten += buffer.totalSize();
        assert(fileSize >= bytesWritten);

        const double progress = static_cast<double>(bytesWritten) / static_cast<double>(fileSize);
        notifyProgressed(progress);
      }

      void fetchData()
      {
        static const size_t ARBITRARY_BYTES_TO_READ_PER_CYCLE = 512 * 1024;
        auto myself = shared_from_this();

        const auto readFuncName = isRemoteDeprecated ? "_read" : "read";

        sourceFile.async<Buffer>(readFuncName, bytesWritten, ARBITRARY_BYTES_TO_READ_PER_CYCLE)
          .connect([this, myself](Future<Buffer> futureBuffer)
        {
          if (futureBuffer.hasError())
          {
            fail(futureBuffer.error());
            clearLocalFile();
            return;
          }
          if (promise.isCancelRequested())
          {
            clearLocalFile();
            cancel();
            return;
          }

          write(futureBuffer.value());
          if (bytesWritten < fileSize)
            fetchData();
          else
            stop();
        }
        );
      }

      void clearLocalFile()
      {
        if (localFile.is_open())
          localFile.close();
        boost::filesystem::remove(localPath);
      }

      boost::filesystem::ofstream localFile;
      std::streamsize bytesWritten = 0;
      const qi::Path localPath;
    };

  };

  /** Copy an open local or remote file to a local file system location.
  *   @param file         Source file to copy.
  *   @param localPath    Local file system location where the specified file will be copied.
  *                       No file or directory should be located at this path otherwise
  *                       the operation will fail.
  *   @return A synchronous future associated with the operation.
  **/
  QICORE_API FutureSync<void> copyToLocal(FilePtr file, Path localPath);
}

#include <qi/detail/warn_pop_ignore_deprecated.hpp>
#endif
