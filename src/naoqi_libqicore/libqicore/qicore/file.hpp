#pragma once
#ifndef _QICORE_FILE_HPP_
#define _QICORE_FILE_HPP_
/*
**  Copyright (C) 2013 Aldebaran Robotics
**  See COPYING for the license
*/

#include <iosfwd>
#include <cassert>

#include <qicore/api.hpp>
#include <qi/anyobject.hpp>
#include <qi/path.hpp>
#include <qi/future.hpp>
#include <qi/buffer.hpp>
#include <qi/property.hpp>

#include <boost/shared_ptr.hpp>

namespace qi
{
/** Provide information about the state of a potentially long remote or async operation.
*   @includename{qicore/file.hpp}
**/
class QICORE_API ProgressNotifier
{
protected:
  ProgressNotifier() = default;

public:
  virtual ~ProgressNotifier() = default;

  /** Describe the status of an associated operation. */
  enum Status
  {
    Status_Idle,      ///< The operation has not start yet.
    Status_Running,   ///< The operation is currently running.
    Status_Finished,  ///< The operation finished successfully.
    Status_Failed,    ///< The operation has failed.
    Status_Canceled,  ///< The operation has been canceled by the user.
  };

  /** Current status of the operation associated to this notifier.
  *   @see ProgressNotifier::Status
  **/
  Property<Status> status;

  /** Progress state of the operation associated with this notifier.
  *   By default you can assume a normalized value ranging between 0.0
  *   (no work is done) to 1.0 (all work is done).
  *   The semantic of this value is defined by the operation implementation
  *   and could be different from the default but should then be documented.
  **/
  Property<double> progress;
  /** @returns true if the operation associated to this notifier has started
  *            and is neither finished nor canceled nor failed yet, false otherwise.
  **/
  virtual bool isRunning() const = 0;

  virtual Future<void> waitForFinished() = 0;

  ///////////////////////
  // The following operations are reserved for the implementation of the associated operations.

  /** Reset the status of the algorithm to the idle state with no progress.
      @remark This function is reserved to be used by the implementation of the associated operations.
  */
  virtual void reset() = 0;

  /** Notify the observers that the operation associated with this notifier is now running.
      @remark This function is reserved to be used by the implementation of the associated operations.
      */
  virtual void notifyRunning() = 0;

  /** Notify the observers that the operation has successfully ended.
      @remark This function is reserved to be used by the implementation of the associated operations.
      */
  virtual void notifyFinished() = 0;

  /** Notify the observers that the operation has been canceled by the user.
      @remark This function is reserved to be used by the implementation of the associated operations.
      */
  virtual void notifyCanceled() = 0;

  /** Notify the observers that the operation has failed.
      @remark This function is reserved to be used by the implementation of the associated operations.
      */
  virtual void notifyFailed() = 0;

  /** Notify the observers that the operation progressed.
      @remark This function is reserved to be used by the implementation of the associated operations.

      @param newProgress    New value representing the total progress of the operation.
                            By default, uses a range from 0.0 (no work has been done yet)
                            to 1.0 (all work is done). The operation implementation
                            is free to use another range if necessary but should clarify
                            the meaning of this value in its documentation.
  **/
  virtual void notifyProgressed(double newProgress) = 0;

  /**
   * @deprecated since 2.5
   **/
  QI_API_DEPRECATED_MSG(Use 'reset' instead) virtual void _reset() = 0;
  QI_API_DEPRECATED_MSG(Use 'notifyRunning' instead) virtual void _notifyRunning() = 0;
  QI_API_DEPRECATED_MSG(Use 'notifyFinished' instead) virtual void _notifyFinished() = 0;
  QI_API_DEPRECATED_MSG(Use 'notifyCanceled' instead) virtual void _notifyCanceled() = 0;
  QI_API_DEPRECATED_MSG(Use 'notifyFailed' instead) virtual void _notifyFailed() = 0;
  QI_API_DEPRECATED_MSG(Use 'notifyProgressed' instead) virtual void _notifyProgressed(double newProgress) = 0;
};

/// Pointer to a ProgressNotifier with shared/remote semantic.
using ProgressNotifierPtr = qi::Object<ProgressNotifier>;

/** Create and provide a remotely shareable ProgressNotifier object.
    @param operationFuture   Optional future of an operation to associate the notifier with.
    @return A progress notifier, associated to the operation of the future if provided.
*/
QICORE_API ProgressNotifierPtr createProgressNotifier(Future<void> operationFuture = {});

/** Provide access to the content of a local or remote file.
*   @includename{qicore/file.hpp}
*   @remark Should be obtained using openLocalFile()
*           or through a service API if the file is potentially remote.
**/
class QICORE_API File
{
protected:
  File() = default;

public:
  virtual ~File() = default;


  /** @return Total count of bytes contained in the file or 0 if the file is closed. */
  virtual std::streamsize size() const = 0;

  /** @return true if the file is currently open for reading, false otherwise. */
  virtual bool isOpen() const = 0;

  /** @return true if the file is located on a remote filesystem, false otherwise. */
  virtual bool isRemote() const = 0;

  /** Provide the progress notifier used by the operations manipulating this file.
  *   The notifier is associated with this file. Therefore, no concurrent operation should be
  *   used by this notifier object, as it is not safe to have concurrent operations
  *   running on the same file.
  **/
  virtual ProgressNotifierPtr operationProgress() const = 0;

  ///////////////////////
  // The following operations are reserved for the implementation of file processing algorithms.

  /// Maximum count of bytes that you can read by reading functions call.
  static const std::streamsize MAX_READ_SIZE = 1000000;

  /** Read a specified count of bytes starting from the current cursor position.
  *   @warning If you try to read more than _MAX_READ_SIZE bytes, this call will throw a std::runtime_error.
  *
  *   @param countBytesToRead       Count of bytes to read from the file, starting from the current position
  *                                 of the file cursor.
  *   @return A buffer of data read from the file, empty if there is no data in the specified byte range
  *           to read or if the file have been closed.
  *           If there is less data to read in the file than the required count,
  *           if we try reading past the end of the file for example,
  *           then the buffer will only contain the available data, nothing more.
  **/
  virtual Buffer read(std::streamsize countBytesToRead) = 0;

  /** Read a specified count of bytes starting from a specified byte position in the file.
  *   @warning If you try to read more than _MAX_READ_SIZE bytes, this call will throw a std::runtime_error.
  *
  *   @param beginOffset            Position in the file to start reading from.
  *   @param countBytesToRead       Count of bytes to read from the file starting at the current position
  *                                 of the file cursor.
  *   @return A buffer of data read from the file, empty if:
  *           - there is no data in the specified byte range to read
  *           - if the file have been closed;
  *           - if the start position is outside the available range of data in the file.
  *           If there is less data to read in the file than the required count,
  *           if we try reading past the end of the file for example,
  *           then the buffer will only contain the available data, nothing more
  **/
  virtual Buffer read(std::streamoff beginOffset, std::streamsize countBytesToRead) = 0;

  /** Move the read cursor to the specified position in the file.
  *   @param offsetFromBegin      New position of the read cursor in the file.
  *                               If it is out of the range of data in the file,
  *                               the cursor will not be changed at all.
  *   @return true if the position is in the range of data available in the file,
  *           false otherwise, in which case the cursor have not been changed.
  **/
  virtual bool seek(std::streamoff offsetFromBegin) = 0;

  /** Close the file.
  *   Once this function is called, calling most other operation will throw
  *   a std::runtime_error.
  *   The size(), isOpen() and isRemote() calls will return work as expected.
  **/
  virtual void close() = 0;

  /**
   * @deprecated since 2.5
   **/
  QI_API_DEPRECATED_MSG(Use 'read' instead) virtual Buffer _read(std::streamsize countBytesToRead) = 0;
  QI_API_DEPRECATED_MSG(Use 'read' instead) virtual Buffer _read(std::streamoff beginOffset, std::streamsize countBytesToRead) = 0;
  QI_API_DEPRECATED_MSG(Use 'seek' instead) virtual bool _seek(std::streamoff offsetFromBegin) = 0;
  QI_API_DEPRECATED_MSG(Use 'close' instead) virtual void _close() = 0;
};

/// Pointer to a file with shared/remote semantic.
using FilePtr = qi::Object<File>;

/** Open a local file located at the specified path and provide it for reading as a sharable file access.
*   @warning Throws a std::runtime_exception if the provided path is not an existing file path.
*
*   @param localPath              Path to a file on the local file system that can be open.
*   @return A shareable access to the opened local file.
**/
QICORE_API FilePtr openLocalFile(const qi::Path& localPath);

}

QI_TYPE_INTERFACE(File);
QI_TYPE_INTERFACE(ProgressNotifier);
QI_TYPE_ENUM(ProgressNotifier::Status);

#include <qicore/detail/fileoperation.hxx>

#endif // _QI_FILE_HPP_
