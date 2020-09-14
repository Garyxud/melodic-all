/*
**  Copyright (C) 2012 Aldebaran Robotics
**  See COPYING for the license
*/

#include <qicore/file.hpp>

#include <boost/filesystem/fstream.hpp>
#include <algorithm>
#include <vector>

#include <qi/anymodule.hpp>

// FIXME: Remove once deprecated method are removed
#include <qi/detail/warn_push_ignore_deprecated.hpp>

namespace qi
{
class FileImpl : public File
{
public:
  explicit FileImpl(const Path& localFilePath)
  {
    if (!localFilePath.exists())
    {
      std::stringstream message;
      message << "File not found on qi::File open: " << localFilePath.str();
      throw std::runtime_error(message.str());
    }

    _progressNotifier = createProgressNotifier();

    _fileStream.open(localFilePath, std::ios::in | std::ios::binary);
    if (_fileStream.is_open())
    {
      _fileStream.seekg(0, _fileStream.end);
      _size = _fileStream.tellg();
      _fileStream.seekg(0, _fileStream.beg);
      assert(_fileStream.tellg() == std::streamoff(0));
    }
  }

  ~FileImpl() = default;

  Buffer read(std::streamoff beginOffset, std::streamsize countBytesToRead) override
  {
    if (seek(beginOffset))
      return read(countBytesToRead);
    else
      return {};
  }

  Buffer read(std::streamsize countBytesToRead) override
  {
    requireOpenFile();
    if (countBytesToRead > MAX_READ_SIZE)
      throw std::runtime_error("Tried to read too much data at once.");

    Buffer output;

    assert(_fileStream.is_open());
    const std::streamoff initialCursorPos = static_cast<std::streamoff>(_fileStream.tellg());
    const std::streamoff targetEnd = std::min(initialCursorPos + countBytesToRead, static_cast<std::streamoff>(_size));
    const std::streamsize distanceToTargetEnd = targetEnd - initialCursorPos;
    const std::streamsize byteCountToRead = std::min(static_cast<std::streamsize>(MAX_READ_SIZE), distanceToTargetEnd);
    assert(byteCountToRead <= MAX_READ_SIZE);

    _readBuffer.resize(static_cast<size_t>(byteCountToRead), '\0');
    _fileStream.read(_readBuffer.data(), byteCountToRead);
    const std::streamsize bytesRead = _fileStream.gcount();
    assert(bytesRead <= byteCountToRead);
    output.write(_readBuffer.data(), static_cast<size_t>(bytesRead));

    return output;
  }

  bool seek(std::streamoff offsetFromBegin) override
  {
    requireOpenFile();

    if (offsetFromBegin >= _size)
      return false;

    _fileStream.seekg(offsetFromBegin);
    return true;
  }

  void close() override
  {
    _fileStream.close();
    _size = 0;
  }

  std::streamsize size() const override
  {
    return _size;
  }

  bool isOpen() const override
  {
    return _fileStream.is_open();
  }

  bool isRemote() const override
  {
    return false;
  }

  ProgressNotifierPtr operationProgress() const override
  {
    return _progressNotifier;
  }

  // Deprecated members:
  Buffer _read(std::streamoff beginOffset, std::streamsize countBytesToRead) override
  {
    return read(beginOffset, countBytesToRead);
  }

  Buffer _read(std::streamsize countBytesToRead) override
  {
    return read(countBytesToRead);
  }

  bool _seek(std::streamoff offsetFromBegin) override
  {
    return seek(offsetFromBegin);
  }

  void _close() override
  {
    return close();
  }

private:
  boost::filesystem::ifstream _fileStream;
  std::vector<char> _readBuffer;
  std::streamsize _size;
  ProgressNotifierPtr _progressNotifier;

  void requireOpenFile()
  {
    if (!_fileStream.is_open())
      throw std::runtime_error("Trying to manipulate a closed file access.");
  }
};

void _qiregisterFile()
{
  ::qi::ObjectTypeBuilder<File> builder;

  QI_OBJECT_BUILDER_ADVERTISE_OVERLOAD(builder, File, read, Buffer,(std::streamoff, std::streamsize));
  QI_OBJECT_BUILDER_ADVERTISE_OVERLOAD(builder, File, read, Buffer, (std::streamsize));
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, seek);
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, close);
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, size);
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, isOpen);
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, isRemote);
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, operationProgress);

  // Deprecated members:
  QI_OBJECT_BUILDER_ADVERTISE_OVERLOAD(builder, File, _read, Buffer, (std::streamoff, std::streamsize));
  QI_OBJECT_BUILDER_ADVERTISE_OVERLOAD(builder, File, _read, Buffer, (std::streamsize));
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, _seek);
  QI_OBJECT_BUILDER_ADVERTISE(builder, File, _close);

  builder.registerType();

  {
    qi::detail::ForceProxyInclusion<File>().dummyCall();
    qi::registerType(typeid(FileImpl), qi::typeOf<File>());
    FileImpl* ptr = static_cast<FileImpl*>(reinterpret_cast<void*>(0x10000));
    File* pptr = ptr;
    intptr_t offset = reinterpret_cast<intptr_t>(pptr)-reinterpret_cast<intptr_t>(ptr);
    if (offset)
    {
      qiLogError("qitype.register") << "non-zero offset for implementation FileImpl of File,"
        "call will fail at runtime";
      throw std::runtime_error("non-zero offset between implementation and interface");
    }
  }

}

FilePtr openLocalFile(const qi::Path& localPath)
{
  return boost::make_shared<FileImpl>(localPath);
}

void registerFileCreation(qi::ModuleBuilder& mb)
{
  mb.advertiseMethod("openLocalFile", &openLocalFile);
}
}

// FIXME: Remove once deprecated method are removed
#include <qi/detail/warn_pop_ignore_deprecated.hpp>
