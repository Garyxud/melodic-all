/*
**  Copyright (C) 2012 Aldebaran Robotics
**  See COPYING for the license
*/

#include <qicore/file.hpp>
#include <qi/anymodule.hpp>

namespace qi
{
  template<class FileOpType, class... Args >
  auto launchStandalone(Args&&... args)->decltype(std::declval<FileOpType>().start())
  {
    FileOpType fileOp{ std::forward<Args>(args)... };
    fileOp.start();
    return fileOp.detach();
  }

  FutureSync<void> copyToLocal(FilePtr file, Path localPath)
  {
    return launchStandalone<FileCopyToLocal>(std::move(file), std::move(localPath));
  }

  FileOperationPtr prepareCopyToLocal(FilePtr file, Path localPath)
  {
    return boost::make_shared<FileCopyToLocal>(std::move(file), std::move(localPath));
  }

  void _qiregisterFileOperation()
  {
    ::qi::ObjectTypeBuilder<FileOperation> builder;
    QI_OBJECT_BUILDER_ADVERTISE(builder, FileOperation, start);
    QI_OBJECT_BUILDER_ADVERTISE(builder, FileOperation, detach);
    QI_OBJECT_BUILDER_ADVERTISE(builder, FileOperation, notifier);
    QI_OBJECT_BUILDER_ADVERTISE(builder, FileOperation, isValid);

    builder.registerType();
  }

  void registerFileOperations(qi::ModuleBuilder& mb)
  {
    mb.advertiseMethod("copyToLocal", &copyToLocal);
    mb.advertiseMethod("FileCopyToLocal", &prepareCopyToLocal);
  }

}
