/*
 *  Author(s):
 *  - Joël Lamotte <jlamotte@aldebaran.com>
 *
 *  Copyright (c) 2014 Aldebaran. All rights reserved.
 */
#include <iostream>
#include <string>
#include <boost/filesystem.hpp>

#include <qi/applicationsession.hpp>
#include <qicore/file.hpp>
#include <qi/session.hpp>

qiLogCategory("test.qi.sendRobotIcon");

void printTranferProgress(double progress)
{
  qiLogInfo() << ">>>> File Transfer Progress = " << (progress * 100.0) << "%";
}

int main(int argc, char** argv)
{
  qi::ApplicationSession app(argc, argv);
  app.startSession();
  if (argc != 2)
  {
    qiLogError() << "This test takes 2 arguments!";
    return EXIT_FAILURE;
  }

  qi::SessionPtr session = app.session();
  if (!session->isConnected())
  {
    qiLogError() << "Failed to connect to the remote naoqi!";
    return EXIT_FAILURE;
  }

  const std::string image_file_path = argv[1];
  qi::FilePtr imageFile = qi::openLocalFile(qi::Path(image_file_path));

  qi::ProgressNotifierPtr progressNotifier = imageFile->operationProgress();
  progressNotifier->progress.connect(&printTranferProgress);

  qi::AnyObject alsystem = session->service("ALSystem");
  if (!alsystem)
  {
    qiLogError() << "Failed access ALSystem service!";
    return EXIT_FAILURE;
  }

  qiLogInfo() << "setRobotIcon..." << std::endl;
  alsystem.async<void>("setRobotIcon", imageFile).wait();
  qiLogInfo() << "setRobotIcon - DONE" << std::endl;
  app.run();
  return EXIT_SUCCESS;
}
