#include<iostream.h>
#include <yaml-cpp/yaml.h>

namespace carviz{

class  : 




}

  std::string config_file_;
  config_file_.assign(argv[1]);
  YAML::Node config = YAML::LoadFile(config_file_);
  std::string topic = config["Topic"].as<std::string>();
  std::string cameraNum = config["cameraNum"].as<std::string>();
  int fps = config["fps"].as<int>();
  std::string name = config["Name"].as<std::string>();
