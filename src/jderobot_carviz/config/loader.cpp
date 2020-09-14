#include "loader.hpp"

namespace jderobotconfig{
namespace loader{

std::string
findConfigFile(const std::string& filename){
    if (std::fileexists(filename))
        return filename;

    std::string path_holders[] = {getEnvironmentVariable(CONFIG_PATH_NAME)};
    
    for (int i=0; i<2; i++){
        if (path_holders[i].empty()) continue;
        for (std::string path : std::split(path_holders[i], ":")){
            if (path.empty()) continue;
            std::string filepath(path+"/"+filename);
            if (std::fileexists(filepath))
                return filepath;
        }
    }

    return "";
}

Config::Properties 
load(std::string filename){
    std::string filepath = findConfigFile(filename);
    if (filepath.empty()){
        YAML::Exception e(YAML::Mark(),"jderobot/config/loader.cpp: file " + filepath + " Not Found");
        throw e;
    }
    YAML::Node nodeConfig = YAML::LoadFile(filepath);

    Config::Properties config(nodeConfig); 
    std::cout<<"[Info] loaded YAML Config file: "<<filepath<<std::endl;
    //properties->setProperty("Ice.Config", filepath);
    return config;
}



}}
