
#include "properties.hpp"

namespace Config{


Properties::Properties(){
}

Properties::Properties(YAML::Node node){
    this->node = node;
}

std::string 
Properties::asString(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<std::string>();
}

std::string 
Properties::asStringWithDefault(std::string element, std::string dataDefault){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    std::string data;
    try{
        data = nod.as<std::string>();
    }catch(YAML::BadConversion e){
        data = dataDefault;
    }
    return data;
}

float 
Properties::asFloat(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<float>();
}

float 
Properties::asFloatWithDefault(std::string element, float dataDefault){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    float data;
    try{
        data = nod.as<float>();
    }catch(YAML::BadConversion e){
        data = dataDefault;
    }
    return data;
}

int 
Properties::asInt(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<int>();
}

int 
Properties::asIntWithDefault(std::string element, int dataDefault){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    int data;
    try{
        data = nod.as<int>();
    }catch(YAML::BadConversion e){
        data = dataDefault;
    }
    return data;
}

double 
Properties::asDouble(std::string element){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    return nod.as<double>();
}

double 
Properties::asDoubleWithDefault(std::string element, double dataDefault){
    std::vector<std::string> v = std::split(element, ".");

    YAML::Node nod = this->searchNode(this->node, v);
    double data;
    try{
        data = nod.as<double>();
    }catch(YAML::BadConversion e){
        data = dataDefault;
    }
    return data;
}

YAML::Node
Properties::getNode(){
    
    return node;
}



YAML::Node 
Properties::searchNode(YAML::Node n, std::vector<std::string> names){
    YAML::Node nod = n[names[0]];
    names.erase(names.begin()); 

    if (names.size()>0){
        return this->searchNode(nod, names);
    }else{
        return nod;
    }
}


}
