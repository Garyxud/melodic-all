
#include <iostream>
#include <string>
#include "stdutils.hpp"
#include <yaml-cpp/yaml.h>
#include <yaml-cpp/exceptions.h>
//#include <jderobot/config/hardcodedlocations.h>
#include "properties.hpp"


namespace jderobotconfig {
namespace loader {

const std::string CONFIG_PATH_NAME = "JDEROBOT_CONFIG_PATHS";


/**
 * @brief Find filename into all defined search paths.
 * Order is:
 * 1. current dir
 * 2. jderobot paths (*)
 *
 * @return empty if file was not found.
 */
std::string findConfigFile(const std::string& filename);

/**
 * @brief Loads File configuration from passed file.
 *
 * @return new Config::Config or passed one.
 */
Config::Properties load(std::string filename);



}}
