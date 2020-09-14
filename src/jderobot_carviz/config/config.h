#include <yaml-cpp/yaml.h>
#include "loader.hpp"
#include "properties.hpp"

namespace Config{


/**
 * @brief loads propierties from a file
 *
 *
 * @param filename
 * 
 *
 * @return config class with all properties
 */
inline
Config::Properties load(int argc, char* argv[])
{	
	std::string filename (argv[1]);
    return jderobotconfig::loader::load(filename);}

} 
