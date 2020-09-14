#pragma once


#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>
#include "stdutils.hpp"

namespace Config{

class Properties {
public:
	Properties();
	Properties(YAML::Node node);
	//~Properties();

	/**
	 * @brief returns as string the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * 
	 */
	std::string asString(std::string element);


	/**
	 * @brief returns as string the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * @param default value
	 * 
	 */
	std::string asStringWithDefault(std::string element, std::string dataDefault);

	/**
	 * @brief returns as float the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * 
	 */
	float asFloat(std::string element);

	/**
	 * @brief returns as float the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * @param default value
	 * 
	 */
	float asFloatWithDefault(std::string element, float dataDefault);

	/**
	 * @brief returns as integer the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * 
	 */
	int asInt(std::string element);

	/**
	 * @brief returns as integer the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * @param default value
	 * 
	 */
	int asIntWithDefault(std::string element, int dataDefault);

	/**
	 * @brief returns as double the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * 
	 */
	double asDouble(std::string element);

	/**
	 * @brief returns as double the propery given 
	 *
	 * @param route to element separated by dots (example: "carViz.Camera.proxy")
	 * @param default value
	 * 
	 */
	double asDoubleWithDefault(std::string element, double dataDefault);

		

	YAML::Node getNode();


private:
	YAML::Node node;

	/**
	 * @brief makes recursively sear for element given in names
	 *
	 *
	 * @param yaml node in which search
	 * @param vector of elements names (route to element of last position of vector)
	 * 
	 *
	 * @return yaml node of element
	 */
	YAML::Node searchNode(YAML::Node n, std::vector<std::string> names);

};


/**
 * @brief function to make printable config class
 */
inline
std::ostream& operator<< (std::ostream & out, Properties & data) {
    out << data.getNode(); 
    return out ;
}

}
