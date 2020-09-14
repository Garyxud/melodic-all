#pragma once

#include <cstdlib>
#include <string>

inline
std::string getEnvironmentVariable(std::string var){
	char* _env = getenv(var.c_str());
	return std::string((_env)?_env:"");
}


//// Fallback std::split
/// source: http://stackoverflow.com/questions/5607589/right-way-to-split-an-stdstring-into-a-vectorstring

#include <vector>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

namespace std {
inline
vector<string> split(string str, string del){
	vector<string> vstrings;
	boost::split(vstrings, str, boost::is_any_of(del));
	return vstrings;
}
}//NS



//// Check if file exists
/// For Linux works for files and directories
/// source: http://www.cplusplus.com/forum/general/1796/
#include <fstream>

namespace std {
inline
bool fileexists(std::string filepath){
	ifstream ifile(filepath.c_str(), ios_base::in);
	return ifile.is_open();
}
}
