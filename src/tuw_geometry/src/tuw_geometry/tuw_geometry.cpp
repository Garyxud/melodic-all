
#include <tuw_geometry/tuw_geometry.h>
#include <iomanip>

using namespace tuw;

std::map<DistributionType, std::string> DistributionTypeName =  
    {{NORMAL_DISTRIBUTION, "normal distribution"}, 
     {UNIFORM_DISTRIBUTION, "uniform distribution"}, 
     {GRID_DISTRIBUTION, "grid distribution"}};

     
std::string format(const cv::Mat_<int8_t> &m){
    std::stringstream ss;
    for (int r = 0; r < m.rows; r++) {
        ss << (r==0?"[":" "); 
        for (int c = 0; c < m.cols; c++) {
            ss << (c==0?"":", ") << std::setw(6) << m(r,c);
        }
        ss << (r<m.cols-1?";":"]") << std::endl; 
    } 
    return ss.str();
}
std::string format(const cv::Mat_<int> &m){
    std::stringstream ss;
    for (int r = 0; r < m.rows; r++) {
        ss << (r==0?"[":" "); 
        for (int c = 0; c < m.cols; c++) {
            ss << (c==0?"":", ") << std::setw(12) << m(r,c);
        }
        ss << (r<m.cols-1?";":"]") << std::endl; 
    } 
    return ss.str();
}
std::string format(const cv::Mat_<float> &m){
    std::stringstream ss;
    for (int r = 0; r < m.rows; r++) {
        ss << std::setprecision(std::numeric_limits<float>::digits10 + 1);
        ss << (r==0?"[":" "); 
        for (int c = 0; c < m.cols; c++) {
            ss << (c==0?"":", ") << std::setw(12) << m(r,c);
        }
        ss << (r<m.cols-1?";":"]") << std::endl; 
    } 
    return ss.str();
}
std::string format(const cv::Mat_<double> &m){
    std::stringstream ss;
    for (int r = 0; r < m.rows; r++) {
        ss << std::setprecision(std::numeric_limits<double>::digits10 + 1);
        ss << (r==0?"[":" "); 
        for (int c = 0; c < m.cols; c++) {
            ss << (c==0?"":", ") << std::setw(24) << m(r,c);
        }
        ss << (r<m.cols-1?";":"]") << std::endl; 
    } 
    return ss.str();    
}
