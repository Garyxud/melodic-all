#include <cfloat>
#include <tgmath.h>
#include <iomanip>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tuw_geometry/world_scoped_maps.h>
#include <tuw_geometry/utils.h>


using namespace tuw;

WorldScopedMaps::WorldScopedMaps () : 
  width_pixel_ (-1),
  height_pixel_(-1),
  min_x_(0), max_x_(0), min_y_(0), max_y_(0) {

}

bool WorldScopedMaps::initialized() {
  return ((width_pixel_ != -1) && (height_pixel_ != -1));
}

void WorldScopedMaps::init () {
       
    dx_ = max_x_ - min_x_;
    dy_ = max_y_ - min_y_;
    sx_ = width_pixel_  / dx_;
    sy_ = height_pixel_ / dy_;
    ox_ = width_pixel_  / 2.0;
    oy_ = height_pixel_ / 2.0;
    double ca = cos ( rotation_ ), sa = sin ( rotation_ );
    mx_ = min_x_ + dx_/2.;
    my_ = min_y_ + dy_/2.;
    cv::Matx<double, 3, 3 > Tw ( 1, 0, -mx_, 0, 1, -my_, 0, 0, 1 ); // translation
    cv::Matx<double, 3, 3 > Sc ( sx_, 0, 0, 0, sy_, 0, 0, 0, 1 );   // scaling
    cv::Matx<double, 3, 3 > Sp ( -1, 0, 0, 0, 1, 0, 0, 0, 1 );      // mirroring
    cv::Matx<double, 3, 3 > R ( ca, -sa, 0, sa, ca, 0, 0, 0, 1 );   // rotation
    cv::Matx<double, 3, 3 > Tm ( 1, 0, ox_, 0, 1, oy_, 0, 0, 1 );   // translation
    Mw2m_ = Tm * R * Sp * Sc * Tw;
    
    Mm2w_ = Mw2m_.inv();
}
void WorldScopedMaps::init ( int width_pixel, int height_pixel, double min_x, double max_x, double min_y, double max_y, double rotation ) {
    width_pixel_ = width_pixel,   height_pixel_ = height_pixel;
    min_y_ = std::min ( min_y, max_y );
    max_y_ = std::max ( min_y, max_y );
    min_x_ = std::min ( min_x, max_x );
    max_x_ = std::max ( min_x, max_x );
    rotation_ = rotation;
    
    init();
}

const cv::Matx33d  &WorldScopedMaps::Mw2m () const {
    return Mw2m_;
}
const cv::Matx33d  &WorldScopedMaps::Mm2w () const {
    return Mm2w_;
}
Point2D WorldScopedMaps::w2m ( const Point2D &src ) const {
    return Mw2m_ * src;
}
Point2D WorldScopedMaps::w2m ( double x, double y ) const {
    return w2m(Point2D(x,y));
}
Point2D &WorldScopedMaps::w2m ( const Point2D &src, Point2D &des ) const {
    des = Mw2m_ * src;
    return des;
}
Point2D WorldScopedMaps::m2w ( const Point2D &src ) const {
    return Mm2w_ * src;
}
Point2D WorldScopedMaps::m2w ( double x, double y ) const {
    return m2w(Point2D(x,y));
}
Point2D &WorldScopedMaps::m2w ( const Point2D &src, Point2D &des ) const {
    des = Mm2w_ * src;
    return des;
}
double WorldScopedMaps::max_x () const {
    return max_x_;
}
double WorldScopedMaps::min_x () const  {
    return min_x_;
}
double WorldScopedMaps::scale_x () const  {
    return sx_;
}
double WorldScopedMaps::max_y () const  {
    return max_y_;
}
double WorldScopedMaps::min_y () const  {
    return min_y_;
}
double WorldScopedMaps::scale_y () const  {
    return sy_;
}
int WorldScopedMaps::width () const {
    return width_pixel_;
}
int WorldScopedMaps::height () const {
    return height_pixel_;
}
    
double WorldScopedMaps::scale_w2m (double v) const {
    return v * sx_;
}
std::string WorldScopedMaps::infoHeader() const{
    char buffer[0x1FF];
    Point2D p0 = m2w(0, 0);
    Point2D p1 = m2w(width_pixel_/2, height_pixel_/2);
    sprintf(buffer, "%4i,%4i [px];  %6.2f, %6.2f [m] => %6.2f, %6.2f [px/m]; 0, 0 [px] = %6.2f, %6.2f [m] @ %3.2f [rad]; %4i, %4i [px] = %6.2f, %6.2f [m] @ %3.2f [rad]", 
            width_pixel_, height_pixel_, dx_, dy_, sx_, sy_, p0.x(), p0.y(), rotation_, width_pixel_/2, height_pixel_/2, p1.x(), p1.y(), rotation_);
    return std::string(buffer);    
}
