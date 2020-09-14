#include "tuw_geometry/point2d.h"
#include "tuw_geometry/polar2d.h"
using namespace tuw;

Point2D::Point2D() : cv::Vec<double, 3 > ( 0,0,1 ) {} ;
Point2D::Point2D ( const Point2D &p ) : cv::Vec<double,3> ( p ) {};
Point2D::Point2D ( const cv::Point &p ) : cv::Vec<double,3> ( p.x,p.y,1. ) {};
Point2D::Point2D ( double x, double y ) : cv::Vec<double,3> ( x, y, 1. ) {};
Point2D::Point2D ( double x, double y, double h ) : cv::Vec<double,3> ( x, y, h ) {};
Point2D::Point2D ( const Polar2D& p ) : cv::Vec<double,3> ( p.rho() * cos ( p.alpha() ), p.rho() * sin ( p.alpha() ), 1 ) {};
/**
 * sets values
 * @param x
 * @param y
 * @return this reference
 **/
Point2D &Point2D::set ( double x, double y ) {
    this->val[0] = x, this->val[1] = y;
    return *this;
}
/**
 * sets values
 * @param x
 * @param y
 * @return this reference
 **/
Point2D &Point2D::set ( double x, double y, double h ) {
    this->val[0] = x, this->val[1] = y, this->val[2] = h;
    return *this;
}
/**
 * translational x component
 * @return rotation
 **/
const double &Point2D::x () const {
    return this->val[0];
}
/**
 * translational x component
 * @return rotation
 **/
double &Point2D::x () {
    return this->val[0];
}
/**
 * translational y component
 * @return y component
 **/
const double &Point2D::y () const {
    return this->val[1];
}
/**
 * translational y component
 * @return y component
 **/
double &Point2D::y () {
    return this->val[1];
}
/**
 * homogeneous component
  * @return rotation
  **/
const double &Point2D::h () const {
    return this->val[2];
}
/**
 * homogeneous component
  * @return rotation
  **/
double &Point2D::h () {
    return this->val[2];
}

/**
 * set funktion for x
 * @param x component
 **/
void Point2D::set_x ( double v ) {
    this->x() = v;
}
/**
 * get function for x
 * @return x component
 **/
double Point2D::get_x () const {
    return this->x();
}
/**
 * set funktion for y
 * @param y component
 **/
void Point2D::set_y ( double v ) {
    this->y() = v;
}
/**
 * get function for y
 * @return y component
 **/
double Point2D::get_y () const {
    return this->y();
}
/**
 * set funktion for h
 * @param h component
 **/
void Point2D::set_h ( double v ) {
    this->h() = v;
}
/**
 * get function for h
 * @return h component
 **/
double Point2D::get_h () const {
    return this->h();
}
/**
 * vector without homogeneous component
 * @return state vector
 **/
cv::Vec<double, 2> Point2D::vector () const {
    return cv::Vec< double, 2  > ( this->x(), this->y() );
}
/**
 * returns the distance to an other point
 * @return disance
 **/
double  Point2D::distanceTo ( const Point2D &p ) const {
    double dx =  p.val[0] - this->val[0], dy =  p.val[1] - this->val[1];
    return sqrt ( dx*dx+dy*dy );
}
/**
 * returns a cv::Point_<double> reference
 * @return cv
 **/
const cv::Point_<double>  &Point2D::cv () const {
    return ( cv::Point_<double>  & ) *this;
}
/**
 * returns a cv::Point_<double> reference
 * @return cv
 **/
cv::Point_<double>  &Point2D::cv () {
    return ( cv::Point_<double>  & ) *this;
}
/**
 * angle form origin to point (alpha in polar space)
 * @see radius
 * @see Polar2D
 * @return angle between -PI and +PI
 **/
double Point2D::angle () const {
    return atan2 ( this->val[1], this->val[0] );
}
/**
  * distance to origin (rho in polar space)
  * @see angle
  * @see Polar2D
  * @return distance
  **/
double Point2D::radius () const {
    return sqrt ( this->val[0]*this->val[0] + this->val[1]*this->val[1] );
}
/**
 * checks if a point is within a rectangle
 * @param x0 top left x
 * @param y0 top left y
 * @param x1 bottom right x
 * @param y1 bottom right y
 * @return true if inside
 **/
bool Point2D::inside ( double x0, double y0, double x1, double y1 ) const {
    return ( ( x() >= x0 ) && ( x() <= x1 ) && ( y() >= y0 ) && ( y() <= y1 ) );
}

/**
  * returns x and y as formated string
  * @param format using printf format
  * @return string
  **/  
std::string Point2D::str(const char* format) const
{
    char str[0xFF];
    sprintf(str,format, x(), y()); 
    return std::string(str);
}

/** 
  * compares with within tolerance
  * @param o 
  * @param tolerance 
  **/
bool Point2D::equal( const Point2D& o, double tolerance) const {
      double d = cv::norm(o - *this);
      return d < tolerance;
}