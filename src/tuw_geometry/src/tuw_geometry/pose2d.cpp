#include "tuw_geometry/pose2d.h"
using namespace tuw;

Pose2D::Pose2D() : position_(), orientation_ ( 0 ) {};
Pose2D::Pose2D ( const Point2D &p, double orientation_ ) : position_ ( p ), orientation_ ( orientation_ ), cossin_uptodate_ ( false ) {};
Pose2D::Pose2D ( const Pose2D &p ) : position_ ( p.position_ ), orientation_ ( p.orientation_ ), cossin_uptodate_ ( false ) {};
Pose2D::Pose2D ( double x, double y, double orientation_ ) : position_ ( x,y ), orientation_ ( orientation_ ), cossin_uptodate_ ( false ) {};
Pose2D::Pose2D ( const cv::Vec<double, 3> &s ) : position_ ( s ( 0 ),s ( 1 ) ), orientation_ ( s ( 2 ) ), cossin_uptodate_ ( false ) {};

/** set the pose
  * @param x
  * @param y
  * @param phi (orientation_)
  * @return this reference
  **/
Pose2D &Pose2D::set ( double x, double y, double phi ) {
    angle_normalize ( phi, -M_PI, +M_PI );
    position_ = cv::Vec<double, 3> ( x, y, 1 ), orientation_ = phi;
    cossin_uptodate_ = false;
    return *this;
}
/**
 * set the pose based on two points in world coordinates
 * @param position
 * @param point_ahead
 * @return this reference
 **/
Pose2D &Pose2D::set ( const Point2D &position, const Point2D &point_ahead ) {
    position_.set ( position.x(), position.y() );
    double dx = point_ahead.x() - position.x(), dy = point_ahead.y() - position.y();
    orientation_ = atan2 ( dy,dx );
    cossin_uptodate_ = false;
    return *this;
}
/** set the pose
  * @param p pose
  * @return this reference
  **/
Pose2D &Pose2D::set ( const Pose2D &p ) {
    position_ = p.position_, orientation_ = p.orientation_;
    cossin_uptodate_ = false;
    return *this;
}
/** location as vector
  * @return translational
  **/
const Point2D &Pose2D::position () const {
    return position_;
}
/** point infront of the pose
 * @param d distance ahead
 * @return point
 **/
Point2D Pose2D::point_ahead ( double d ) const {
    update_cached_cos_sin();
    return Point2D ( x() + costheta_ * d, y() + sintheta_ * d );
}
/** translational x component
  * @return x component
  **/
const double &Pose2D::x () const {
    return position_[0];
}
/** translational y component
  * @return y component
  **/
const double &Pose2D::y () const {
    return position_[1];
}
/** roational component
  * @return rotation
  **/
const double &Pose2D::theta () const {
    return orientation_;
}
/** location as vector
  * @return translational
  **/
Point2D &Pose2D::position () {
    return position_;
}
/** translational x component
  * @return x component
  **/
double &Pose2D::x () {
    return position_[0];
}
/** translational y component
  * @return y component
  **/
double &Pose2D::y () {
    return position_[1];
}
/** roational component
  * @return rotation
  **/
double &Pose2D::theta () {
    cossin_uptodate_ = false;
    return orientation_;
}

/**
 * set funktion for x
 * @param x component
 **/
void Pose2D::set_x ( double v ) {
    this->x() = v;
}
/**
 * get function for x
 * @return x component
 **/
double Pose2D::get_x () const {
    return this->x();
}
/**
 * set funktion for y
 * @param y component
 **/
void Pose2D::set_y ( double v ) {
    this->y() = v;
}
/**
 * get function for y
 * @return y component
 **/
double Pose2D::get_y () const {
    return this->y();
}
/**
 * set funktion for theta
 * @param theta component
 **/
void Pose2D::set_theta ( double v ) {
    this->theta () = v;
}
/**
 * get function for theta
 * @return theta component
 **/
double Pose2D::get_theta () const {
    return this->theta ();
}

/** normalizes the orientation value betwenn -PI and PI
  **/
void Pose2D::normalizeOrientation () {
    angle_normalize ( orientation_, -M_PI, +M_PI );
}
/** computes a transformation matrix
  * @return transformation
  **/
Tf2D Pose2D::tf () const {
    update_cached_cos_sin();
    return cv::Matx33d ( costheta_, -sintheta_, x(), sintheta_, costheta_, y(), 0, 0, 1. );
}
/**
 * retuns a state vector [x, y, theta]
 * @return state vector [x, y, theta]
 **/
cv::Vec<double, 3> Pose2D::state_vector () const {
    return cv::Vec<double, 3> ( x(), y(), theta() );
}
/**
* invert pose
* @return inverted pose
**/
Pose2D Pose2D::inv () const {
    Pose2D p ( -this->x(), -this->y(), -this->theta() );
    return p;
}
/** 
  * transforms a point from pose target space into pose base space 
  * @param src point in pose target space
  * @param des point in pose base space
  * @return ref point in pose base space
  **/
Point2D &Pose2D::transform_into_base(const Point2D &src, Point2D &des) const{
    //update_cached_cos_sin();
    des.set(src.x() * costheta_ - src.y() * sintheta_ + src.h() * x(), 
            src.x() * sintheta_ + src.y() * costheta_ + src.h() * y(),
	    src.h() );
    return des;
}

/**
 * adds a state vector [x, y, theta]
 * @param s object
 * @return this
 **/
Pose2D &Pose2D::operator += ( const cv::Vec<double, 3> &s ) {
    this->x() += s.val[0], this->y() += s.val[1], this->theta() += s.val[2];
    angle_normalize ( this->theta() );
    cossin_uptodate_ = false;
    return *this;
}
/**
 * substracts a state vector [x, y, theta]
 * @param s object
 * @return this
 **/
Pose2D &Pose2D::operator -= ( const cv::Vec<double, 3> &s ) {
    this->x() -= s.val[0], this->y() -= s.val[1], angle_difference ( this->theta(), s.val[2] );
    cossin_uptodate_ = false;
    return *this;
}

/**
 * enforces the recompuation of the cached value of cos(theta) and sin(theta),
 * recomputing it only once when theta changes.
 */
void Pose2D::recompute_cached_cos_sin() const {
    costheta_ = cos ( orientation_ );
    sintheta_ = sin ( orientation_ );
    cossin_uptodate_=true;
}
/**
 * Updates the cached value of cos(phi) and sin(phi),
 * recomputing it only once when phi changes.
 **/
void Pose2D::update_cached_cos_sin() const {
    if ( cossin_uptodate_ ) {
        return;
    }
    recompute_cached_cos_sin();
}
/** 
  * get a (cached) value of cos(theta), 
  * recomputing it only once when theta changes. 
  * @return cos(theta)
 **/
double Pose2D::theta_cos() const { 
  update_cached_cos_sin(); 
  return costheta_;   
}
/** 
  * get a (cached) value of cos(theta), 
  * recomputing it only once when theta changes. 
  * @return sin(theta)
 **/
double Pose2D::theta_sin() const { 
  update_cached_cos_sin(); 
  return sintheta_;   
}
/**
  * returns x, y and theta as formated string
  * @param format using printf format
  * @return string
  **/  
std::string Pose2D::str(const char* format) const
{
    char str[0xFF];
    sprintf(str,format, x(), y(), theta()); 
    return std::string(str);
}

/** 
  * compares with within tolerance
  * @param o 
  * @param tolerance 
  **/
bool Pose2D::equal( const Pose2D& o, double tolerance) const {
      double d_position = cv::norm(o.position() - this->position());
      double d_angle = angle_difference(o.theta(),this->theta());
      return (d_angle < tolerance) && (fabs(d_angle) < tolerance);
}