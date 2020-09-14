#include <memory>
#include <tuw_geometry/polar2d.h>

using namespace tuw;

Polar2D::Polar2D () : Point2D ( 0,0 ) {};
Polar2D::Polar2D ( const Point2D &p ) : Point2D ( atan2( p.y(), p.x()), sqrt( p.x() * p.x() + p.y()* p.y() ), 1 ) {};
Polar2D::Polar2D ( double alpha, double rho ) : Point2D ( alpha,rho ) {};
Polar2D::Polar2D ( double alpha, double rho, double h ) : Point2D ( alpha, rho, h ) {};

/**
 * @return alpha
 **/
const double &Polar2D::alpha () const {
    return x();
}
/**
 * @return alpha
 **/
double &Polar2D::alpha () {
    return x();
}
/**
 * @return rho component
 **/
const double &Polar2D::rho () const {
    return y();
}
/**
 * @return rho component
 **/
double &Polar2D::rho () {
    return y();
}
/**
 * @return point in cartesian space
 **/
Point2D Polar2D::point () const {
    return Point2D(cos(alpha()) * rho(), sin(alpha()) * rho());
}
