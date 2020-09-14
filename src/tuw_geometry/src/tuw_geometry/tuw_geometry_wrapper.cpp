#include <boost/python.hpp>
#include "tuw_geometry/point2d.h"
#include "tuw_geometry/pose2d.h"

using namespace boost::python;
using namespace tuw;

double Point2Dgetitem ( Point2D& f, int index ) {
    if ( index < 0 || index >=3 ) {
        PyErr_SetString ( PyExc_IndexError, "index out of range" );
        throw boost::python::error_already_set();;
    }
    return f.val[index];
}

void Point2Dsetitem ( Point2D& f, int index, int val ) {
    if ( index < 0 || index >=3 ) {
        PyErr_SetString ( PyExc_IndexError, "index out of range" );
        throw boost::python::error_already_set();;
    }
    f.val[index] = val;
}

    BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(str_overloads, str, 0, 1)
BOOST_PYTHON_MODULE ( tuw_geometry_wrapper ) {

    Point2D & ( Point2D::*Point2Dset2 ) ( double, double )                          = &Point2D::set;
    Point2D & ( Point2D::*Point2Dset3 ) ( double, double, double )                  = &Point2D::set;
    cv::Point_<double>  & ( Point2D::*Point2cv ) ()                                      = &Point2D::cv;

    class_<Point2D> ( "Point2D" )
    .def ( init<double, double>() )
    .def ( init<double, double, double>() )
    .add_property("x",             &Point2D::get_x, &Point2D::set_x)
    .add_property("y",             &Point2D::get_y, &Point2D::set_y)
    .add_property("h",             &Point2D::get_h, &Point2D::set_h)
    .def ( "angle",                &Point2D::angle )
    .def ( "inside",               &Point2D::inside )
    .def ( "__getitem__",          &Point2Dgetitem )
    .def ( "__setitem__",          &Point2Dsetitem )
    .def ( "set",                  Point2Dset2, return_value_policy<copy_non_const_reference>() )
    .def ( "set",                  Point2Dset3, return_value_policy<copy_non_const_reference>() )
    .def ( "cv",                   Point2cv, return_value_policy<copy_non_const_reference>() )
    .def("__str__", &Pose2D::str, str_overloads())
    .def("str", &Pose2D::str, str_overloads());


    Pose2D  & ( Pose2D::*Pose2Dset3 ) ( double, double, double )                = &Pose2D::set;
    Point2D & ( Pose2D::*Pose2Dposition ) ()                                    = &Pose2D::position;


    class_<Pose2D> ( "Pose2D" )
    .def ( init<double, double, double>() )
    .def ( init<const Point2D&, double>() )
    .add_property("x",             &Pose2D::get_x, &Pose2D::set_x)
    .add_property("y",             &Pose2D::get_y, &Pose2D::set_y)
    .add_property("theta",         &Pose2D::get_theta, &Pose2D::set_theta)
    .def ( "set", Pose2Dset3, return_value_policy<copy_non_const_reference>() )
    .def ( "position", Pose2Dposition, return_value_policy<copy_non_const_reference>() )
    .def ( "recompute_cached_cos_sin", &Pose2D::recompute_cached_cos_sin )
    .def ( "transform_into_base", &Pose2D::transform_into_base, return_value_policy<copy_non_const_reference>())
    //.def ( "__str__", &Pose2D::str );
    .def("__str__", &Pose2D::str, str_overloads())
    .def("str", &Pose2D::str, str_overloads());
}
