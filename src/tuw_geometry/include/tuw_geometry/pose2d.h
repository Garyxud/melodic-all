#ifndef POSE2D_H
#define POSE2D_H

#include <memory>
#include <opencv2/core/core.hpp>
#include <tuw_geometry/point2d.h>
namespace tuw
{
class Pose2D;
using Pose2DPtr = std::shared_ptr< Pose2D >;
using Pose2DConstPtr = std::shared_ptr< Pose2D const>;

/**
 * class to represent a pose in 2D space
 * the class caches the cos(theta) and sin(theta) values
 **/
class Pose2D
{
protected:
    Point2D position_;    /// position
    double orientation_;  /// rotation in rad
    mutable double costheta_, sintheta_; // precomputed cos() & sin() of theta.
    mutable bool   cossin_uptodate_;

    /**
     * Updates the cached value of cos(phi) and sin(phi),
     * recomputing it only once when phi changes.
     **/
    void update_cached_cos_sin() const;

public:
    Pose2D();
    Pose2D ( const Point2D &p, double orientation );
    Pose2D ( const Pose2D &p );
    Pose2D ( double x, double y, double orientation );
    Pose2D ( const cv::Vec<double, 3> &s );
    /**
     * set the pose
     * @param x
     * @param y
     * @param phi (orientation)
     * @return this reference
     **/
    Pose2D &set ( double x, const double y, double phi ) ;
    /**
     * set the pose based on two points in world coordinates
     * @param position
     * @param point_ahead
     * @return this reference
     **/
    Pose2D &set ( const Point2D &position, const Point2D &point_ahead ) ;

    /**
     * set the pose
     * @param p pose
     * @return this reference
     **/
    Pose2D &set ( const Pose2D &p ) ;

    /**
     * position
     * @return translational
     **/
    const Point2D &position () const;

    /**
     * location as vector
     * @return translational
     **/
    Point2D &position ();

    /**
     * point in front of the pose
     * @param d distance ahead
     * @return point
     **/
    const double &x () const;

    /**
     * translational y component
     * @return y component
     **/
    double &x ();

    /**
     * translational y component
     * @return y component
     **/
    const double &y () const;

    /**
     * rotational component
     * @return rotation
     **/
    double &y ();

    /**
     * rotational component
     * @return rotation
     **/
    const double &theta () const;

    /**
     * position
     * @return rotation
     **/
    double &theta ();

    /**
     * set funktion for x
     * @param x component
     **/
    void set_x ( double v );

    /**
     * get function for x
     * @return x component
     **/
    double get_x () const;

    /**
     * set funktion for y
     * @param y component
     **/
    void set_y ( double v );

    /**
     * get function for y
     * @return y component
     **/
    double get_y () const;

    /**
     * set funktion for theta
     * @param theta component
     **/
    void set_theta ( double v );
    /**
     * transforms a point from pose target space into pose base space
     * @param src point in pose target space
     * @param des point in pose base space
     * @return ref point in pose base space
     **/
    Point2D &transform_into_base ( const Point2D &src, Point2D &des ) const;

    /**
     * get function for theta
     * @return theta component
     **/
    double get_theta () const;

    /**
     * enforces the recompuation of the cached value of cos(theta) and sin(theta),
     * recomputing it only once when theta changes.
     */
    void recompute_cached_cos_sin() const;
    /**
     * get a (cached) value of cos(theta),
     * recomputing it only once when theta changes.
     * @return cos(theta)
     **/
    double theta_cos() const;
    /**
     * get a (cached) value of cos(theta),
     * recomputing it only once when theta changes.
     * @return sin(theta)
     **/
    double theta_sin() const;

    /**
     * computes a transformation matrix
     * @return transformation
     **/
    Point2D point_ahead ( double d = 1. ) const;

    /**
     * normalizes the orientation value betwenn -PI and PI
      **/
    void normalizeOrientation ();

    /**
     * translational x component
     * @return x component
     **/
    Tf2D tf () const;

    /**
     * retuns a state vector [x, y, theta]
     * @return state vector [x, y, theta]
     **/
    cv::Vec<double, 3> state_vector () const;

    /**
     * invert pose
     * @return inverted pose
     **/
    Pose2D inv () const;


    /**
     * adds a state vector [x, y, theta]
     * the orientation will be normalized between -PI and PI
     * @param s object
     * @return thisl
     **/
    Pose2D &operator += ( const cv::Vec<double, 3> &s );

    /**
    * substracts a state vector [x, y, theta]
    * the orientation will be normalized between -PI and PI
    * @param s object
    * @return this
    **/
    Pose2D &operator -= ( const cv::Vec<double, 3> &s ) ;

    /**
     * Stream extraction
     * @param os outputstream
     * @param o object
     * @return stream
     **/
    friend std::ostream &operator << ( std::ostream &os, const Pose2D &o )
    {
        os << "[" << o.x() <<  ", " << o.y() <<  ", " << o.theta()  << "]";
        return os;
    }

    /**
     * returns x, y and theta as formated string
     * @param format using printf format
     * @return string
     **/
    std::string str ( const char* format = "[%6.4lf, %6.4lf, %6.5lf]" ) const;

    /**
      * compares with within tolerance
      * @param o
      * @param tolerance
      **/
    bool equal ( const Pose2D& o, double tolerance ) const;


    /**
    * Used to copy the Pose2D object into a ros geometry_msgs::Pose
    * @param copy t
    **/
    template <typename T>
    void copyToROSPose ( T &des ) const
    {
        des.position.x = x();
        des.position.y = y();
        des.position.z = 0.;
        EulerYawToQuaternion ( theta(), des.orientation );
    }
};

}
#endif //POSE2D_H

