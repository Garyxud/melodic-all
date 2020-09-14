#ifndef TUW_GEOMETRY_UTILS
#define TUW_GEOMETRY_UTILS

#include <string>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>

namespace tuw
{
typedef cv::Matx<double, 3, 3> Tf2D;

/**
 * creates a new vector by adding a single component
 * usable to homogeneous vectors
 * @param src to normalize
 * @param value
 * @return extended vector
 **/
inline cv::Vec<double, 3>  append ( const cv::Vec<double, 2> &src, double value = 1.0 )
{
    return cv::Vec<double, 3> ( src.val[0], src.val[1], value );
}

/**
 * creates a new vector by adding a single component
 * usable to homogeneous vectors
 * @param src to normalize
 * @param value
 * @return extended vector
 **/
inline cv::Vec<double, 4>  append ( const cv::Vec<double, 3> &src, double value = 1.0 )
{
    return cv::Vec<double, 4> ( src.val[0], src.val[1], src.val[2], value );
}

/**
 * normalizes an angle between min_angle and max_angle but max_angle - min_angle >= 2PI
 * @param angle to normalize
 * @param min_angle
 * @param max_angle
 * @return normalize angle
 **/
inline double angle_normalize ( double angle, double min_angle = -M_PI, double max_angle = +M_PI )
{
    while ( angle > max_angle ) {
        angle -= ( 2.*M_PI );
    }
    while ( angle < min_angle ) {
        angle += ( 2.*M_PI );
    }
    return angle;
}
/**
 * computes the angle difference between two angles by taking into account the circular space
 * @param alpha0
 * @param angle1
 * @return difference
 **/
inline double angle_difference ( double alpha0, double angle1 )
{
    return atan2 ( sin ( alpha0-angle1 ), cos ( alpha0-angle1 ) );
}

/**
 * Euler Pitch to Quaternion roation arount Y
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void EulerPitchToQuaternion ( double pitch, Quaternion& q )
{
    double cp = cos ( pitch * 0.5 );
    double sp = sin ( pitch * 0.5 );

    q.w = cp;
    q.x = 0;
    q.y = sp;
    q.z = 0;

}

/**
 * Euler Roll to Quaternion roation arount X
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void EulerToQuaternion ( double roll, const Quaternion& q )
{
    double cr = cos ( roll * 0.5 );
    double sr = sin ( roll * 0.5 );

    q.w = cr;
    q.x = sr;
    q.y = 0.;
    q.z = 0.;
}

/**
 * Euler Yaw to Quaternion rotation around Z
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void EulerYawToQuaternion ( double yaw, Quaternion& q )
{
    double cy = cos ( yaw * 0.5 );
    double sy = sin ( yaw * 0.5 );

    q.w = cy;
    q.x = 0.;
    q.y = 0.;
    q.z = sy;
}

/**
 * Euler to Quaternion
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void EulerToQuaternion ( double pitch, double roll, double yaw, const Quaternion& q )
{
    double cy = cos ( yaw * 0.5 );
    double sy = sin ( yaw * 0.5 );
    double cr = cos ( roll * 0.5 );
    double sr = sin ( roll * 0.5 );
    double cp = cos ( pitch * 0.5 );
    double sp = sin ( pitch * 0.5 );

    q.w = cy * cr * cp + sy * sr * sp;
    q.x = cy * sr * cp - sy * cr * sp;
    q.y = cy * cr * sp + sy * sr * cp;
    q.z = sy * cr * cp - cy * sr * sp;
    return q;
}

/**
 * Quaternion to an euler roll angle
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void QuaternionToRoll ( const Quaternion& q, double& roll )
{
    // roll (x-axis rotation)
    double sinr = +2.0 * ( q.w * q.x + q.y * q.z );
    double cosr = +1.0 - 2.0 * ( q.x * q.x + q.y * q.y );
    roll = atan2 ( sinr, cosr );
}
/**
 * Quaternion to an euler pitch angle
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void QuaternionToPitch ( const Quaternion& q, double& pitch )
{
    // pitch (y-axis rotation)
    double sinp = +2.0 * ( q.w * q.y - q.z * q.x );
    if ( fabs ( sinp ) >= 1 ) {
        pitch = copysign ( M_PI / 2, sinp );    // use 90 degrees if out of range
    } else {
        pitch = asin ( sinp );
    }
}
/**
 * Quaternion to an euler yaw angle
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void QuaternionToYaw ( const Quaternion& q, double& yaw )
{
    // yaw (z-axis rotation)
    double siny = +2.0 * ( q.w * q.z + q.x * q.y );
    double cosy = +1.0 - 2.0 * ( q.y * q.y + q.z * q.z );
    yaw = atan2 ( siny, cosy );
}

/**
 * Quaternion to euler angles
 * @see https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
 **/
template <typename Quaternion>
void QuaternionToEuler ( const Quaternion& q, double& roll, double& pitch, double& yaw )
{
    QuaternionToRoll ( q, roll ), QuaternionToPitch ( q, pitch ),  QuaternionToYaw ( q, yaw );
}

std::string format ( const cv::Mat_<int8_t> &m );
std::string format ( const cv::Mat_<int> &m );
std::string format ( const cv::Mat_<float> &m );
std::string format ( const cv::Mat_<double> &m );
std::string format ( const cv::Matx33d &m );
}

#endif // TUW_GEOMETRY_UTILS
