#ifndef QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED
#define QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED

/**
 * @file quaternion_operation.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definitions of quaternion operation
 * @version 0.1
 * @date 2019-04-21
 * 
 * @copyright Copyright (c) 2019
 * 
 */

//headers in ROS
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

//headers in Eigen
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

/**
 * @brief + Operator overload for geometry_msgs::Quaternion (Addition)
 * 
 * @param quat1 
 * @param quat2 
 * @return geometry_msgs::Quaternion result of Addition
 */
geometry_msgs::Quaternion operator+(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2);

/**
 * @brief * Operator overload for geometry_msgs::Quaternion (Multiplication)
 * 
 * @param quat1 
 * @param quat2 
 * @return geometry_msgs::Quaternion result of Multiplication
 */
geometry_msgs::Quaternion operator*(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2);

/**
 * @brief namespace of quaternion_operation ROS package
 * 
 */
namespace quaternion_operation
{
    /**
     * @brief convert Euler angles to Quaternion 
     * 
     * @param euler Euler angles
     * @return geometry_msgs::Quaternion Quaternion  
     */
    geometry_msgs::Quaternion convertEulerAngleToQuaternion(geometry_msgs::Vector3 euler);
    
    /**
     * @brief Get the Rotation Matrix from geometry_msgs::Quaternion 
     * 
     * @param quat input geometry_msgs::Quaternion 
     * @return Eigen::Matrix3d get 3x3 Rotation Matrix
     */
    Eigen::Matrix3d getRotationMatrix(geometry_msgs::Quaternion quat);

    /**
     * @brief convert Quaternion to the Euler angle
     * 
     * @param quat Quaternion
     * @return geometry_msgs::Vector3 euler angle 
     */
    geometry_msgs::Vector3 convertQuaternionToEulerAngle(geometry_msgs::Quaternion quat);

    /**
     * @brief checke 2 double values are equal or not
     * 
     * @param a 
     * @param b 
     * @return true a == b
     * @return false a != b
     */
    bool equals(double a,double b);

    /**
     * @brief check 2 Quaternion values are equal or not
     * 
     * @param quat1 
     * @param quat2 
     * @return true a == b
     * @return false a != b
     */
    bool equals(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2);

    /**
     * @brief convert geometry_msgs::Quaternion to Eigen::MatrixXd
     * 
     * @param quat input Quaternion
     * @return Eigen::MatrixXd converted Eigen Matrix (4,1)
     */
    Eigen::MatrixXd convertToEigenMatrix(geometry_msgs::Quaternion quat);

    /**
     * @brief get conjugate Quaternion 
     * 
     * @param quat1 input Quaternion 
     * @return geometry_msgs::Quaternion conjugate Quaternion 
     */
    geometry_msgs::Quaternion conjugate(geometry_msgs::Quaternion quat1);

    /**
     * @brief rotate Quaternion 
     * 
     * @param from from pose orientation
     * @param rotation Rotation quaternion
     * @return geometry_msgs::Quaternion Rotated pose orientation
     */
    geometry_msgs::Quaternion rotation(geometry_msgs::Quaternion from,geometry_msgs::Quaternion rotation);

    /**
     * @brief Get the Rotation from 2 Quaternions 
     * 
     * @param from from pose orientation
     * @param to to pose orientation
     * @return geometry_msgs::Quaternion Rotation between 2 pose orientation described as Quaternion
     * @sa quaternion_operation::rotation
     */
    geometry_msgs::Quaternion getRotation(geometry_msgs::Quaternion from,geometry_msgs::Quaternion to);

    /**
     * @brief Spherical linear interpolation function for geometry_msgs::Quaternion
     * 
     * @param quat1 
     * @param quat2 
     * @param t parameter for interpolation (if t=0,return==quat1),(if t=1,return==quat2)
     * @return geometry_msgs::Quaternion result of Spherical linear interpolation opertaion
     */
    geometry_msgs::Quaternion slerp(geometry_msgs::Quaternion quat1,geometry_msgs::Quaternion quat2,double t);
}

#endif  //QUATERNION_OPERATION_QUATERNION_OPERATION_H_INCLUDED