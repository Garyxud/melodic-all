/**
 * @file gps_sensor_model.cpp
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief implimentation of GPS noise model
 * @version 0.1
 * @date 2019-06-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <nmea_gps_plugin/gps_sensor_model.h>

GpsSensorModel::GpsSensorModel(double position_gaussian_noise,
    double orientation_gaussian_noise,double velocity_gaussian_noise) 
    : position_gaussian_noise(position_gaussian_noise),
    position_dist_(0.0,position_gaussian_noise),
    orientation_gaussian_noise(orientation_gaussian_noise),
    orientation_dist_(0.0,orientation_gaussian_noise),
    velocity_gaussian_noise(velocity_gaussian_noise),
    twist_dist_(0.0,velocity_gaussian_noise),
    engine_(seed_gen_())
{

}

GpsSensorModel::~GpsSensorModel()
{
    
}

geometry_msgs::Twist GpsSensorModel::addGaussianNoise(geometry_msgs::Twist twist)
{
    twist.linear.x = twist.linear.x + twist_dist_(engine_);
    twist.linear.y = twist.linear.y + twist_dist_(engine_);
    twist.linear.z = twist.linear.z + twist_dist_(engine_);
    twist.angular.x = twist.angular.x + twist_dist_(engine_);
    twist.angular.y = twist.angular.y + twist_dist_(engine_);
    twist.angular.z = twist.angular.z + twist_dist_(engine_);
    return twist;
}

geometry_msgs::Quaternion GpsSensorModel::addGaussianNoise(geometry_msgs::Quaternion orientation)
{
    geometry_msgs::Vector3 vec = quaternion_operation::convertQuaternionToEulerAngle(orientation);
    vec.z = vec.z + orientation_dist_(engine_);
    return quaternion_operation::convertEulerAngleToQuaternion(vec);
}

geodesy::UTMPoint GpsSensorModel::addGaussianNoise(geodesy::UTMPoint point)
{
    point.easting = point.easting + position_dist_(engine_);
    point.northing = point.northing + position_dist_(engine_);
    return point;
}