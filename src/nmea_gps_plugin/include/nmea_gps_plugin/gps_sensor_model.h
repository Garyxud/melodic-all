#ifndef NMEA_GPS_PLUGIN_GPS_SENSOR_MODEL_H_INCLUDED
#define NMEA_GPS_PLUGIN_GPS_SENSOR_MODEL_H_INCLUDED

/**
 * @file gps_sensor_model.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief Sensor model of the nmea GPS plugin
 * @version 0.1
 * @date 2019-06-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

// Headers in ROS
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geodesy/utm.h>
#include <quaternion_operation/quaternion_operation.h>

// Headers in STL
#include <random>

/**
 * @brief Class for the GPS sensor model
 * 
 */
class GpsSensorModel
{
public:
    /**
     * @brief variance of the position gaussian noise
     * 
     */
    const double position_gaussian_noise;
    /**
     * @brief variance of the velocity gaussian noise
     * 
     */
    const double velocity_gaussian_noise;
    /**
     * @brief variance of the orientation gaussian noise
     * 
     */
    const double orientation_gaussian_noise;
    /**
     * @brief constructer of the GpsSensorModel class
     * 
     */
    GpsSensorModel(double position_gaussian_noise,double orientation_gaussian_noise,double velocity_gaussian_noise);
    /**
     * @brief destructor of GpsSensorModel class
     * 
     */
    ~GpsSensorModel();
    /**
     * @brief add gausiann noise to the twist
     * 
     */
    geometry_msgs::Twist addGaussianNoise(geometry_msgs::Twist twist);
    /**
     * @brief add gaussian noise to the orientation
     * 
     */
    geometry_msgs::Quaternion addGaussianNoise(geometry_msgs::Quaternion orientation);
    /**
     * @brief add gausiann noise to the point
     * 
     */
    geodesy::UTMPoint addGaussianNoise(geodesy::UTMPoint point);
private:
    /**
     * @brief normal distribution generator for position
     * 
     */
    std::normal_distribution<> position_dist_;
    /**
     * @brief normal distribution generator for orientation
     * 
     */
    std::normal_distribution<> orientation_dist_;
    /**
     * @brief normal distribution generator for twist
     * 
     */
    std::normal_distribution<> twist_dist_;
    /**
     * @brief random seed generator
     * 
     */
    std::random_device seed_gen_;
    /**
     * @brief random generation engine
     * 
     */
    std::default_random_engine engine_;
};
#endif  //NMEA_GPS_PLUGIN_GPS_SENSOR_MODEL_H_INCLUDED