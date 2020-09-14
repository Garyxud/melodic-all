#ifndef NMEA_GPS_PLUGIN_NMEA_GPS_PLUGIN_H_INCLUDED
#define NMEA_GPS_PLUGIN_NMEA_GPS_PLUGIN_H_INCLUDED

/**
 * @file nmea_gps_plugin.h
 * @author Masaya Kataoka (ms.kataoka@gmail.com)
 * @brief definition for the NmeaGpsPlugin class
 * @version 0.1
 * @date 2019-06-16
 * 
 * @copyright Copyright (c) 2019
 */

// Headers in Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>

// Headers in ROS
#include <ros/ros.h>
#include <nmea_msgs/Sentence.h>
#include <geographic_msgs/GeoPose.h>
#include <quaternion_operation/quaternion_operation.h>
#include <geodesy/utm.h>
#include <geodesy/wgs84.h>

// Headers in STL
#include <time.h>
#include <math.h>
#include <memory>
#include <iomanip>

// Headers in this package
#include <nmea_gps_plugin/gps_sensor_model.h>

// Headers in Boost
#include <boost/optional.hpp>

namespace nmea_gps_plugin
{
    /**
     * @brief default parameters
     * 
     */
    namespace default_param
    {
        /**
         * @brief initial longitude of the robot
         * 
         */
        constexpr double reference_longitude = 0.0;
        /**
         * @brief initial longitude of the robot
         * 
         */
        constexpr double reference_latitude = 0.0;
        /**
         * @brief nitial heading of the robot
         * 
         */
        constexpr double reference_heading = 0.0;
        /**
         * @brief initial altitude of the robot
         * 
         */
        constexpr double reference_altitude = 0.0;
        /**
         * @brief publish rate of the each sentence
         * 
         */
        constexpr double publish_rate = 1.0;
        /**
         * @brief topic name of the nmea_sentence topic
         * 
         */
        const std::string nmea_topic = "/nmea_sentence";
        /**
         * @brief gaussian noise of the posision
         * 
         */
        constexpr double position_gaussiaa_noise = 0.05;
        /**
         * @brief gaussian noise of the orientation
         * 
         */
        constexpr double orientation_gaussian_noise = 0.05;
        /**
         * @brief gaussian noise of the velocity
         * 
         */
        constexpr double velocity_gaussian_noise = 0.05;
    }
}

namespace gazebo
{
    /**
     * @brief Class of the NMEA gps plugin
     * 
     */
    class NmeaGpsPlugin : public ModelPlugin
    {
        public:
            NmeaGpsPlugin();
            virtual ~NmeaGpsPlugin();
        protected:
            /**
             * @brief Load parameters for the nmea gps plugin
             * 
             */
            virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf);
            /**
             * @brief Reset the nmea gps plugin.
             * 
             */
            virtual void Reset();
            /**
             * @brief Update the sensor state and publish nmea sentence.
             * 
             */
            virtual void Update();
        private:
            physics::WorldPtr world_ptr_;
            physics::LinkPtr link_ptr_;
            physics::ModelPtr model_ptr_;
            ros::NodeHandle node_handle_;
            std::string namespace_;
            std::string link_name_;
            std::string frame_id_;
            std::string nmea_topic_;
            double reference_altitude_;
            double reference_longitude_;
            double reference_latitude_;
            double reference_heading_;
            double publish_rate_;
            ros::Publisher nmea_pub_;
            geographic_msgs::GeoPose initial_pose_;
            geographic_msgs::GeoPose current_geo_pose_;
            geodesy::UTMPose initial_utm_pose_;
            //UpdateTimer update_timer_;
            event::ConnectionPtr update_connection_;
            boost::optional<common::Time> last_publish_timestamp_;
            std::string getCheckSum(std::string sentence);
            /**
             * @brief Get Unix time from the timestmap.
             * 
             */
            std::string getUnixTime(ros::Time stamp);
            /**
             * @brief Get Unix day from the timestamp.
             * 
             */
            std::string getUnixDay(ros::Time stamp);
            /**
             * @brief generate GPRMC sentence
             * @sa https://docs.novatel.com/OEM7/Content/Logs/GPRMC.htm
             */
            nmea_msgs::Sentence getGPRMC(ros::Time stamp);
            /**
             * @brief generate GPGGA sentence
             * @sa https://docs.novatel.com/OEM7/Content/Logs/GPGGA.htm?Highlight=GPGGA
             */
            nmea_msgs::Sentence getGPGGA(ros::Time stamp);
            /**
             * @brief generate GPVTG sentence
             * @sa https://docs.novatel.com/OEM7/Content/Logs/GPVTG.htm?Highlight=GPVTG
             */
            nmea_msgs::Sentence getGPVTG(ros::Time stamp);
            nmea_msgs::Sentence getGPHDT(ros::Time stamp);
            /**
             * @brief Convert DDD -> DMM format
             * 
             */
            std::string convertToDmm(double value);
            std::string getHexString(uint8_t value);
            geometry_msgs::Twist current_twist_;
            std::unique_ptr<GpsSensorModel> sensor_model_ptr_;
            double position_gaussiaa_noise_;
            double orientation_gaussian_noise_;
            double velocity_gaussian_noise_;
#if (GAZEBO_MAJOR_VERSION >= 8)
            boost::optional<ignition::math::Pose3d> initial_gazebo_pose_;
#else
            boost::optional<gazebo::math::Pose> initial_gazebo_pose_;
#endif
    };
}

#endif  //NMEA_GPS_PLUGIN_NMEA_GPS_PLUGIN_H_INCLUDED