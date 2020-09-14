#ifndef HEIFU_MAVROS_HPP
#define HEIFU_MAVROS_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <iterator>
#include <numeric>
#include <ctime>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <ros_utils/Console.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geographic_msgs/GeoPose.h>
#include <std_msgs/Bool.h>

#define XBOX_CMD 0
#define LAND_CMD 1
#define SIM_CMD 2

namespace HM
{
    class Heifu_mavros
    {
    public:

        Heifu_mavros ();
        virtual ~Heifu_mavros (){};

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subXBoxTakeOff;
        ros::Subscriber subXBoxLand;
        ros::Subscriber subXBoxDisarm;
        ros::Subscriber subXBoxMoveVel;
        ros::Subscriber subXBoxRawCommand;
        ros::Subscriber subXBoxRawCommandFront;
        ros::Subscriber subLandRawCommand;
        ros::Subscriber subOdomUAV;
        ros::Subscriber subSimCommand;
        ros::Subscriber subAuto;
        ros::Subscriber subSetPointConverter;
        ros::Subscriber subMissionStart;
        ros::Subscriber subMissionStop;
        ros::Subscriber subReturnHome;


        // Services Clients
        ros::ServiceClient srvClientArming;
        ros::ServiceClient srvClientSetMode;
        ros::ServiceClient srvClientTakeOff;

        // Publishers
        ros::Publisher pubMoveVel;
        ros::Publisher pubXBoxBodyCommand;
        ros::Publisher pubLandBodyCommand;
        ros::Publisher pubSimCommand;
        ros::Publisher pubGpsFixState;
        ros::Publisher pubDesiredPosition;
        ros::Publisher pubSetPointConverter;
        ros::Publisher pubTakeoffDiagnostic;
        ros::Publisher pubLandDiagnostic;

        // TF relation
        tf::TransformListener tfListener;
        tf::StampedTransform tfTransform;
        geometry_msgs::TwistStamped msgVel;

        // Parameters
        double paramTakeOffAltitude;
        double paramTakeOffThreshold;
        double paramNodeRate;
        std::string paramTargetFrame;
        std::string paramSourceFrame;       

        // Variables
        bool   ongoingTakeOff;
        bool   onMission;
        bool   timeCheck;
        bool   modeAuto;
        double currentAltitude;
        double desiredAltitude;
        int    timeSec;
        geometry_msgs::PoseStamped currentPosition;

        // Functions
        void cbTakeOff(const std_msgs::EmptyConstPtr& msg);
        void cbLand(const std_msgs::EmptyConstPtr& msg);
        void cbDisarm(const std_msgs::EmptyConstPtr& msg);
        void cbMoveVel(const geometry_msgs::TwistConstPtr& msg);
        void cbConvertCommand(const geometry_msgs::TwistConstPtr& msg, int command_origin);
        void cbConvertXBoxCommand(const geometry_msgs::TwistConstPtr& msg);
        void cbConvertLandCommand(const geometry_msgs::TwistConstPtr& msg);
        void cbOdomUAV(const geometry_msgs::PoseStampedConstPtr& msg);
        void cbConvertSIMCommand(const geometry_msgs::TwistConstPtr& msg);
        void cbMissionStart(const std_msgs::EmptyConstPtr& msg);
        void cbMissionStop(const std_msgs::EmptyConstPtr& msg);
        void changeToAutoMode();
        void changeToGuidedMode();
        void cbRTL(const std_msgs::EmptyConstPtr& msg);
        void cbAuto(const std_msgs::EmptyConstPtr& msg);
        void cbSetPointConverter(const geographic_msgs::GeoPose& msg);
    };

}

#endif
