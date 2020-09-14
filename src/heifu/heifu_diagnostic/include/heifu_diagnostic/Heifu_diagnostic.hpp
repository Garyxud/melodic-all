#ifndef HEIFU_DIAGNOSTIC_HPP
#define HEIFU_DIAGNOSTIC_HPP

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int8.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <ros_utils/Console.h>

namespace HD
{
    class Heifu_diagnostic
    {
    public:

        Heifu_diagnostic ();
        virtual ~Heifu_diagnostic (){};

        void run();

    private:
        ros::NodeHandle n;

        // Subscribers
        ros::Subscriber subDiagnostic;

        // Publishers
        ros::Publisher pubGpsFixState;

        // Parameters
        double paramNodeRate;

        // Variables
        std_msgs::Int8 gpsFixMode;

        // Const
        std::string str1 = "3D fix";
        std::string str2 = "Fix type";

        // Functions
        void cbCheckGpsFix(const diagnostic_msgs::DiagnosticArrayConstPtr& msg);
    };

}

#endif
