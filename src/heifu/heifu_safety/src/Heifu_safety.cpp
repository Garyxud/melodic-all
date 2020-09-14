#include "heifu_safety/Heifu_safety.hpp"

using namespace HS;

HeifuSafety::HeifuSafety():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subOdomUAV          = n.subscribe(ns + "/mavros/local_position/pose", 10, &HeifuSafety::cbOdomUAV, this);
    subJoystickVelocity = n.subscribe(ns + "/xbox_vel", 10, &HeifuSafety::cbJoystickVelocity, this);

    // Publishers
    pubSafeVelocity     = n.advertise<geometry_msgs::Twist>(ns + "/xbox_vel_safety", 10);

    // Service Servers
    srvEnableSafety     = n.advertiseService(ns + "/EnableSafetyFence", &HeifuSafety::enableSafety, this);

    // Variables
    enableSafetyFence = false;
    safetyDistance    = -1.0;
    safetyVelocity.linear.x  = safetyVelocity.linear.y  = safetyVelocity.linear.z  = 0;
    safetyVelocity.angular.x = safetyVelocity.angular.y = safetyVelocity.angular.z = 0;

    ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD, "Safety Fence Node Ready");
}

void HeifuSafety::run(){

    ros::Rate go(100);

    while (ros::ok()){
        ros::spinOnce();
        go.sleep();
    }
}

bool HeifuSafety::enableSafety(heifu_msgs::EnableSafety::Request &req, heifu_msgs::EnableSafety::Response &res)
{
    enableSafetyFence = req.enable;

    safetyDistance    = enableSafetyFence ? req.distance : -1.0;

    if((enableSafetyFence && (safetyDistance > 0.0)) || (!enableSafetyFence))
    {   
        res.success   = true;
        startPosition = currentUAVPosition;
    }
    else
    {
        res.success = false;
        ros_utils::ROS_PRINT(ros_utils::RED_NORMAL,"Please insert a valid safety distance (> 0).");

        enableSafetyFence = false;
        safetyDistance    = -1.0;
    }

    minPosition = maxPosition = startPosition;

    if(enableSafetyFence)
    {
        maxPosition.x += safetyDistance;
        maxPosition.y += safetyDistance;
        maxPosition.z += safetyDistance;

        minPosition.x -= static_cast<double>(safetyDistance);
        minPosition.y -= static_cast<double>(safetyDistance);
        minPosition.z -= (minPosition.z > static_cast<double>(safetyDistance) ? static_cast<double>(safetyDistance) : 0.0);
    }

    res.max_pos   = maxPosition;
    res.min_pos   = minPosition;
    res.start_pos = startPosition;

    if(res.success)
    {
        ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD,"SERVICE REQUEST RESULT: \033[32mSUCCESS");

        if(enableSafetyFence)
            ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD,"          SAFETY FENCE: \033[32mENABLED");
        else
            ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD,"          SAFETY FENCE: \033[31mDISABLED");
    }
    else
        ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD,"SERVICE REQUEST RESULT: \033[31mFAILED");
}

void HeifuSafety::cbOdomUAV(const geometry_msgs::PoseStampedConstPtr &msg)
{
    currentUAVPosition = msg->pose.position;
}

void HeifuSafety::cbJoystickVelocity(const geometry_msgs::Twist &ROSmsg)
{
    if(enableSafetyFence)
    {
        safetyVelocity  = ROSmsg;

        bool x_valid, y_valid, z_valid;

        x_valid =  (currentUAVPosition.x > minPosition.x) || (safetyVelocity.linear.x > 0.0);
        x_valid &= (currentUAVPosition.x < maxPosition.x) || (safetyVelocity.linear.x < 0.0);

        y_valid =  (currentUAVPosition.y > minPosition.y) || (safetyVelocity.linear.y > 0.0);
        y_valid &= (currentUAVPosition.y < maxPosition.y) || (safetyVelocity.linear.y < 0.0);

        z_valid =  (currentUAVPosition.z > minPosition.z) || (safetyVelocity.linear.z > 0.0);
        z_valid &= (currentUAVPosition.z < maxPosition.z) || (safetyVelocity.linear.z < 0.0);

        if(!x_valid)
            safetyVelocity.linear.x = 0.0;
        if(!y_valid)
            safetyVelocity.linear.y = 0.0;
        if(!z_valid)
            safetyVelocity.linear.z = 0.0;

        pubSafeVelocity.publish(safetyVelocity);
    }
}
