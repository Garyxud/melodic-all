#include "heifu_simple_waypoint/Heifu_simple_waypoint.hpp"

using namespace HSW;

Heifu_simple_waypoint::Heifu_simple_waypoint():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subPoseCommander    = n.subscribe(ns + "/mission/setpoint", 10, &Heifu_simple_waypoint::cbPoseCommander, this);
    subOdomUAV          = n.subscribe(ns + "/mavros/local_position/pose", 10, &Heifu_simple_waypoint::cbOdomUAV, this);

    // Publishers
    pubMissionStart     = n.advertise < std_msgs::Empty > (ns + "/mission/start", 10);
    pubMissionStop      = n.advertise < std_msgs::Empty > (ns + "/mission/stop", 10);
    pubDesiredPosition  = n.advertise < geometry_msgs::PoseStamped > (ns + "/mavros/setpoint_position/local", 10);

    // Parameters
    n.param<double>("paramNodeRate",         paramNodeRate,  100.0);
}


void Heifu_simple_waypoint::run(){
    
    ros::Rate go(paramNodeRate);
    while (ros::ok()){        
        ros::spinOnce();
        go.sleep();
    }
}


void Heifu_simple_waypoint::cbOdomUAV(const geometry_msgs::PoseStampedConstPtr &msg)
{
    currentPosition = *msg;
}

//RETIRAR DO MAVROS
void Heifu_simple_waypoint::cbPoseCommander(const geometry_msgs::Pose& msg){
    pubMissionStart.publish(emptyMsg);
	geometry_msgs::PoseStamped desiredPosition;
    ros::Rate go(paramNodeRate);

    double  distanceRelation, dx, dy, dz, desiredAngle, desiredAngleRaw, desiredW, yawValid, aCat, oCat, positionX, positionY, positionZ;
    
    tf::Quaternion angleQuaternion;
    //Calculate angle rotation to next waypoint
    aCat = msg.position.x - currentPosition.pose.position.x;
    oCat = msg.position.y - currentPosition.pose.position.y;
    desiredAngleRaw = atan(oCat/aCat);
    if (aCat < 0)
        desiredAngleRaw = desiredAngleRaw - M_PI;
                
    angleQuaternion.setRPY(0, 0, desiredAngleRaw);
    desiredAngle = angleQuaternion.getZ();
    desiredW = angleQuaternion.getW();
    desiredPosition.pose.position = currentPosition.pose.position;
    desiredPosition.pose.orientation.w = desiredW;
    desiredPosition.pose.orientation.z = desiredAngle;
    //Whaiting correct angle to goes to the next point
    do
    {
        ros::spinOnce();
        pubDesiredPosition.publish(desiredPosition);               
        yawValid = fabs(fabs(currentPosition.pose.orientation.z) - fabs(desiredPosition.pose.orientation.z));
        go.sleep();
        
    } while (yawValid >= 0.1);

    desiredPosition.pose.position.x = msg.position.x;
    desiredPosition.pose.position.y = msg.position.y;
    desiredPosition.pose.position.z = msg.position.z;

    //Start move the drone to position
    do
    {
        ros::spinOnce();
        desiredPosition.header.stamp    = ros::Time::now();
        pubDesiredPosition.publish(desiredPosition);               
        positionX = fabs(fabs(currentPosition.pose.position.x) - fabs(desiredPosition.pose.position.x));
        positionY = fabs(fabs(currentPosition.pose.position.y) - fabs(desiredPosition.pose.position.y));
        positionZ = fabs(fabs(currentPosition.pose.position.z) - fabs(desiredPosition.pose.position.z));
        go.sleep();
        
    } while (positionX >= 0.1 && positionY >= 0.1 && positionZ >= 0.1);

    pubMissionStop.publish(emptyMsg);
}
