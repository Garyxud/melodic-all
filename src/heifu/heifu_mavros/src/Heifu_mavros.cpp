#include "heifu_mavros/Heifu_mavros.hpp"

using namespace HM;

Heifu_mavros::Heifu_mavros():n("~") {
    std::string ns = ros::this_node::getNamespace();

    // Subscribers
    subXBoxTakeOff           = n.subscribe(ns + "/takeoff", 10, &Heifu_mavros::cbTakeOff, this);
    subReturnHome            = n.subscribe(ns + "/rtl", 10, &Heifu_mavros::cbRTL, this);
    subXBoxLand              = n.subscribe(ns + "/land", 10, &Heifu_mavros::cbLand, this);
    subXBoxDisarm            = n.subscribe(ns + "/disarm", 10, &Heifu_mavros::cbDisarm, this);
    subXBoxMoveVel           = n.subscribe(ns + "/cmd_vel", 10, &Heifu_mavros::cbMoveVel, this);
    subXBoxRawCommand        = n.subscribe(ns + "/xbox_vel_raw", 10, &Heifu_mavros::cbConvertXBoxCommand, this);
    subLandRawCommand        = n.subscribe(ns + "/land_vel_raw", 10, &Heifu_mavros::cbConvertLandCommand, this);
    subSimCommand            = n.subscribe(ns + "/simulation_raw", 10, &Heifu_mavros::cbConvertSIMCommand, this);
    subXBoxRawCommandFront   = n.subscribe(ns + "/frontend/cmd", 10, &Heifu_mavros::cbConvertXBoxCommand, this);
	subAuto		             = n.subscribe(ns + "/mode_auto", 10, &Heifu_mavros::cbAuto, this);
    subSetPointConverter     = n.subscribe(ns + "/global_setpoint_converter", 10, &Heifu_mavros::cbSetPointConverter, this);
    subOdomUAV               = n.subscribe(ns + "/mavros/local_position/pose", 1, &Heifu_mavros::cbOdomUAV, this);
    subMissionStart          = n.subscribe(ns + "/mission/start", 10, &Heifu_mavros::cbMissionStart, this);
    subMissionStop           = n.subscribe(ns + "/mission/stop", 10, &Heifu_mavros::cbMissionStop, this);

    // Service Clients
    srvClientArming    = n.serviceClient < mavros_msgs::CommandBool > (ns + "/mavros/cmd/arming");
    srvClientSetMode   = n.serviceClient < mavros_msgs::SetMode > (ns + "/mavros/set_mode");
    srvClientTakeOff   = n.serviceClient < mavros_msgs::CommandTOL > (ns + "/mavros/cmd/takeoff");

    // Publishers
    pubMoveVel          = n.advertise < geometry_msgs::TwistStamped > (ns + "/mavros/setpoint_velocity/cmd_vel", 10);
    pubXBoxBodyCommand  = n.advertise < geometry_msgs::Twist > (ns + "/xbox_vel", 10);
    pubLandBodyCommand  = n.advertise < geometry_msgs::Twist > (ns + "/land_vel", 10);
    pubSimCommand       = n.advertise < geometry_msgs::Twist > (ns + "/simulation", 10);
    pubGpsFixState      = n.advertise < std_msgs::Int8 > (ns + "/g/f/m", 10);
    pubDesiredPosition  = n.advertise < geometry_msgs::PoseStamped > (ns + "/mavros/setpoint_position/local", 1);
    pubSetPointConverter = n.advertise < geometry_msgs::Pose > (ns + "/GNSS_utils/goal_coordinates", 1);
    pubTakeoffDiagnostic = n.advertise <std_msgs::Bool> (ns + "/diagnostic/takeoff",1);
    pubLandDiagnostic = n.advertise <std_msgs::Bool> (ns + "/diagnostic/land",1);

    // Variables
    ongoingTakeOff  = false;
    currentAltitude = desiredAltitude = 0.0;
    onMission = false;
    timeCheck = true;
	modeAuto = false;

    // Parameters
    n.param<double>("paramTakeOffAltitude",  paramTakeOffAltitude, 10.0);
    n.param<double>("paramTakeOffThreshold", paramTakeOffThreshold, 0.3);
    n.param<double>("paramNodeRate",         paramNodeRate,  100.0);

    n.param<std::string>("paramTargetFrame", paramTargetFrame, "base_link");
    n.param<std::string>("paramSourceFrame", paramSourceFrame, "odom");

    ros_utils::ROS_PRINT(ros_utils::WHITE_BOLD, "Mavros Node Ready");
}


void Heifu_mavros::run(){
    
    ros::Rate go(paramNodeRate);
    while (ros::ok()){

        if(ongoingTakeOff)
        {
            ROS_INFO_THROTTLE(1.0, "UAV Doing TakeOff!");
            ongoingTakeOff = (currentAltitude < desiredAltitude - paramTakeOffThreshold);
            if(!ongoingTakeOff) {
                ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "Takeoff Position Reached!");

				if (modeAuto) {
					changeToAutoMode();
				}
			}
        }
        else {
            if (onMission == false)
            {
                pubMoveVel.publish(msgVel);
            }
			
			if (modeAuto) {
				changeToAutoMode();
			}
		}
        
        ros::spinOnce();
        go.sleep();
    }
}

void Heifu_mavros::cbTakeOff(const std_msgs::EmptyConstPtr& msg){

    mavros_msgs::SetMode heifuSetMode;
    heifuSetMode.request.base_mode   = 0;
    heifuSetMode.request.custom_mode = "GUIDED";

    if (srvClientSetMode.call(heifuSetMode) && heifuSetMode.response.mode_sent)
        ROS_INFO("GUIDED enabled");
    else
        ROS_ERROR("Failed to set GUIDED");

    mavros_msgs::CommandBool heifuArming;
    heifuArming.request.value = true;

    if (srvClientArming.call(heifuArming) && heifuArming.response.success)
        ROS_INFO("Vehicle armed");
    else
        ROS_ERROR("Arming failed");

    mavros_msgs::CommandTOL heifuTakeoff;
    heifuTakeoff.request.latitude   = 0.0;
    heifuTakeoff.request.longitude  = 0.0;
    heifuTakeoff.request.altitude   = static_cast<float>(paramTakeOffAltitude);

    std_msgs::Bool takeoffResult;
    if (srvClientTakeOff.call(heifuTakeoff) && heifuTakeoff.response.success)
    {
        ROS_INFO("Takeoff success");
        ongoingTakeOff = true;
        desiredAltitude = paramTakeOffAltitude;
        takeoffResult.data = true;
        pubTakeoffDiagnostic.publish(takeoffResult);
    }
    else {
        ROS_ERROR("Takeoff failed");
        takeoffResult.data = false;
        pubTakeoffDiagnostic.publish(takeoffResult);
    }
}

void Heifu_mavros::cbLand(const std_msgs::EmptyConstPtr& msg){

    mavros_msgs::SetMode heifuSetMode;
    heifuSetMode.request.base_mode   = 0;
    heifuSetMode.request.custom_mode = "LAND";

    std_msgs::Bool landResult;
    if (srvClientSetMode.call(heifuSetMode) && heifuSetMode.response.mode_sent) {
        ROS_INFO("Landing Vehicle");
        landResult.data = true;
        pubLandDiagnostic.publish(landResult);
    }
    else {
        ROS_ERROR("Failed to Land");
        landResult.data = false;
        pubLandDiagnostic.publish(landResult);
    }

}

void Heifu_mavros::cbDisarm(const std_msgs::EmptyConstPtr& msg){

    mavros_msgs::CommandBool heifuArming;
    heifuArming.request.value = false;

    if (srvClientArming.call(heifuArming) && heifuArming.response.success)
        ROS_INFO("Vehicle disarm");
    else
        ROS_ERROR("Disarm failed");
}

void Heifu_mavros::cbMoveVel(const geometry_msgs::TwistConstPtr& msg){
    msgVel.header.stamp     = ros::Time::now();
    msgVel.header.frame_id  = paramSourceFrame;
    msgVel.twist            = *msg;
}

void Heifu_mavros::cbConvertXBoxCommand(const geometry_msgs::TwistConstPtr &msg)
{
	cbConvertCommand(msg, XBOX_CMD);
}

void Heifu_mavros::cbConvertLandCommand(const geometry_msgs::TwistConstPtr &msg)
{
	cbConvertCommand(msg, LAND_CMD);
}

void Heifu_mavros::cbConvertSIMCommand(const geometry_msgs::TwistConstPtr &msg)
{
	cbConvertCommand(msg, SIM_CMD);
}

void Heifu_mavros::cbConvertCommand(const geometry_msgs::TwistConstPtr &msg, int command_origin)
{
    tf::Quaternion q;
    tf::quaternionMsgToTF(currentPosition.pose.orientation, q);
    geometry_msgs::Twist msgVelBody;

    double angle = tf::getYaw(q);
    msgVelBody.angular.z  = msg->angular.z;
    msgVelBody.linear.z   = msg->linear.z;
    msgVelBody.linear.x   = msg->linear.x * cos(angle) - msg->linear.y * sin(angle);
    msgVelBody.linear.y   = msg->linear.x * sin(angle) + msg->linear.y * cos(angle);
    
	switch(command_origin) {
    	case XBOX_CMD: pubXBoxBodyCommand.publish(msgVelBody); break;
		case LAND_CMD: pubLandBodyCommand.publish(msgVelBody); break;
        case SIM_CMD: pubSimCommand.publish(msgVelBody); break;
		default: ROS_ERROR("Command origin not correct!");
	}
}

void Heifu_mavros::cbOdomUAV(const geometry_msgs::PoseStampedConstPtr &msg)
{
    currentAltitude = msg->pose.position.z;
    currentPosition = *msg;
    tf::Quaternion q;
    tf::quaternionMsgToTF(currentPosition.pose.orientation, q);
    geometry_msgs::Twist msgVelBody;
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(currentPosition.pose.position.x, currentPosition.pose.position.y, currentPosition.pose.position.z));
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/odom", "/base_link"));
}

void Heifu_mavros::cbMissionStart(const std_msgs::EmptyConstPtr& msg){
    changeToGuidedMode();
    onMission = true;
}

void Heifu_mavros::cbMissionStop(const std_msgs::EmptyConstPtr& msg){
    changeToGuidedMode();
    onMission = false;
}

void Heifu_mavros::cbRTL(const std_msgs::EmptyConstPtr& msg){
    mavros_msgs::SetMode heifuSetModeRTL;
    heifuSetModeRTL.request.base_mode   = 0;
    heifuSetModeRTL.request.custom_mode = "RTL";

    if (srvClientSetMode.call(heifuSetModeRTL) && heifuSetModeRTL.response.mode_sent)
        ROS_INFO("RTL enabled");
    else
        ROS_ERROR("Failed to set RTL");

}

void Heifu_mavros::cbAuto(const std_msgs::EmptyConstPtr& msg){
	modeAuto = true;
	//ros_utils::ROS_PRINT(ros_utils::GREEN_BOLD, "auto true");
}

void Heifu_mavros::cbSetPointConverter(const geographic_msgs::GeoPose& msg){
	geometry_msgs::Pose globalSetpoint;
    globalSetpoint.position.x = msg.position.latitude;
    globalSetpoint.position.y = msg.position.longitude;
    globalSetpoint.position.z = msg.position.altitude;
    globalSetpoint.orientation.x = 0;
    globalSetpoint.orientation.y = 0;
    globalSetpoint.orientation.z = 0;
    globalSetpoint.orientation.w = 0;

    pubSetPointConverter.publish(globalSetpoint);
}

void Heifu_mavros::changeToAutoMode(){
    mavros_msgs::SetMode heifuSetMode;
    heifuSetMode.request.base_mode   = 0;
    heifuSetMode.request.custom_mode = "AUTO";

    if (srvClientSetMode.call(heifuSetMode) && heifuSetMode.response.mode_sent) {
        ROS_INFO("AUTO enabled");
        modeAuto = false;
    }
    else
        ROS_ERROR("Failed to set AUTO");
}

void Heifu_mavros::changeToGuidedMode(){
    mavros_msgs::SetMode heifuSetMode;
    heifuSetMode.request.base_mode   = 0;
    heifuSetMode.request.custom_mode = "GUIDED";

    if (srvClientSetMode.call(heifuSetMode) && heifuSetMode.response.mode_sent)
        ROS_INFO("GUIDED enabled");
    else
        ROS_ERROR("Failed to set GUIDED");
}
