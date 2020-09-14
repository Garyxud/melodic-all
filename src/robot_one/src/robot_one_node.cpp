#include "ros/ros.h"

#include <signal.h>
#include <cstring>

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float32.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float32MultiArray.h"

#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/fill_image.h"

#include "robot_one/Get.h"
#include "robot_one/Set.h"

#include <tf/transform_broadcaster.h>

// From https://github.com/AlexanderSilvaB/Robot-One
#include "robotOne.h"

#define BASE_Z 0.86
#define SENSORS_X 0.35
#define LASER_Z 0.5
#define CAMERA_Z 0.7

// Robot One Handler
int handler = 0;
float waitTime = 0;
LidarData lidarData;
CameraData cameraData;
Value2 value2;
Value3 value3;

sensor_msgs::LaserScan lidarMsg;
sensor_msgs::Image cameraMsg;

void robotOneSigintHandler(int sig)
{
    ROS_INFO("Finishing...");
    disconnectRobotOne();
    ROS_INFO("Finished");

    ros::shutdown();
}

// Callbacks
void connectCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("Trying to connect to: [%s]", msg->data.c_str());
    if(handler)
    {
        ROS_INFO("Disconnecting: %d", handler);
        disconnectRobotOne();
    }
    handler = connectRobotOne(msg->data.c_str());
    ROS_INFO("Connected: [%s]", handler > 0 ? "yes" : "no");
}

void disconnectCallback(const std_msgs::Empty::ConstPtr& msg)
{
    if(handler)
    {
        ROS_INFO("Disconnecting: %d", handler);
        disconnectRobotOne();
        handler = 0;
    }
    else
        ROS_INFO("Already disconnected");
}

void poseCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    value3.values[0] = msg->x;
    value3.values[1] = msg->y;
    value3.values[2] = msg->theta;
    ROS_INFO("Setting pose = [%f, %f, %f]", value3.values[0], value3.values[1], value3.values[2]);
    setPose(&value3);
}

void poseXCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting pose X = %f", msg->data);
    getPose(&value3);
    value3.values[0] = msg->data;
    setPose(&value3);
}

void poseYCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting pose Y = %f", msg->data);
    getPose(&value3);
    value3.values[1] = msg->data;
    setPose(&value3);
}

void poseThetaCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting pose Theta = %f", msg->data);
    getPose(&value3);
    value3.values[2] = msg->data;
    setPose(&value3);
}

void odometryCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    value3.values[0] = msg->x;
    value3.values[1] = msg->y;
    value3.values[2] = msg->theta;
    ROS_INFO("Setting odometry = [%f, %f, %f]", value3.values[0], value3.values[1], value3.values[2]);
    setOdometry(&value3);
}

void odometryXCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting odometry X = %f", msg->data);
    getOdometry(&value3);
    value3.values[0] = msg->data;
    setOdometry(&value3);
}

void odometryYCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting odometry Y = %f", msg->data);
    getOdometry(&value3);
    value3.values[1] = msg->data;
    setOdometry(&value3);
}

void odometryThetaCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting odometry Theta = %f", msg->data);
    getOdometry(&value3);
    value3.values[2] = msg->data;
    setOdometry(&value3);
}

void odometryStdCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    value2.values[0] = msg->x;
    value2.values[1] = msg->y;
    ROS_INFO("Setting odometryStd = [%f, %f]", value2.values[0], value2.values[1]);
    setOdometryStd(&value2);
}

void odometryStdLinearCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting odometryStd linear = %f", msg->data);
    getOdometryStd(&value2);
    value2.values[0] = msg->data;
    setOdometryStd(&value2);
}

void odometryStdAngularCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting odometryStd angular = %f", msg->data);
    getOdometryStd(&value2);
    value2.values[1] = msg->data;
    setOdometryStd(&value2);
}

void gpsStdCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    value3.values[0] = msg->x;
    value3.values[1] = msg->y;
    value3.values[2] = msg->theta;
    ROS_INFO("Setting gpsStd = [%f, %f, %f]", value3.values[0], value3.values[1], value3.values[2]);
    getGPSStd(&value3);
}

void gpsStdXCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting gpsStd X = %f", msg->data);
    getGPSStd(&value3);
    value3.values[0] = msg->data;
    getGPSStd(&value3);
}

void gpsStdYCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting gpsStd Y = %f", msg->data);
    getGPSStd(&value3);
    value3.values[1] = msg->data;
    getGPSStd(&value3);
}

void gpsStdThetaCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting gpsStd Theta = %f", msg->data);
    getGPSStd(&value3);
    value3.values[2] = msg->data;
    getGPSStd(&value3);
}

void velocityCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    value2.values[0] = msg->x;
    value2.values[1] = msg->y;
    ROS_INFO("Setting velocity = [%f, %f]", value2.values[0], value2.values[1]);
    setVelocity(&value2);
}

void velocityLinearCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting velocity linear = %f", msg->data);
    getVelocity(&value2);
    value2.values[0] = msg->data;
    setVelocity(&value2);
}

void velocityAngularCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting velocity angular = %f", msg->data);
    getVelocity(&value2);
    value2.values[1] = msg->data;
    setVelocity(&value2);
}

void wheelsCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    value2.values[0] = msg->x;
    value2.values[1] = msg->y;
    ROS_INFO("Setting wheels velocity = [%f, %f]", value2.values[0], value2.values[1]);
    setWheels(&value2);
}

void wheelsLeftCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting wheels velocity linear = %f", msg->data);
    getWheels(&value2);
    value2.values[0] = msg->data;
    setWheels(&value2);
}

void wheelsRightCallback(const std_msgs::Float32::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }

    ROS_INFO("Setting wheels velocity angular = %f", msg->data);
    getWheels(&value2);
    value2.values[1] = msg->data;
    setWheels(&value2);
}

void lowLevelControlCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }
    ROS_INFO("Setting low level control = %d", (int)msg->data);
    setLowLevelControl(msg->data);
}

void manualControllerCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }
    ROS_INFO("Setting manual controller = %d", (int)msg->data);
    setManualController(msg->data);
}


void traceCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(!handler)
    {
        ROS_INFO("Already disconnected");
        return;
    }
    ROS_INFO("Setting trace = %d", (int)msg->data);
    setTrace(msg->data);
}

// Service callbacks
bool getCallback(robot_one::Get::Request& req, robot_one::Get::Response& res)
{
    if(handler)
    {
        ROS_INFO("Getting: %s", req.name.c_str());
        res.value = get(req.name.c_str());
        return true;
    }
    else
        ROS_INFO("Disconnected");
    return false;
}

bool setCallback(robot_one::Set::Request& req, robot_one::Set::Response& res)
{
    if(handler)
    {
        ROS_INFO("Setting: %s = %f", req.name.c_str(), req.value);
        res.value = set(req.name.c_str(), req.value);
        return true;
    }
    else
        ROS_INFO("Disconnected");
    return false;
}

// Publishers
void publishConnected(ros::Publisher& pub)
{
    std_msgs::Bool msg;
    msg.data = handler > 0;
    pub.publish(msg);
}

void publishVersion(ros::Publisher& pub)
{
    std_msgs::Float32 msg;
    msg.data = versionRobotOne();
    pub.publish(msg);
}


void publishPose(tf::TransformBroadcaster& broadcaster, ros::Time& t, ros::Publisher& pub, ros::Publisher& pubX, ros::Publisher& pubY, ros::Publisher& pubTheta)
{
    if(!handler)
        return;

    getPose(&value3);

    geometry_msgs::Pose2D pose;
    pose.x = value3.values[0];
    pose.y = value3.values[1];
    pose.theta = value3.values[2];
    pub.publish(pose);

    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose.x, pose.y, BASE_Z) );
    tf::Quaternion q;
    q.setRPY(0, 0, pose.theta);
    transform.setRotation(q);
    broadcaster.sendTransform(tf::StampedTransform(transform, t, "world", "base_link"));

    std_msgs::Float32 x;
    x.data = value3.values[0];
    pubX.publish(x);

    std_msgs::Float32 y;
    y.data = value3.values[1];
    pubY.publish(y);

    std_msgs::Float32 theta;
    theta.data = value3.values[2];
    pubTheta.publish(theta);
}

void publishOdometry(ros::Publisher& pub, ros::Publisher& pubX, ros::Publisher& pubY, ros::Publisher& pubTheta)
{
    if(!handler)
        return;

    getOdometry(&value3);

    geometry_msgs::Pose2D pose;
    pose.x = value3.values[0];
    pose.y = value3.values[1];
    pose.theta = value3.values[2];
    pub.publish(pose);

    std_msgs::Float32 x;
    x.data = value3.values[0];
    pubX.publish(x);

    std_msgs::Float32 y;
    y.data = value3.values[1];
    pubY.publish(y);

    std_msgs::Float32 theta;
    theta.data = value3.values[2];
    pubTheta.publish(theta);
}

void publishOdometryStd(ros::Publisher& pub, ros::Publisher& pubLinear, ros::Publisher& pubAngular)
{
    if(!handler)
        return;

    getOdometryStd(&value2);

    geometry_msgs::Point point;
    point.x = value2.values[0];
    point.y = value2.values[1];
    pub.publish(point);

    std_msgs::Float32 x;
    x.data = value2.values[0];
    pubLinear.publish(x);

    std_msgs::Float32 theta;
    theta.data = value2.values[1];
    pubAngular.publish(theta);
}

void publishGPS(ros::Publisher& pub, ros::Publisher& pubX, ros::Publisher& pubY, ros::Publisher& pubTheta)
{
    if(!handler)
        return;

    getGPS(&value3);

    geometry_msgs::Pose2D pose;
    pose.x = value3.values[0];
    pose.y = value3.values[1];
    pose.theta = value3.values[2];
    pub.publish(pose);

    std_msgs::Float32 x;
    x.data = value3.values[0];
    pubX.publish(x);

    std_msgs::Float32 y;
    y.data = value3.values[1];
    pubY.publish(y);

    std_msgs::Float32 theta;
    theta.data = value3.values[2];
    pubTheta.publish(theta);
}

void publishGPSStd(ros::Publisher& pub, ros::Publisher& pubX, ros::Publisher& pubY, ros::Publisher& pubTheta)
{
    if(!handler)
        return;

    getGPSStd(&value3);

    geometry_msgs::Pose2D pose;
    pose.x = value3.values[0];
    pose.y = value3.values[1];
    pose.theta = value3.values[2];
    pub.publish(pose);

    std_msgs::Float32 x;
    x.data = value3.values[0];
    pubX.publish(x);

    std_msgs::Float32 y;
    y.data = value3.values[1];
    pubY.publish(y);

    std_msgs::Float32 theta;
    theta.data = value3.values[2];
    pubTheta.publish(theta);
}

void publishVelocity(ros::Publisher& pub, ros::Publisher& pubLinear, ros::Publisher& pubAngular)
{
    if(!handler)
        return;

    getVelocity(&value2);

    geometry_msgs::Point point;
    point.x = value2.values[0];
    point.y = value2.values[1];
    pub.publish(point);

    std_msgs::Float32 x;
    x.data = value2.values[0];
    pubLinear.publish(x);

    std_msgs::Float32 theta;
    theta.data = value2.values[1];
    pubAngular.publish(theta);
}

void publishWheels(ros::Publisher& pub, ros::Publisher& pubLeft, ros::Publisher& pubRight)
{
    if(!handler)
        return;

    getWheels(&value2);

    geometry_msgs::Point point;
    point.x = value2.values[0];
    point.y = value2.values[1];
    pub.publish(point);

    std_msgs::Float32 x;
    x.data = value2.values[0];
    pubLeft.publish(x);

    std_msgs::Float32 theta;
    theta.data = value2.values[1];
    pubRight.publish(theta);
}

void publishLowLevelControl(ros::Publisher& pub)
{
    if(!handler)
        return;

    std_msgs::Bool msg;
    msg.data = getLowLevelControl();
    pub.publish(msg);
}

void publishManualController(ros::Publisher& pub)
{
    if(!handler)
        return;
        
    std_msgs::Bool msg;
    msg.data = getManualController();
    pub.publish(msg);
}

void publishTrace(ros::Publisher& pub)
{
    if(!handler)
        return;
        
    std_msgs::Bool msg;
    msg.data = getTrace();
    pub.publish(msg);
}

void publishWait(ros::Publisher& pub)
{
    if(!handler)
        return;
        
    std_msgs::Float32 msg;
    msg.data = waitTime;
    pub.publish(msg);
}

void publishCamera(ros::Time& t, ros::Publisher& pub)
{
    if(!handler)
        return;

    if(cameraData.size != 320*240*3)
        return;
        
    cameraMsg.header.stamp = t;
    cameraMsg.header.frame_id = "base_camera";
    sensor_msgs::fillImage(cameraMsg, 
                            sensor_msgs::image_encodings::RGB8, 
                            cameraData.height, 
                            cameraData.width, 
                            cameraData.width * cameraData.channels, 
                            cameraData.data);

    pub.publish(cameraMsg);
}

void publishLidar(ros::Time& t, ros::Publisher& pub)
{
    if(!handler)
        return;
        
    if(lidarData.size <= 0)
        return;

    lidarMsg.header.stamp = t;
    lidarMsg.header.frame_id = "base_laser";
    lidarMsg.angle_min = -1.57;
    lidarMsg.angle_max = 1.57;
    lidarMsg.angle_increment = 3.14 / lidarData.size;
    lidarMsg.time_increment = (1.0 / 30.0) / (lidarData.size);
    lidarMsg.range_min = 0.0;
    lidarMsg.range_max = 8.1;
    lidarMsg.ranges.resize(lidarData.size);
    lidarMsg.intensities.resize(lidarData.size);
    
    unsigned int start = lidarData.size - 1;
    for(unsigned int i = 0; i < lidarData.size; i++)
    {
        lidarMsg.ranges[start - i] = lidarData.readings[i];
        lidarMsg.intensities[i] = lidarData.readings[i];;
    }

    
    pub.publish(lidarMsg);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotOne", ros::init_options::NoSigintHandler);
    ROS_INFO("Starting...");
    
    ros::NodeHandle n;
    signal(SIGINT, robotOneSigintHandler);

    ROS_INFO("Reading parameters...");
    std::string address;
    bool autoConnect;
    n.param<std::string>("address", address, "127.0.0.1");
    n.param<bool>("autoConnect", autoConnect, true);

    ROS_INFO("Registering services");
    ros::ServiceServer srvSet = n.advertiseService("/robotOne/set", setCallback);
    ros::ServiceServer srvGet = n.advertiseService("/robotOne/get", getCallback);

    ROS_INFO("Registering topics...");
    // Subscribers
    ros::Subscriber subConnect = n.subscribe("/robotOne/connect", 1, connectCallback);
    ros::Subscriber subDisconnect = n.subscribe("/robotOne/disconnect", 1, disconnectCallback);

    ros::Subscriber subPose = n.subscribe("/robotOne/set/pose", 1, poseCallback);
    ros::Subscriber subPoseX = n.subscribe("/robotOne/set/pose/x", 1, poseXCallback);
    ros::Subscriber subPoseY = n.subscribe("/robotOne/set/pose/y", 1, poseYCallback);
    ros::Subscriber subPoseTheta = n.subscribe("/robotOne/set/pose/theta", 1, poseThetaCallback);

    ros::Subscriber subOdometry = n.subscribe("/robotOne/set/odometry", 1, odometryCallback);
    ros::Subscriber subOdometryX = n.subscribe("/robotOne/set/odometry/x", 1, odometryXCallback);
    ros::Subscriber subOdometryY = n.subscribe("/robotOne/set/odometry/y", 1, odometryYCallback);
    ros::Subscriber subOdometryTheta = n.subscribe("/robotOne/set/odometry/theta", 1, odometryThetaCallback);

    ros::Subscriber subOdometryStd = n.subscribe("/robotOne/set/odometry/std", 1, odometryStdCallback);
    ros::Subscriber subOdometryStdLinear = n.subscribe("/robotOne/set/odometry/std/linear", 1, odometryStdLinearCallback);
    ros::Subscriber subOdometryStdAngular = n.subscribe("/robotOne/set/odometry/std/angular", 1, odometryStdAngularCallback);

    ros::Subscriber subGPSStd = n.subscribe("/robotOne/set/gps/std", 1, gpsStdCallback);
    ros::Subscriber subGPSStdX = n.subscribe("/robotOne/set/gps/std/x", 1, gpsStdXCallback);
    ros::Subscriber subGPSStdY = n.subscribe("/robotOne/set/gps/std/y", 1, gpsStdYCallback);
    ros::Subscriber subGPSStdTheta = n.subscribe("/robotOne/set/gps/std/theta", 1, gpsStdThetaCallback);

    ros::Subscriber subVelocity = n.subscribe("/robotOne/set/velocity", 1, velocityCallback);
    ros::Subscriber subVelocityLinear = n.subscribe("/robotOne/set/velocity/linear", 1, velocityLinearCallback);
    ros::Subscriber subVelocityAngular = n.subscribe("/robotOne/set/velocity/angular", 1, velocityAngularCallback);

    ros::Subscriber subWheels = n.subscribe("/robotOne/set/velocity/wheels", 1, wheelsCallback);
    ros::Subscriber subWheelsLeft = n.subscribe("/robotOne/set/velocity/wheels/left", 1, wheelsLeftCallback);
    ros::Subscriber subWheelsRight = n.subscribe("/robotOne/set/velocity/wheels/right", 1, wheelsRightCallback);

    ros::Subscriber subLowLevelControl = n.subscribe("/robotOne/set/controller/lowlevel", 1, lowLevelControlCallback);
    ros::Subscriber subManualController = n.subscribe("/robotOne/set/controller/manual", 1, manualControllerCallback);

    ros::Subscriber subTrace = n.subscribe("/robotOne/set/trace", 1, traceCallback);

    // Publishers
    ros::Publisher pubConnected = n.advertise<std_msgs::Bool>("/robotOne/connected", 1);
    ros::Publisher pubVersion = n.advertise<std_msgs::Float32>("/robotOne/version", 1);

    ros::Publisher pubPose = n.advertise<geometry_msgs::Pose2D>("/robotOne/get/pose", 1);
    ros::Publisher pubPoseX = n.advertise<std_msgs::Float32>("/robotOne/get/pose/x", 1);
    ros::Publisher pubPoseY = n.advertise<std_msgs::Float32>("/robotOne/get/pose/y", 1);
    ros::Publisher pubPoseTheta = n.advertise<std_msgs::Float32>("/robotOne/get/pose/theta", 1);

    ros::Publisher pubOdometry = n.advertise<geometry_msgs::Pose2D>("/robotOne/get/odometry", 1);
    ros::Publisher pubOdometryX = n.advertise<std_msgs::Float32>("/robotOne/get/odometry/x", 1);
    ros::Publisher pubOdometryY = n.advertise<std_msgs::Float32>("/robotOne/get/odometry/y", 1);
    ros::Publisher pubOdometryTheta = n.advertise<std_msgs::Float32>("/robotOne/get/odometry/theta", 1);

    ros::Publisher pubOdometryStd = n.advertise<geometry_msgs::Point>("/robotOne/get/odometry/std", 1);
    ros::Publisher pubOdometryStdLinear = n.advertise<std_msgs::Float32>("/robotOne/get/odometry/std/linear", 1);
    ros::Publisher pubOdometryStdAngular = n.advertise<std_msgs::Float32>("/robotOne/get/odometry/std/angular", 1);

    ros::Publisher pubGPS = n.advertise<geometry_msgs::Pose2D>("/robotOne/get/gps", 1);
    ros::Publisher pubGPSX = n.advertise<std_msgs::Float32>("/robotOne/get/gps/x", 1);
    ros::Publisher pubGPSY = n.advertise<std_msgs::Float32>("/robotOne/get/gps/y", 1);
    ros::Publisher pubGPSTheta = n.advertise<std_msgs::Float32>("/robotOne/get/gps/theta", 1);

    ros::Publisher pubGPSStd = n.advertise<geometry_msgs::Pose2D>("/robotOne/get/gps/std", 1);
    ros::Publisher pubGPSStdX = n.advertise<std_msgs::Float32>("/robotOne/get/gps/std/x", 1);
    ros::Publisher pubGPSStdY = n.advertise<std_msgs::Float32>("/robotOne/get/gps/std/y", 1);
    ros::Publisher pubGPSStdTheta = n.advertise<std_msgs::Float32>("/robotOne/get/gps/std/theta", 1);

    ros::Publisher pubVelocity = n.advertise<geometry_msgs::Point>("/robotOne/get/velocity", 1);
    ros::Publisher pubVelocityLinear = n.advertise<std_msgs::Float32>("/robotOne/get/velocity/linear", 1);
    ros::Publisher pubVelocityAngular = n.advertise<std_msgs::Float32>("/robotOne/get/velocity/angular", 1);

    ros::Publisher pubWheels = n.advertise<geometry_msgs::Point>("/robotOne/get/velocity/wheels", 1);
    ros::Publisher pubWheelsLeft = n.advertise<std_msgs::Float32>("/robotOne/get/velocity/wheels/left", 1);
    ros::Publisher pubWheelsRight = n.advertise<std_msgs::Float32>("/robotOne/get/velocity/wheels/right", 1);

    ros::Publisher pubLowLevelControl = n.advertise<std_msgs::Bool>("/robotOne/get/controller/lowlevel", 1);
    ros::Publisher pubManualController = n.advertise<std_msgs::Bool>("/robotOne/get/controller/manual", 1);

    ros::Publisher pubTrace = n.advertise<std_msgs::Bool>("/robotOne/get/trace", 1);
    ros::Publisher pubWait = n.advertise<std_msgs::Float32>("/robotOne/wait", 1);

    ros::Publisher pubCamera = n.advertise<sensor_msgs::Image>("/robotOne/get/camera", 1);
    ros::Publisher pubLidar = n.advertise<sensor_msgs::LaserScan>("/robotOne/get/laser", 1);
    

    // Init
    memset(&lidarData, '\0', sizeof(LidarData));
    memset(&cameraData, '\0', sizeof(CameraData));
    initLidar(&lidarData, 180);
    initCamera(&cameraData);

    ros::Rate loop_rate(30);
    tf::TransformBroadcaster broadcaster;

    if(autoConnect)
    {
        boost::shared_ptr<std_msgs::String> connectMsg(new std_msgs::String());
        connectMsg->data = address;
        connectCallback(connectMsg);
    }

    ROS_INFO("Running");
    while (ros::ok())
    {
        ros::Time readTime = ros::Time::now();

        readLidar(&lidarData);
        captureCamera(&cameraData);

        // Lidar transform
        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(SENSORS_X, 0.0, LASER_Z)),
                readTime, "base_link", "base_laser"));

        broadcaster.sendTransform(
            tf::StampedTransform(
                tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(SENSORS_X, 0.0, CAMERA_Z)),
                readTime, "base_link", "base_camera"));

        publishVersion(pubVersion);
        publishConnected(pubConnected);
        publishPose(broadcaster, readTime, pubPose, pubPoseX, pubPoseY, pubPoseTheta);
        publishOdometry(pubOdometry, pubOdometryX, pubOdometryY, pubOdometryTheta);
        publishOdometryStd(pubOdometryStd, pubOdometryStdLinear, pubOdometryStdAngular);
        publishGPS(pubGPS, pubGPSX, pubGPSY, pubGPSTheta);
        publishGPSStd(pubGPSStd, pubGPSStdX, pubGPSStdY, pubGPSStdTheta);
        publishVelocity(pubVelocity, pubVelocityLinear,pubVelocityAngular);
        publishWheels(pubWheels, pubWheelsLeft, pubWheelsRight);
        publishLowLevelControl(pubLowLevelControl);
        publishManualController(pubManualController);
        publishTrace(pubTrace);
        publishWait(pubWait);
        publishCamera(readTime, pubCamera);
        publishLidar(readTime, pubLidar);
        

        if(handler)
            waitTime = waitRobotOne();
        else
            waitTime = 0;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}