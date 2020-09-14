/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

#include <cob_hand_bridge/InitFinger.h>
#include <cob_hand_bridge/Status.h>

#include <std_srvs/Trigger.h>

#include <sensor_msgs/JointState.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <diagnostic_updater/publisher.h>

#include <actionlib/server/simple_action_server.h>

#include <ros/ros.h>

#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

#include <angles/angles.h>

boost::mutex g_mutex;

cob_hand_bridge::Status::ConstPtr g_status; 
boost::shared_ptr<diagnostic_updater::TimeStampStatus> g_topic_status;

ros::Publisher g_js_pub;
sensor_msgs::JointState g_js;

ros::ServiceServer g_init_srv;

ros::ServiceClient g_init_finger_client;
ros::ServiceClient g_halt_client;

ros::Publisher g_command_pub;
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> FollowJointTrajectoryActionServer;
boost::scoped_ptr<FollowJointTrajectoryActionServer> g_as;
control_msgs::FollowJointTrajectoryGoalConstPtr g_goal;
cob_hand_bridge::JointValues g_command;
cob_hand_bridge::JointValues g_default_command;
ros::Timer g_command_timer;
ros::Timer g_deadline_timer;
double g_stopped_velocity;
double g_stopped_current;
bool g_initialized;
bool g_motion_stopped;
bool g_control_stopped;
bool g_motors_moved;
std::vector<double> g_goal_tolerance;

bool isFingerReady_nolock() {
    return g_status && (g_status->status & (g_status->MASK_FINGER_READY | g_status->MASK_ERROR)) == g_status->MASK_FINGER_READY && g_status->rc == 0;
}

bool checkAction_nolock(bool deadline_exceeded){
    control_msgs::FollowJointTrajectoryResult result;
    if(g_as->isActive()){
        bool goal_reached = false;
        if(g_motion_stopped) {
            goal_reached = true;;
            for(size_t i = 0; i < g_status->joints.position_cdeg.size(); ++i){
                if(fabs(g_status->joints.position_cdeg[i]-g_command.position_cdeg[i]) > g_goal_tolerance[i]){
                    goal_reached = false;
                    result.error_code = result.GOAL_TOLERANCE_VIOLATED;
                    break;
                }
            }
        }
        if(!isFingerReady_nolock()) {
            g_deadline_timer.stop();
            g_as->setAborted();
        }else if(g_motion_stopped && (goal_reached || g_motors_moved)) {
            g_deadline_timer.stop();
            g_as->setSucceeded(result);
        }else if (deadline_exceeded) {
            g_deadline_timer.stop();
            result.error_code = result.GOAL_TOLERANCE_VIOLATED;
            g_as->setAborted(result, "goal not reached in time");
            return false;
        }
    }
    return true;
}

bool initCallback(std_srvs::Trigger::Request  &req, std_srvs::Trigger::Response &res)
{
    boost::mutex::scoped_lock lock(g_mutex);
    
    ros::NodeHandle nh_priv("~");
    
    if(!g_status) {
        res.message = "hand is not yet connected";
    }else if(g_status->status == g_status->NOT_INITIALIZED) {
        lock.unlock();
        cob_hand_bridge::InitFinger srv;
        srv.request.port = nh_priv.param<std::string>("sdhx/port", "/dev/ttyACM0");
        srv.request.min_pwm0 = nh_priv.param("sdhx/min_pwm0", 0);
        srv.request.min_pwm1 = nh_priv.param("sdhx/min_pwm1", 0);
        srv.request.max_pwm0 = nh_priv.param("sdhx/max_pwm0", 0);
        srv.request.max_pwm1 = nh_priv.param("sdhx/max_pwm1", 0);

        if(g_init_finger_client.waitForExistence(ros::Duration(nh_priv.param("sdhx/connect_timeout", 10)))){
            if(!g_init_finger_client.call(srv)) return false;
            res.success = srv.response.success;
            if(res.success) g_as->start();
        }else{
            res.message = "init_finger service does not exist";
        }
        lock.lock();
        g_initialized = res.success;
    }else if(!g_initialized){
        g_as->start();
        res.success = true;
        res.message = "finger already initialized, restarting the controller";
        g_initialized = true;
    }else{
        res.success = true;
        res.message = "already initialized";
    }

    return true;
}

void statusCallback(const cob_hand_bridge::Status::ConstPtr& msg){
    boost::mutex::scoped_lock lock(g_mutex);
    double dt = g_js.header.stamp.toSec() - msg->stamp.toSec();
    bool first = !g_status;
    bool calc_vel = !first && dt != 0 ;
    g_status = msg;
    g_topic_status->tick(msg->stamp);

    g_motion_stopped = true;
    g_control_stopped = true;

    if(msg->status & msg->MASK_FINGER_READY){
        for(size_t i=0; i < msg->joints.position_cdeg.size(); ++i){
            double new_pos = angles::from_degrees(msg->joints.position_cdeg[i]/100.0);
            if(calc_vel){
                g_js.velocity[i] = (new_pos - g_js.position[i]) / dt;
            }
            if(fabs(g_js.velocity[i]) > g_stopped_velocity){
                g_motion_stopped = false;
        	g_motors_moved = true; 
            }
            if(fabs(msg->joints.current_100uA[i]) > g_stopped_current){
                g_control_stopped = false;
            }
            g_js.position[i] = new_pos;
        }
        g_js.header.stamp = msg->stamp;
        g_js_pub.publish(g_js);
    }
    checkAction_nolock(false);
    
    if(first) g_init_srv = ros::NodeHandle("driver").advertiseService("init", initCallback);
}

void reportDiagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat){
    boost::mutex::scoped_lock lock(g_mutex);
    
    if(!g_status){
        stat.summary(stat.ERROR, "not connected");
        return;
    }
    
    if(g_status->status == g_status->NOT_INITIALIZED){
        stat.summary(stat.WARN, "not initialized");
    }else if(g_status->status & g_status->MASK_ERROR){
        stat.summary(stat.ERROR, "Bridge has error");
    }else{
        stat.summary(stat.OK, "Connected and running");
    }
    stat.add("sdhx_ready", bool(g_status->status & g_status->MASK_FINGER_READY));
    stat.add("sdhx_rc", uint32_t(g_status->rc));
    stat.add("sdhx_motion_stopped", g_motion_stopped);
    stat.add("sdhx_control_stopped", g_control_stopped);

    if(g_status->rc > 0){
        stat.mergeSummary(stat.ERROR, "SDHx has error");
    }
}

void callHalt(){
    if(g_halt_client.exists()){
        std_srvs::Trigger srv;
        if(!g_halt_client.call(srv) || !srv.response.success){
            ROS_ERROR("Halt service did not succeed");
        }
    }else{
        ROS_ERROR("Halt service is not available");
    }
}

void handleDeadline(const ros::TimerEvent &){
    boost::mutex::scoped_lock lock(g_mutex);
    if(g_as->isActive()){
        if(!checkAction_nolock(true)){
            g_command.position_cdeg = g_status->joints.position_cdeg;
            lock.unlock();
            callHalt();
            lock.lock();
        }
    }
} 

void goalCB() {

    control_msgs::FollowJointTrajectoryGoalConstPtr goal = g_as->acceptNewGoal();
    
    g_deadline_timer.stop();

    // goal is invalid if goal has more than 2 (or 0) points. If 2 point, the first needs time_from_start to be 0
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = result.INVALID_GOAL;
    if(goal->trajectory.points.size()!=1 && (goal->trajectory.points.size()!=2 || !goal->trajectory.points[0].time_from_start.isZero() ) ){
        g_as->setAborted(result, "goal is not valid");
        return;
    }

    cob_hand_bridge::JointValues new_command = g_default_command;
    size_t found = 0;

    for(size_t i=0; i < g_js.name.size(); ++i){
        for(size_t j=0; j < goal->trajectory.joint_names.size(); ++j){
            if(g_js.name[i] == goal->trajectory.joint_names[j]){
                new_command.position_cdeg[i]= angles::to_degrees(goal->trajectory.points.back().positions[j])*100; // cdeg to rad
                
                if(goal->trajectory.points.back().effort.size() >  0){
                    if(goal->trajectory.points.back().effort.size() ==  new_command.current_100uA.size()){
                        new_command.current_100uA[i] = goal->trajectory.points.back().effort[j] * 1000.0; // (A -> 100uA)
                    }else{
                        g_as->setAborted(result, "Number of effort values  mismatch");
                        return;
                    }
                }
                
                ++found;
                break;
            }
        }
    }

    if(found != g_js.name.size()){
        g_as->setAborted(result, "Joint names mismatch");
        return;
    }
    std::vector<double> goal_tolerance(g_command.position_cdeg.size(), angles::to_degrees(g_stopped_velocity)*100); // assume 1s movement is allowed, cdeg to rad

    for(size_t i = 0; i < goal->goal_tolerance.size(); ++i){
        bool missing = true;
        for(size_t j=0; j < g_js.name.size(); ++j){
            if(goal->goal_tolerance[i].name == g_js.name[j]){
                missing = false;
                if(goal->goal_tolerance[i].position > 0.0){
                    goal_tolerance[j] = goal->goal_tolerance[i].position; 
                }
                break;
            }
        }
        if(missing){
            g_as->setAborted(result, "Goal tolerance invalid");
            return;
        }
    }

    ros::Time now = ros::Time::now();
    ros::Time trajectory_deadline = (goal->trajectory.header.stamp.isZero() ? now : goal->trajectory.header.stamp)
                                  + goal->trajectory.points.back().time_from_start + ros::Duration(goal->goal_time_tolerance);
    if(trajectory_deadline <= now){
        result.error_code = result.OLD_HEADER_TIMESTAMP;
        g_as->setAborted(result, "goal is not valid");
        return;
    }

    boost::mutex::scoped_lock lock(g_mutex);

    if(!isFingerReady_nolock()) {
        g_as->setAborted(result, "SDHx is not ready for commands");
        return;
    }

    g_command = new_command;
    
    g_goal_tolerance = goal_tolerance;
    g_motors_moved = false;    
    g_command_timer.stop();
    g_command_pub.publish(g_command);
    g_deadline_timer.setPeriod(trajectory_deadline-ros::Time::now());
    g_deadline_timer.start();
    g_command_timer.start();
}

void cancelCB() {
    boost::mutex::scoped_lock lock(g_mutex);
    g_command.position_cdeg = g_status->joints.position_cdeg;
    g_deadline_timer.stop();
    lock.unlock();
    
    callHalt();
    g_as->setPreempted();
}

void resendCommand(const ros::TimerEvent &e){
    boost::mutex::scoped_lock lock(g_mutex);
    if(isFingerReady_nolock()){
        if(g_control_stopped || !g_motion_stopped) g_command_pub.publish(g_command);
    } else {
        g_command_timer.stop();
        lock.unlock();
        callHalt();
        ROS_WARN("finger is not ready, stopped resend timer");
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "cob_hand_bridge_node");
    
    ros::NodeHandle nh;
    ros::NodeHandle nh_d("driver");
    ros::NodeHandle nh_i("internal");
    ros::NodeHandle nh_priv("~");
    
    if(!nh_priv.getParam("sdhx/joint_names", g_js.name)){
        ROS_ERROR("Please provide joint names for SDHx");
        return 1;
    }
    
    if(g_command.position_cdeg.size() != g_js.name.size()){
        ROS_ERROR_STREAM("Number of joints does not match " << g_command.position_cdeg.size());
        return 1;
    }
    
    
    nh_priv.param("sdhx/stopped_velocity",g_stopped_velocity, 0.05);
    if(g_stopped_velocity <= 0.0){
        ROS_ERROR_STREAM("stopped_velocity must be a positive number");
        return 1;
    }

    double stopped_current= nh_priv.param("sdhx/stopped_current", 0.1);
    if(stopped_current <= 0.0){
        ROS_ERROR_STREAM("stopped_current must be a positive number");
        return 1;
    }
    g_stopped_current = stopped_current * 1000.0; // (A -> 100uA)
    
    std::vector<double> default_currents;
    if(nh_priv.getParam("sdhx/default_currents", default_currents)){
        if(default_currents.size() !=  g_default_command.current_100uA.size()){
            ROS_ERROR_STREAM("Number of current values does not match number of joints");
            return 1;
        }
        for(size_t i=0; i< g_default_command.current_100uA.size(); ++i){
            g_default_command.current_100uA[i] = default_currents[i] * 1000.0; // (A -> 100uA)
        }
    }else{
        g_default_command.current_100uA[0] = 2120;
        g_default_command.current_100uA[1] = 1400;
    }

    g_default_command.velocity_cdeg_s[0] = 1000;
    g_default_command.velocity_cdeg_s[1] = 1000;

    g_js.position.resize(g_command.position_cdeg.size());
    g_js.velocity.resize(g_command.position_cdeg.size());

    diagnostic_updater::Updater diag_updater;
    diag_updater.setHardwareID(nh_priv.param("hardware_id", std::string("none")));
    diag_updater.add("bridge", reportDiagnostics);
    
    diagnostic_updater::TimeStampStatusParam param(
        nh_priv.param("status/min_duration", -1.0),
        nh_priv.param("status/max_duration", 0.1)
    );
    g_topic_status.reset ( new diagnostic_updater::TimeStampStatus(param) );
    diag_updater.add("connection", boost::bind(&diagnostic_updater::TimeStampStatus::run, g_topic_status, _1));
    
    ros::Timer diag_timer = nh.createTimer(ros::Duration(diag_updater.getPeriod()/2.0),boost::bind(&diagnostic_updater::Updater::update, &diag_updater));
    
    double resend_period = nh_priv.param("sdhx/resend_period", 0.1);
    if(resend_period > 0.0) g_command_timer = nh.createTimer(ros::Duration(resend_period), resendCommand, false, false);
    
    g_deadline_timer = nh.createTimer(ros::Duration(1.0), handleDeadline, true, false); // period is not used, will be overwritten

    g_js_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    
    ros::Subscriber status_sub = nh_i.subscribe("status", 1, statusCallback);
    g_init_finger_client = nh_i.serviceClient<cob_hand_bridge::InitFinger>("init_finger");
    g_command_pub = nh_i.advertise<cob_hand_bridge::JointValues>("command", 1);
    g_halt_client = nh_d.serviceClient<std_srvs::Trigger>("halt");

    g_as.reset(new FollowJointTrajectoryActionServer(ros::NodeHandle("joint_trajectory_controller"), "follow_joint_trajectory",false));
    g_as->registerPreemptCallback(cancelCB);
    g_as->registerGoalCallback(goalCB);
    
    ros::spin();
    return 0;
}
