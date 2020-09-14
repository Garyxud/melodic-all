#ifndef ROS_REFLEXXES_POSITION_INTERFACE_H
#define ROS_REFLEXXES_POSITION_INTERFACE_H

#include <ros/ros.h>
#include <libreflexxestype2/ReflexxesAPI.h>
#include <libreflexxestype2/RMLPositionFlags.h>
#include <libreflexxestype2/RMLPositionInputParameters.h>
#include <libreflexxestype2/RMLPositionOutputParameters.h>

class RosReflexxesPositionInterface
{
private:
    ros::NodeHandle nh_;
    //paramers
    int n_dim_;
    double period_;
    std::vector<double> max_velocities_;
    std::vector<double> max_acceleration_;
    std::vector<double> max_jerk_;
    
    
    RMLPositionFlags flags_;
    boost::shared_ptr<RMLPositionInputParameters> input_params_;
    boost::shared_ptr<RMLPositionOutputParameters> output_params_;
    boost::shared_ptr<ReflexxesAPI> rml_;
    bool position_initialized_;
    
    //functions
    bool load_parameters(std::string ns);
    
public:
    RosReflexxesPositionInterface(std::string ns);
    ~RosReflexxesPositionInterface();
    
    void starting(std::vector<double> c_pos);
    void advance_reflexxes();//advance reflexxes if initialized and position is known
    std::vector<double> update();//advance reflexxes and returns next position
    void reset_to_previous_state(RMLPositionInputParameters previous_state);
    
    //getter functions
    std::vector<double> get_current_velocity();
    std::vector<double> get_current_position();
    std::vector<double> get_target_position();
    RMLPositionInputParameters get_current_state();
    ros::Duration get_time_to_target_completedness();
    
    //setter functions
    void set_target_position(std::vector<double> t_pos);
    void set_target_velocity(std::vector<double> t_vel);
    
};

#endif
