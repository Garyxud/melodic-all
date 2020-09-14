#ifndef ROS_REFLEXXES_VELOCITY_INTERFACE_H
#define ROS_REFLEXXES_VELOCITY_INTERFACE_H

#include <ros/ros.h>
#include <libreflexxestype2/ReflexxesAPI.h>
#include <libreflexxestype2/RMLVelocityFlags.h>
#include <libreflexxestype2/RMLVelocityInputParameters.h>
#include <libreflexxestype2/RMLVelocityOutputParameters.h>

class RosReflexxesVelocityInterface
{
private:
    ros::NodeHandle nh_;
    //paramers
    int n_dim_;
    double period_;
    std::vector<double> max_velocities_;
    std::vector<double> max_acceleration_;
    std::vector<double> max_jerk_;

    RMLVelocityFlags flags_;
    boost::shared_ptr<RMLVelocityInputParameters> input_params_;
    boost::shared_ptr<RMLVelocityOutputParameters> output_params_;
    boost::shared_ptr<ReflexxesAPI> rml_;
    bool position_initialized_;
    bool load_parameters(std::string ns);
    
public:
    RosReflexxesVelocityInterface(std::string ns);
    ~RosReflexxesVelocityInterface();

    void starting(std::vector<double> c_pos);
    void advance_reflexxes();//advance reflexxes if initialized and position is known
    std::vector<double> update(); //advance reflexxes and return current velocity
    void reset_to_previous_state(RMLVelocityInputParameters previous_state);
    
    //getter functions
    std::vector<double> get_current_acceleration();
    std::vector<double> get_current_velocity();
    std::vector<double> get_current_position();
    std::vector<double> get_target_velocity();
    double get_period();
    RMLVelocityInputParameters get_current_state();
    
    //setter functions
    void set_target_velocity(std::vector<double> t_vel);
    
};

#endif
