#include <ros_reflexxes/RosReflexxesVelocityInterface.h>

RosReflexxesVelocityInterface::RosReflexxesVelocityInterface( std::string ns ) {
        if (!load_parameters(ns) ) {
            ROS_FATAL_STREAM("Unable to initialize Reflexxes as no parameters could be read. Please verify that all parameters exist on the given namespace '" << ns <<"' and try again!");
        } else {
                //initialize Reflexxes with loaded value
                rml_.reset(new ReflexxesAPI(n_dim_, period_));
                input_params_.reset(new RMLVelocityInputParameters(n_dim_));
                output_params_.reset(new RMLVelocityOutputParameters(n_dim_));
                for (int i=0; i<n_dim_; i++) {
                    input_params_->CurrentVelocityVector->VecData[i] = 0.0;
                    input_params_->CurrentAccelerationVector->VecData[i] = 0.0;
                    input_params_->MaxAccelerationVector->VecData[i] = max_acceleration_[i];
                    input_params_->MaxJerkVector->VecData[i] = max_jerk_[i];
                    input_params_->TargetVelocityVector->VecData[i] = 0.0;
                    input_params_->SelectionVector->VecData[i] = true;
                }
        }
        position_initialized_ = false;
}//RosReflexxesVelocityInterface::RosReflexxesVelocityInterface()

bool RosReflexxesVelocityInterface::load_parameters( std::string ns ) {
 std::string param_name = ns + "/dimensions";
    if (!nh_.getParam(param_name, n_dim_)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
        return false;
    }
        param_name = ns + "/period";
    if (!nh_.getParam(param_name, period_)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
        return false;
    }
    param_name = ns + "/max_velocities";
    if (!nh_.getParam(param_name, max_velocities_)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
        return false;
    }
    param_name = ns + "/max_acceleration";
    if (!nh_.getParam(param_name, max_acceleration_)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
        return false;
    }
    param_name = ns + "/max_jerk";
    if (!nh_.getParam(param_name, max_jerk_)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
        return false;
    }
    param_name = ns + "/sync_behavior";
    int sync_behavior_;
    if (!nh_.getParam(param_name, sync_behavior_)) {
        ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
        return false;
    }   
    if (sync_behavior_ == 0) { //set SyncronizationBehavior flag
            flags_.SynchronizationBehavior = RMLFlags::PHASE_SYNCHRONIZATION_IF_POSSIBLE;
    } else if (sync_behavior_ == 1) {
            flags_.SynchronizationBehavior = RMLFlags::ONLY_TIME_SYNCHRONIZATION;
    } else if (sync_behavior_ == 2) {
            flags_.SynchronizationBehavior = RMLFlags::ONLY_PHASE_SYNCHRONIZATION;       
    } else {
            flags_.SynchronizationBehavior = RMLFlags::NO_SYNCHRONIZATION;      
    }
    return true;
}//RosReflexxesVelocityInterface::load_parameters

void RosReflexxesVelocityInterface::starting(std::vector<double> c_pos) {
    
    if (c_pos.size() == n_dim_) {
            for (int i=0; i<n_dim_; i++) {
                    input_params_->CurrentPositionVector->VecData[i] = c_pos[i];
            }
            ROS_INFO("RosReflexxesVelocityInterface::starting successful");
            position_initialized_ = true;
    } else {
            ROS_WARN("RosReflexxesVelocityInterface::starting is unable to execute the input because input dimensions (%d) don't match the reflexxes dimension (%d)", (int) c_pos.size(), n_dim_ );
    }
}//RosReflexxesVelocityInterface::starting

void RosReflexxesVelocityInterface::advance_reflexxes() {
        if (position_initialized_) {
            int result_value = rml_->RMLVelocity(*input_params_, output_params_.get(), flags_);
            *input_params_->CurrentPositionVector   =   *output_params_->NewPositionVector;
            *input_params_->CurrentVelocityVector   =   *output_params_->NewVelocityVector;
            *input_params_->CurrentAccelerationVector   =   *output_params_->NewAccelerationVector;
    } else {
            ROS_WARN("RosReflexxesVelocityInterface::update is unable to advance as the current position is not initialized yet. Please define the current position by using 'set_current_position' setter function first!");
    }
}//RosReflexxesVelocityInterface::advance_reflexxes()

std::vector<double> RosReflexxesVelocityInterface::update() {
        advance_reflexxes();
        return get_current_velocity();
}//RosReflexxesVelocityInterface::update() 

void RosReflexxesVelocityInterface::reset_to_previous_state(RMLVelocityInputParameters previous_state) {
        input_params_.reset(new RMLVelocityInputParameters(n_dim_));
        *input_params_ = previous_state;
        output_params_.reset(new RMLVelocityOutputParameters(n_dim_));
}//RosReflexxesVelocityInterface::reset_to_previous_state

/*getter*/
std::vector<double> RosReflexxesVelocityInterface::get_current_acceleration() {
    std::vector<double> c_acc(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            c_acc[i] = input_params_->CurrentAccelerationVector->VecData[i];
    }
    return c_acc;
}//RosReflexxesVelocityInterface::get_current_acceleration()

std::vector<double> RosReflexxesVelocityInterface::get_current_velocity() {
    std::vector<double> c_vel(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            c_vel[i] = input_params_->CurrentVelocityVector->VecData[i];
    }
    return c_vel;
}//RosReflexxesVelocityInterface::get_current_velocity()

std::vector<double> RosReflexxesVelocityInterface::get_current_position() {
    std::vector<double> c_pos(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            c_pos[i] = input_params_->CurrentPositionVector->VecData[i];
    }
    return c_pos;
}//RosReflexxesVelocityInterface::get_current_position() 

std::vector<double> RosReflexxesVelocityInterface::get_target_velocity() {
    std::vector<double> t_vel(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            t_vel[i] = input_params_->TargetVelocityVector->VecData[i];
    }
    return t_vel;
}//RosReflexxesVelocityInterface::get_target_velocity()

/** 
 * This getter is given in order to copy and store the current state of reflexxes,
 * in order to be able to reset to a previous state if desired using the stored
 * states retrieved by this function.
 * @return reflexxes input parameters
 **/
RMLVelocityInputParameters RosReflexxesVelocityInterface::get_current_state() {
    RMLVelocityInputParameters c_state = *input_params_;
    return c_state;
}//RosReflexxesVelocityInterface::get_current_state()

double RosReflexxesVelocityInterface::get_period() {
        return period_;
}//RosReflexxesVelocityInterface::get_period()

/*setter*/
void RosReflexxesVelocityInterface::set_target_velocity(std::vector<double> t_vel) {
    if (t_vel.size() == n_dim_) {
            for (int i=0; i<n_dim_; i++) {
                    input_params_->TargetVelocityVector->VecData[i] = t_vel[i];
            }
    } else {
            ROS_WARN("RosReflexxesVelocityInterface::set_target_velocity is unable to execute the input because input dimensions (%d) don't match the reflexxes dimension (%d)", (int) t_vel.size(), n_dim_ );
    }
}//RosReflexxesVelocityInterface::set_target_velocity
