#include <ros_reflexxes/RosReflexxesPositionInterface.h>

RosReflexxesPositionInterface::RosReflexxesPositionInterface( std::string ns ) {
    
        if (!load_parameters(ns) ) {
            ROS_FATAL_STREAM("RosReflexxesPositionInterface: Unable to initialize Reflexxes as no parameters could be read. Please verify that all parameters exist on the given namespace '" << ns <<"' and try again!");
        } else {
                //initialize Reflexxes with loaded values
                rml_.reset(new ReflexxesAPI(n_dim_, period_));
                input_params_.reset(new RMLPositionInputParameters(n_dim_));
                output_params_.reset(new RMLPositionOutputParameters(n_dim_));
                for (int i=0; i<n_dim_; i++) {
                    input_params_->CurrentVelocityVector->VecData[i] = 0.0;
                    input_params_->CurrentAccelerationVector->VecData[i] = 0.0;
                    input_params_->MaxVelocityVector->VecData[i] = max_velocities_[i];
                    input_params_->MaxAccelerationVector->VecData[i] = max_acceleration_[i];
                    input_params_->MaxJerkVector->VecData[i] = max_jerk_.at(i);
                    input_params_->TargetVelocityVector->VecData[i] = 0.0;
                    input_params_->SelectionVector->VecData[i] = true;
                }
        }
        position_initialized_ = false; 
}//RosReflexxesPositionInterface::RosReflexxesPositionInterface()

bool RosReflexxesPositionInterface::load_parameters(std::string ns) {
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
        param_name = ns + "/final_behavior";
        int final_behavior_;
        if (!nh_.getParam(param_name, final_behavior_)) {
            ROS_ERROR_STREAM("Failed to getParam '" << param_name << "' (namespace: " << ns << ").");
            return false;
        }
        if (final_behavior_ == 0) { //set BehaviorAfterFinalStateOfMotionIsReached
                        flags_.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::KEEP_TARGET_VELOCITY;
        } else {
                flags_.BehaviorAfterFinalStateOfMotionIsReached = RMLPositionFlags::RECOMPUTE_TRAJECTORY;
        }
        ROS_DEBUG("RosReflexxesPositionInterface::load_params:\nPeriod:%f\nMaxVel:[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\nMaxAcc:[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\nMaxJerk:[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\nSyncBehavior: %d\nFinalBehavior: %d", 
        period_, 
        max_velocities_[0], max_velocities_[1], max_velocities_[2], max_velocities_[3], max_velocities_[4], max_velocities_[5], 
        max_acceleration_[0], max_acceleration_[1], max_acceleration_[2], max_acceleration_[3], max_acceleration_[4], max_acceleration_[5], 
        max_jerk_[0], max_jerk_[1], max_jerk_[2], max_jerk_[3], max_jerk_[4], max_jerk_[5], 
        sync_behavior_, 
        final_behavior_);
        return true;
}//RosReflexxesPositionInterface::load_parameters

void RosReflexxesPositionInterface::starting( std::vector<double> c_pos ) {
    if (c_pos.size() == n_dim_) {
            for (int i=0; i<n_dim_; i++) {
                    input_params_->CurrentPositionVector->VecData[i] = c_pos[i];
            }
            position_initialized_ = true;
    } else {
            ROS_WARN("RosReflexxesPositionInterface::starting is unable to execute the input because input dimensions (%d) don't match the reflexxes dimension (%d)", (int) c_pos.size(), n_dim_ );
    }
}//RosReflexxesPositionInterface::starting

void RosReflexxesPositionInterface::advance_reflexxes() {
    if (position_initialized_) {
            int result_value = rml_->RMLPosition(*input_params_, output_params_.get(), flags_);
            *input_params_->CurrentPositionVector   =   *output_params_->NewPositionVector;
            *input_params_->CurrentVelocityVector   =   *output_params_->NewVelocityVector;
            *input_params_->CurrentAccelerationVector   =   *output_params_->NewAccelerationVector;
            ROS_DEBUG("Reflexxes::advance NEW CurrentPos:[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] \n", input_params_->CurrentPositionVector->VecData[0], input_params_->CurrentPositionVector->VecData[1], input_params_->CurrentPositionVector->VecData[2], input_params_->CurrentPositionVector->VecData[3], input_params_->CurrentPositionVector->VecData[4], input_params_->CurrentPositionVector->VecData[5] );
    } else {
            ROS_WARN("RosReflexxesPositionInterface::update is unable to advance as the current position is not initialized yet. Please define the current position by using 'set_current_position' setter function first!");
    }
}//RosReflexxesPositionInterface::advance_reflexxes()

std::vector<double> RosReflexxesPositionInterface::update() {
    advance_reflexxes();
    return get_current_position();
}//RosReflexxesPositionInterface::update()
    
void RosReflexxesPositionInterface::reset_to_previous_state(RMLPositionInputParameters previous_state) {
        input_params_.reset(new RMLPositionInputParameters(n_dim_));
        *input_params_ = previous_state;
        output_params_.reset(new RMLPositionOutputParameters(n_dim_));
}//RosReflexxesPositionInterface::reset_to_previous_state
    
/*getter*/
std::vector<double> RosReflexxesPositionInterface::get_current_velocity() {
    
    std::vector<double> c_vel(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            c_vel[i] = input_params_->CurrentVelocityVector->VecData[i];
    }
    return c_vel;
}//RosReflexxesPositionInterface::get_current_velocity()

std::vector<double> RosReflexxesPositionInterface::get_current_position() {
    
    std::vector<double> c_pos(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            c_pos[i] = input_params_->CurrentPositionVector->VecData[i];
    }
    return c_pos;
}//RosReflexxesPositionInterface::get_current_position()

std::vector<double> RosReflexxesPositionInterface::get_target_position() {
    
    std::vector<double> t_pos(n_dim_);
     for (int i=0; i<n_dim_; i++) {
            t_pos[i] = input_params_->TargetPositionVector->VecData[i];
    }
    return t_pos;
}//RosReflexxesPositionInterface::get_target_position()

/** 
 * This getter is given in order to copy and store the current state of reflexxes,
 * in order to be able to reset to a previous state if desired using the stored
 * states retrieved by this function.
 * @return reflexxes input parameters
 **/
RMLPositionInputParameters RosReflexxesPositionInterface::get_current_state() {
    RMLPositionInputParameters c_state = *input_params_;
    return c_state;
}//RosReflexxesPositionInterface::get_current_state()

ros::Duration RosReflexxesPositionInterface::get_time_to_target_completedness() {
    if ( output_params_->WasACompleteComputationPerformedDuringTheLastCycle() ) {
        return ros::Duration( output_params_->GetGreatestExecutionTime() );
    } else {
        return ros::Duration(-1.0);
    }
}
    
/*setter*/
void RosReflexxesPositionInterface::set_target_position( std::vector<double> t_pos ) {
    
    if (t_pos.size() == n_dim_) {
            for (int i=0; i<n_dim_; i++) {
                    input_params_->TargetPositionVector->VecData[i] = t_pos[i];
            }
    } else {
            ROS_WARN("RosReflexxesPositionInterface::set_target_position is unable to execute the input because input dimensions (%d) don't match the reflexxes dimension (%d)", (int) t_pos.size(), n_dim_ );
    }
}//RosReflexxesPositionInterface::set_target_position

void RosReflexxesPositionInterface::set_target_velocity( std::vector<double> t_vel ) {
    
    if (t_vel.size() == n_dim_) {
            for (int i=0; i<n_dim_; i++) {
                    input_params_->TargetVelocityVector->VecData[i] = t_vel[i];
            }
    } else {
            ROS_WARN("RosReflexxesPositionInterface::set_target_velocity is unable to execute the input because input dimensions (%d) don't match the reflexxes dimension (%d)", (int) t_vel.size(), n_dim_ );
    }
}//RosReflexxesPositionInterface::set_target_velocity

