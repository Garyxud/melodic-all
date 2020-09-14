/**
 * @file    base_controller_node.cpp
 * @author  Fabian Prinzing <scheik.todeswache@googlemail.com>
 * @version v2.0.0
 *
 * @section LICENSE
 *
 * Copyright (C) 2015, Fabian Prinzing. All Rights Reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   1.Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   2.Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   3.The name of the author may not be used to endorse or promote products
 *     derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Fabian Prinzing "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * @section DESCRIPTION
 *
 * Details
 */

#include <ros/ros.h>                                                                            /**<  ROS */
#include <geometry_msgs/Twist.h>                                                                /**<  ROS Twist message */
#include <md49_serialport/md49_serialport.h>                                                              /**<  library for serial communications via UART*/
#include <md49_messages/md49_data.h>                                                          /**<  custom message /md49_data */
#include <md49_messages/md49_encoders.h>                                                      /**<  custom message /md49_encoders */
#include "md49_base_controller/md49_base_controller_class.h"

int main( int argc, char* argv[] ){

    // *****************
    // * Init as ROS node *
    // *****************
    ros::init(argc, argv, "base_controller" );
    BaseController myBaseController;
    ros::Rate loop_rate(10);
    ROS_INFO("base_controller: base_controller running...");

    // *******************
    // * Open serialport *
    // *******************
    myBaseController.open_serialport();

    // *****************************
    // * Set initial MD49 settings *
    // *****************************
    myBaseController.init_md49(myBaseController.get_requested_speed_l(), myBaseController.get_requested_speed_r(), myBaseController.get_initial_md49_mode(), \
                               myBaseController.get_initial_md49_acceleration(), myBaseController.get_initial_md49_timeout(), myBaseController.get_initial_md49_regulator());

    // ************
    // * Mainloop *
    // ************
    while(myBaseController.n.ok())
    {
        // set speed on MD49 via UART as set through /cmd_vel if speed_l or speed_r changed since last cycle
        if ((myBaseController.get_requested_speed_l() != myBaseController.get_actual_speed_l()) || (myBaseController.get_requested_speed_r() != myBaseController.get_actual_speed_r()))
        {
            myBaseController.set_speed(myBaseController.get_requested_speed_l(),myBaseController.get_requested_speed_r());
            myBaseController.set_actual_speed_l(myBaseController.get_requested_speed_l());
            myBaseController.set_actual_speed_r(myBaseController.get_requested_speed_r());
            ROS_INFO("base_controller: Set speed_l=%i and speed_r=%i on MD49", myBaseController.get_requested_speed_l(), myBaseController.get_requested_speed_r());
        }
        // Read encoder- data from MD49 via UART and publish encoder values as read to topic /md49_encoders
        myBaseController.publish_encoders();
        // Read other- data from MD49 via UART and publish MD49 data as read to topic /md49_data
        myBaseController.publish_md49_data();
        // Loop
        ros::spinOnce();
        loop_rate.sleep();
    }// end.mainloop
    return 1;
} // end.main

