/*******************************************************************************
 * Copyright (c) 2019 Nerian Vision GmbH
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *******************************************************************************/

#include <nodelet/nodelet.h>
#include "nerian_stereo_node_base.h"

namespace nerian_stereo {

class StereoNodelet: public StereoNodeBase, public nodelet::Nodelet {
public:
    /**
     * \brief Callback for the ros::Timer that replaces a processing loop in Nodelets; wraps processOneImageSet()
     */
    void stereoIteration(const ros::TimerEvent&);
    /**
     * \brief Nodelet initialization: performs ROS parameter/dynamic_reconfigure init, connects to image service, starts main iteration Timer
     */
    virtual void onInit();
private:
    // The nodelet does not initialize its own node handles
    inline ros::NodeHandle& getNH() override { return nodelet::Nodelet::getNodeHandle(); }
    inline ros::NodeHandle& getPrivateNH() override { return nodelet::Nodelet::getPrivateNodeHandle(); }
    ros::Timer timer;
};

} // namespace

