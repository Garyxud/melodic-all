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

#include <pluginlib/class_list_macros.h>
#include "nerian_stereo_nodelet.h"

namespace nerian_stereo {

void StereoNodelet::stereoIteration(const ros::TimerEvent&) {
    processOneImageSet();
}

void StereoNodelet::onInit() {
    StereoNodeBase::init();
    StereoNodeBase::initDynamicReconfigure();
    prepareAsyncTransfer();
    // 2kHz timer for lower latency (stereoIteration will then block)
    timer = getNH().createTimer(ros::Duration(0.0005), &StereoNodelet::stereoIteration, this);
}

} // namespace

PLUGINLIB_EXPORT_CLASS(nerian_stereo::StereoNodelet, nodelet::Nodelet)


