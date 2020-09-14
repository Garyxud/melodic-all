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

#include "nerian_stereo_node_base.h"

namespace nerian_stereo {

class StereoNode: public StereoNodeBase {
public:
    StereoNode(): privateNhInternal("~") { }

    /**
     * \brief The main loop of this node
     */
    int run() {
        prepareAsyncTransfer();
        try {
            while(ros::ok()) {
                // Dispatch any queued ROS callbacks
                ros::spinOnce();
                // Get a single image set and process it
                processOneImageSet();
            }
        } catch(const std::exception& ex) {
            ROS_FATAL("Exception occured: %s", ex.what());
            return 1;
        }
        return 0;
    }
private:
    // The standalone node has its own private node handles
    ros::NodeHandle nhInternal;
    ros::NodeHandle privateNhInternal;
    inline ros::NodeHandle& getNH() override { return nhInternal; }
    inline ros::NodeHandle& getPrivateNH() override { return privateNhInternal; }
};

} // namespace

int main(int argc, char** argv) {
    try {
        ros::init(argc, argv, "nerian_stereo");
        nerian_stereo::StereoNode node;
        node.init();
        node.initDynamicReconfigure();
        return node.run();
    } catch(const std::exception& ex) {
        ROS_FATAL("Exception occured: %s", ex.what());
        return 1;
    }
}

