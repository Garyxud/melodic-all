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

#ifndef __NERIAN_STEREO_NODE_H__
#define __NERIAN_STEREO_NODE_H__

#include <ros/ros.h>
#include <visiontransfer/asynctransfer.h>
#include <visiontransfer/reconstruct3d.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <iomanip>
#include <nerian_stereo/StereoCameraInfo.h>
#include <boost/smart_ptr.hpp>
#include <colorcoder.h>

#include <dynamic_reconfigure/server.h>
#include <nerian_stereo/NerianStereoConfig.h>
#include <visiontransfer/scenescanparameters.h>
#include <visiontransfer/exceptions.h>

using namespace std;
using namespace visiontransfer;

/**
 * \brief A driver node that receives data from SceneScan/SP1 and forwards
 * it to ROS.
 *
 * SceneScan and SP1 by Nerian Vision GmbH are hardware systems for
 * real-time stereo vision. They transmit a computed disparity map (an
 * inverse depth map) through gigabit ethernet, which is then received by
 * this node. The node converts the received data into ROS messages, which
 * contain the following data:
 *
 * - Point cloud of reconstructed 3D locations
 * - Disparity map with optional color coding
 * - Rectified left camera image
 *
 * In addition, camera calibration information is also published. For
 * configuration parameters, please see the provided example launch file.
 * For more information about Nerian's SceneScan system, please visit
 * http://nerian.com/products/scenescan-stereo-vision/
 */

namespace nerian_stereo {

class StereoNodeBase {
public:
    StereoNodeBase(): initialConfigReceived(false), frameNum(0) {
    }

    ~StereoNodeBase() {
    }

    /**
     * \brief Performs general initializations
     */
    void init();

    /*
     * \brief Initialize and publish configuration with a dynamic_reconfigure server
     */
    void initDynamicReconfigure();

    /**
     * \brief Connects to the image service to request the stream of image sets
     */
    void prepareAsyncTransfer();

    /*
     * \brief Collect and process a single image set (or return after timeout if none are available)
     */
    void processOneImageSet();

private:
    enum PointCloudColorMode {
        RGB_SEPARATE,
        RGB_COMBINED,
        INTENSITY,
        NONE
    };

    virtual ros::NodeHandle& getNH() = 0;
    virtual ros::NodeHandle& getPrivateNH() = 0;

    //
    boost::scoped_ptr<ros::Publisher> cloudPublisher;
    boost::scoped_ptr<ros::Publisher> disparityPublisher;
    boost::scoped_ptr<ros::Publisher> leftImagePublisher;
    boost::scoped_ptr<ros::Publisher> rightImagePublisher;
    boost::scoped_ptr<ros::Publisher> cameraInfoPublisher;

    // ROS dynamic_reconfigure
    boost::scoped_ptr<dynamic_reconfigure::Server<nerian_stereo::NerianStereoConfig>> dynReconfServer;
    nerian_stereo::NerianStereoConfig lastKnownConfig;
    bool initialConfigReceived;
    
    // Connection to parameter server on device
    boost::scoped_ptr<SceneScanParameters> sceneScanParameters;

    // Parameters
    bool useTcp;
    std::string colorCodeDispMap;
    bool colorCodeLegend;
    bool rosCoordinateSystem;
    bool rosTimestamps;
    std::string remotePort;
    std::string frame;
    std::string remoteHost;
    std::string calibFile;
    double execDelay;
    double maxDepth;
    bool useQFromCalibFile;
    PointCloudColorMode pointCloudColorMode;

    // Other members
    int frameNum;
    boost::scoped_ptr<Reconstruct3D> recon3d;
    boost::scoped_ptr<ColorCoder> colCoder;
    cv::Mat_<cv::Vec3b> colDispMap;
    sensor_msgs::PointCloud2Ptr pointCloudMsg;
    cv::FileStorage calibStorage;
    nerian_stereo::StereoCameraInfoPtr camInfoMsg;
    ros::Time lastCamInfoPublish;

    boost::scoped_ptr<AsyncTransfer> asyncTransfer;
    ros::Time lastLogTime;
    int lastLogFrames = 0;

    /**
     * \brief Loads a camera calibration file if configured
     */
    void loadCameraCalibration();

    /**
     * \brief Publishes the disparity map as 16-bit grayscale image or color coded
     * RGB image
     */
    void publishImageMsg(const ImageSet& imageSet, int imageIndex, ros::Time stamp, bool allowColorCode,
            ros::Publisher* publisher);

    /**
     * \brief Transform Q matrix to match the ROS coordinate system:
     * Swap y/z axis, then swap x/y axis, then invert y and z axis.
     */
    void qMatrixToRosCoords(const float* src, float* dst);

    /**
     * \brief Reconstructs the 3D locations form the disparity map and publishes them
     * as point cloud.
     */
    void publishPointCloudMsg(ImageSet& imageSet, ros::Time stamp);

    /**
     * \brief Copies the intensity or RGB data to the point cloud
     */
    template <PointCloudColorMode colorMode> void copyPointCloudIntensity(ImageSet& imageSet);

    /**
     * \brief Copies all points in a point cloud that have a depth smaller
     * than maxDepth. Other points are set to NaN.
     */
    template <int coord> void copyPointCloudClamped(float* src, float* dst, int size);

    /**
     * \brief Performs all neccessary initializations for point cloud+
     * publishing
     */
    void initPointCloud();

    /**
     * \brief Publishes the camera info once per second
     */
    void publishCameraInfo(ros::Time stamp, const ImageSet& imageSet);

    /**
     * \brief Reads a vector from the calibration file to a boost:array
     */
    template<class T> void readCalibrationArray(const char* key, T& dest);
    
    /*
     * \brief Callback that receives an updated configuration from ROS; internally uses autogen_dynamicReconfigureCallback
     */
    void dynamicReconfigureCallback(nerian_stereo::NerianStereoConfig &config, uint32_t level);

    /*
     * \brief Forward parameters from the device as initial values to the ROS parameter server; internally uses autogen_updateParameterServerFromDevice
     */
    void updateParameterServerFromDevice(std::map<std::string, ParameterInfo>& cfg);

    /*
     * \brief Uses parameters from the device as a run-time override for limits and defaults in the dynamic_reconfigure node; internally uses autogen_updateDynamicReconfigureFromDevice
     */
    void updateDynamicReconfigureFromDevice(std::map<std::string, ParameterInfo>& cfg);


    // The following three implementations are autogenerated by generate_nerian_config_cpp.py
    //  by parsing cfg/NerianStereo.cfg (which is also used by dynamic_reconfigure)
    /**
     * \brief Auto-generated code to check for parameter changes and forward them to the device
     */
    void autogen_dynamicReconfigureCallback(nerian_stereo::NerianStereoConfig &config, uint32_t level);
    /**
     * \brief Auto-generated code to set initial parameters according to those obtained from the device
     */
    void autogen_updateParameterServerFromDevice(std::map<std::string, ParameterInfo>& cfg);
    /**
     * \brief Auto-generated code to override the dynamic_reconfigure limits and defaults for all parameters
     */
    void autogen_updateDynamicReconfigureFromDevice(std::map<std::string, ParameterInfo>& cfg);

};

} // namespace

#endif

