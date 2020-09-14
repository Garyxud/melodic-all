#pragma once

/* Navigation data blocs */
#include "navigation_data/acceleration_geographic_frame.h"
#include "navigation_data/acceleration_vessel_frame.h"
#include "navigation_data/acceleration_vessel_frame_deviation.h"
#include "navigation_data/ahrs_algorithm_status.h"
#include "navigation_data/ahrs_system_status.h"
#include "navigation_data/ahrs_user_status.h"
#include "navigation_data/attitude_heading.h"
#include "navigation_data/attitude_heading_deviation.h"
#include "navigation_data/attitude_quaternion.h"
#include "navigation_data/attitude_quaternion_deviation.h"
#include "navigation_data/course_speed_over_ground.h"
#include "navigation_data/current_geographic_frame.h"
#include "navigation_data/current_geographic_frame_deviation.h"
#include "navigation_data/heading_roll_pitch_rate.h"
#include "navigation_data/heave_surge_sway_speed.h"
#include "navigation_data/ins_algorithm_status.h"
#include "navigation_data/ins_system_status.h"
#include "navigation_data/ins_user_status.h"
#include "navigation_data/position.h"
#include "navigation_data/position_deviation.h"
#include "navigation_data/raw_acceleration_vessel_frame.h"
#include "navigation_data/realtime_heave_surge_sway.h"
#include "navigation_data/rotation_rate_vessel_frame.h"
#include "navigation_data/rotation_rate_vessel_frame_deviation.h"
#include "navigation_data/sensor_status.h"
#include "navigation_data/smart_heave.h"
#include "navigation_data/speed_geographic_frame.h"
#include "navigation_data/speed_geographic_frame_deviation.h"
#include "navigation_data/speed_vessel_frame.h"
#include "navigation_data/system_date.h"
#include "navigation_data/temperatures.h"

/* Extended navigation data blocs */
#include "extended_navigation_data/raw_rotation_rate_vessel_frame.h"
#include "extended_navigation_data/rotation_acceleration_vessel_frame.h"
#include "extended_navigation_data/rotation_acceleration_vessel_frame_deviation.h"
#include "extended_navigation_data/vehicle_attitude_heading.h"
#include "extended_navigation_data/vehicle_attitude_heading_deviation.h"
#include "extended_navigation_data/vehicle_position.h"
#include "extended_navigation_data/vehicle_position_deviation.h"

/* External data blocs */
#include "external_data/depth.h"
#include "external_data/dmi.h"
#include "external_data/dvl_ground_speed.h"
#include "external_data/dvl_water_speed.h"
#include "external_data/emlog.h"
#include "external_data/eventmarker.h"
#include "external_data/gnss.h"
#include "external_data/lbl.h"
#include "external_data/logbook.h"
#include "external_data/sound_velocity.h"
#include "external_data/turret_angles.h"
#include "external_data/usbl.h"
#include "external_data/utc.h"
#include "external_data/vtg.h"

#include <boost/optional.hpp>

namespace ixblue_stdbin_decoder
{
namespace Data
{
struct BinaryNav
{
    /* Navigation data blocs */
    boost::optional<AttitudeHeading> attitudeHeading;
    boost::optional<AttitudeHeadingDeviation> attitudeHeadingDeviation;
    boost::optional<RealTimeHeaveSurgeSway> rtHeaveSurgeSway;
    boost::optional<SmartHeave> smartHeave;
    boost::optional<HeadingRollPitchRate> headingRollPitchRate;
    boost::optional<RotationRateVesselFrame> rotationRateVesselFrame;
    boost::optional<AccelerationVesselFrame> accelerationVesselFrame;
    boost::optional<Position> position;
    boost::optional<PositionDeviation> positionDeviation;
    boost::optional<SpeedGeographicFrame> speedGeographicFrame;
    boost::optional<SpeedGeographicFrameDeviation> speedGeographicFrameDeviation;
    boost::optional<CurrentGeographicFrame> currentGeographicFrame;
    boost::optional<CurrentGeographicFrameDeviation> currentGeographicFrameDeviation;
    boost::optional<SystemDate> systemDate;
    boost::optional<SensorStatus> sensorStatus;
    boost::optional<INSAlgorithmStatus> insAlgorithmStatus;
    boost::optional<INSSystemStatus> insSystemStatus;
    boost::optional<INSUserStatus> insUserStatus;
    boost::optional<AHRSAlgorithmStatus> ahrsAlgorithmStatus;
    boost::optional<AHRSSystemStatus> ahrsSystemStatus;
    boost::optional<AHRSUserStatus> ahrsUserStatus;
    boost::optional<HeaveSurgeSwaySpeed> heaveSurgeSwaySpeed;
    boost::optional<SpeedVesselFrame> speedVesselFrame;
    boost::optional<AccelerationGeographicFrame> accelerationGeographicFrame;
    boost::optional<CourseSpeedoverGround> courseSpeedoverGround;
    boost::optional<Temperatures> temperatures;
    boost::optional<AttitudeQuaternion> attitudeQuaternion;
    boost::optional<AttitudeQuaternionDeviation> attitudeQuaternionDeviation;
    boost::optional<RawAccelerationVesselFrame> rawAccelerationVesselFrame;
    boost::optional<AccelerationVesselFrameDeviation> accelerationVesselFrameDeviation;
    boost::optional<RotationRateVesselFrameDeviation> rotationRateVesselFrameDeviation;

    /* Extended navigation data blocs */
    boost::optional<RotationAccelerationVesselFrame> rotationAccelerationVesselFrame;
    boost::optional<RotationAccelerationVesselFrameDeviation>
        rotationAccelerationVesselFrameDeviation;
    boost::optional<RawRotationRateVesselFrame> rawRotationRateVesselFrame;
    boost::optional<VehicleAttitudeHeading> vehicleAttitudeHeading;
    boost::optional<VehicleAttitudeHeadingDeviation> vehicleAttitudeHeadingDeviation;
    boost::optional<VehiclePosition> vehiclePosition;
    boost::optional<VehiclePositionDeviation> vehiclePositionDeviation;

    /* External data blocs */
    boost::optional<Utc> utc;
    boost::optional<Gnss> gnss1;
    boost::optional<Gnss> gnss2;
    boost::optional<Gnss> gnssManual;
    boost::optional<Emlog> emlog1;
    boost::optional<Emlog> emlog2;
    boost::optional<Usbl> usbl1;
    boost::optional<Usbl> usbl2;
    boost::optional<Usbl> usbl3;
    boost::optional<Depth> depth;
    boost::optional<DvlGroundSpeed> dvlGroundSpeed1;
    boost::optional<DvlWaterSpeed> dvlWaterSpeed1;
    boost::optional<SoundVelocity> soundVelocity;
    boost::optional<Dmi> dmi;
    boost::optional<Lbl> lbl1;
    boost::optional<Lbl> lbl2;
    boost::optional<Lbl> lbl3;
    boost::optional<Lbl> lbl4;
    boost::optional<EventMarker> eventMarkerA;
    boost::optional<EventMarker> eventMarkerB;
    boost::optional<EventMarker> eventMarkerC;
    boost::optional<DvlGroundSpeed> dvlGroundSpeed2;
    boost::optional<DvlWaterSpeed> dvlWaterSpeed2;
    boost::optional<TurretAngles> turretAngles;
    boost::optional<Vtg> vtg1;
    boost::optional<Vtg> vtg2;
    boost::optional<LogBook> logBook;
};
} // namespace Data
} // namespace ixblue_stdbin_decoder
