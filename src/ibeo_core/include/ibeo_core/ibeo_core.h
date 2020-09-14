/*
 * MIT License
 *
 * Copyright (c) 2018 AutonomouStuff, LLC
 *
 */

#ifndef IBEO_CORE_IBEO_CORE_H
#define IBEO_CORE_IBEO_CORE_H

#include <vector>
#include <cstdio>
#include <iostream>
#include <memory>
#include <cmath>

#include <sys/time.h>

#include <network_interface/network_utils.h>
#include <ibeo_core/utils.h>

namespace AS
{
namespace Drivers
{
namespace Ibeo
{
const uint8_t IBEO_HEADER_SIZE = 24;  // LUX_MESSAGE_DATA_OFFSET and LUX_HEADER_SIZE
const int32_t IBEO_PAYLOAD_SIZE = 10000;  // LUX_PAYLOAD_SIZE

enum Classification
{
  UNCLASSIFIED = 0,
  UNKNOWN_SMALL,
  UNKNOWN_BIG,
  PEDESTRIAN,
  BIKE,
  CAR,
  TRUCK
};

enum MirrorSide
{
  FRONT = 0,
  REAR
};

enum CoordinateSystem
{
  SCANNER = 0,
  VEHICLE
};

enum MotorRotatingDirection
{
  CLOCKWISE = 0,
  COUNTER_CLOCKWISE
};

enum PointLocation
{
  COG = 0,
  TOP_FRONT_LEFT_CORNER,
  TOP_FRONT_RIGHT_CORNER,
  BOTTOM_REAR_RIGHT_CORNER,
  BOTTOM_REAR_LEFT_CORNER,
  CENTER_OF_TOP_FRONT_EDGE,
  CENTER_OF_RIGHT_EDGE,
  CENTER_OF_BOTTOM_REAR_EDGE,
  CENTER_OF_LEFT_EDGE,
  BOX_CENTER,
  INVALID = 255
};

enum TrackingModel
{
  DYNAMIC = 0,
  STATIC
};

enum ObjectPhase
{
  INITIALIZATION = 0,
  TRACKING
};

enum DynamicProperty
{
  DYNAMIC_AND_MOVING = 0,
  DYNAMIC_AND_STOPPED,
  A_PRIORI_STATIONARY = 4
};

enum ImageFormat
{
  JPEG = 0,
  MJPEG,
  GRAY8,
  YUV420,
  YUV422
};

struct Point3D
{
  double x;
  double y;
  double z;
};

struct Point3DL :
  public Point3D
{
  uint32_t label;
};

class MountingPositionF
{
public:
  float yaw_angle;
  float pitch_angle;
  float roll_angle;
  float x_position;
  float y_position;
  float z_position;

  void parse(uint8_t *in);
};

class Point2Df  // LUX: Float2D and Point2D
{
public:
  float x;
  float y;

  void parse(uint8_t *in, ByteOrder bo);
};

class Point2Di
{
public:
  int16_t x;
  int16_t y;

  void parse(uint8_t *in, ByteOrder bo);
};

class Point2Dui
{
public:
  uint16_t x;
  uint16_t y;

  void parse(uint8_t *in, ByteOrder bo);
};

class Sigma2D
{
public:
  uint16_t sigma_x;
  uint16_t sigma_y;

  void parse(uint8_t *in, ByteOrder bo);
};

class Size2D
{
public:
  uint16_t size_x;
  uint16_t size_y;

  void parse(uint8_t *in, ByteOrder bo);
};

class Size2Df
{
public:
  float size_x;
  float size_y;
};

class Velocity2D
{
public:
  int16_t velocity_x;
  int16_t velocity_y;

  void parse(uint8_t *in, ByteOrder bo);
};

class ContourPointSigma : public Point2Di
{
public:
  uint8_t x_sigma;
  uint8_t y_sigma;

  void parse(uint8_t *in, ByteOrder bo);
};

class ResolutionInfo  // LUX: Resolution
{
public:
  float resolution_start_angle;
  float resolution;

  void parse(uint8_t *in);
};

class ScannerInfo2204
{
public:
  uint8_t device_id;
  uint8_t scanner_type;
  uint16_t scan_number;
  float start_angle;
  float end_angle;
  float yaw_angle;
  float pitch_angle;
  float roll_angle;
  float offset_x;
  float offset_y;
  float offset_z;

  void parse(uint8_t *in);
};

class ScannerInfo2205
{
public:
  uint8_t device_id;
  uint8_t scanner_type;
  uint16_t scan_number;
  float start_angle;
  float end_angle;
  NTPTime scan_start_time;
  NTPTime scan_end_time;
  NTPTime scan_start_time_from_device;
  NTPTime scan_end_time_from_device;
  float scan_frequency;
  float beam_tilt;
  uint32_t scan_flags;
  MountingPositionF mounting_position;
  ResolutionInfo resolutions[8];

  void parse(uint8_t *in);
};

class UntrackedProperties
{
public:
  uint16_t relative_time_of_measurement;
  Point2Di position_closest_point;
  Point2Di object_box_size;
  Point2Dui object_box_size_sigma;
  int16_t object_box_orientation;
  uint16_t object_box_orientation_sigma;
  Point2Di tracking_point_coordinate;
  Point2Dui tracking_point_coordinate_sigma;
  uint8_t number_of_contour_points;
  std::vector<ContourPointSigma> contour_point_list;

  void parse(uint8_t *in);
};

class TrackedProperties
{
public:
  uint16_t object_age;
  uint16_t hidden_status_age;
  ObjectPhase object_phase;
  DynamicProperty dynamic_property;
  uint16_t relative_time_of_measure;
  Point2Di position_closest_point;
  Point2Di relative_velocity;
  Point2Dui relative_velocity_sigma;
  Classification classification;
  uint16_t classification_age;
  Point2Di object_box_size;
  Point2Dui object_box_size_sigma;
  int16_t object_box_orientation;
  uint16_t object_box_orientation_sigma;
  PointLocation tracking_point_location;
  Point2Di tracking_point_coordinate;
  Point2Dui tracking_point_coordinate_sigma;
  Point2Di velocity;
  Point2Dui velocity_sigma;
  Point2Di acceleration;
  Point2Dui acceleration_sigma;
  int16_t yaw_rate;
  uint16_t yaw_rate_sigma;
  uint8_t number_of_contour_points;
  std::vector<ContourPointSigma> contour_point_list;

  void parse(uint8_t *in);
};

class ScanPoint2202
{
public:
  uint8_t layer;
  uint8_t echo;
  bool transparent_point;
  bool clutter_atmospheric;
  bool ground;
  bool dirt;
  int16_t horizontal_angle;
  uint16_t radial_distance;
  uint16_t echo_pulse_width;

  void parse(uint8_t *in);
};

class ScanPoint2204
{
public:
  float x_position;
  float y_position;
  float z_position;
  float echo_width;
  uint8_t device_id;
  uint8_t layer;
  uint8_t echo;
  uint32_t time_offset;
  bool ground;
  bool dirt;
  bool precipitation;

  void parse(uint8_t *in);
};

class ScanPoint2205
{
public:
  float x_position;
  float y_position;
  float z_position;
  float echo_width;
  uint8_t device_id;
  uint8_t layer;
  uint8_t echo;
  uint32_t time_offset;
  bool ground;
  bool dirt;
  bool precipitation;
  bool transparent;

  void parse(uint8_t *in);
};

class ScanPoint2208
{
public:
  uint8_t echo;
  uint8_t layer;
  bool transparent_point;
  bool clutter_atmospheric;
  bool ground;
  bool dirt;
  int16_t horizontal_angle;
  uint16_t radial_distance;
  uint16_t echo_pulse_width;

  void parse(uint8_t *in);
};

struct IbeoObject
{
  uint16_t id;
  uint32_t age;
  uint16_t prediction_age;
  uint16_t relative_timestamp;
  CoordinateSystem object_coordinate_system;
  PointLocation reference_point_location;
  Point2Df reference_point;
  Point2Df reference_point_sigma;
  Point2Df closest_point;
  Point2Df bounding_box_center;
  Size2Df bounding_box_size;
  Point2Df object_box_center;
  Size2Df object_box_size;
  float object_box_orientation;
  Point2Df absolute_velocity;
  Point2Df absolute_velocity_sigma;
  Point2Df relative_velocity;
  uint16_t classification;
  uint16_t classification_age;
  uint16_t classification_certainty;
  uint16_t number_of_contour_points;
  std::vector<Point3D> contour_point_list;
};

class Object2221
{
public:
  uint16_t id;
  uint16_t age;
  uint16_t prediction_age;
  uint16_t relative_timestamp;
  Point2Di reference_point;
  Point2Di reference_point_sigma;
  Point2Di closest_point;
  Point2Di bounding_box_center;
  uint16_t bounding_box_width;
  uint16_t bounding_box_length;
  Point2Di object_box_center;
  Size2D object_box_size;
  int16_t object_box_orientation;
  Point2Di absolute_velocity;
  Size2D absolute_velocity_sigma;
  Point2Di relative_velocity;
  Classification classification;
  uint16_t classification_age;
  uint16_t classification_certainty;
  uint16_t number_of_contour_points;
  std::vector<Point2Di> contour_point_list;

  void parse(uint8_t *in);
};

class Object2225
{
public:
  uint16_t id;
  uint32_t age;
  NTPTime timestamp;
  uint16_t hidden_status_age;
  Classification classification;
  uint8_t classification_certainty;
  uint32_t classification_age;
  Point2Df bounding_box_center;
  Point2Df bounding_box_size;
  Point2Df object_box_center;
  Point2Df object_box_center_sigma;
  Point2Df object_box_size;
  float yaw_angle;
  Point2Df relative_velocity;
  Point2Df relative_velocity_sigma;
  Point2Df absolute_velocity;
  Point2Df absolute_velocity_sigma;
  uint8_t number_of_contour_points;
  uint8_t closest_point_index;
  std::vector<Point2Df> contour_point_list;

  void parse(uint8_t *in);
};

class Object2270
{
public:
  uint16_t id;
  uint16_t age;
  uint16_t prediction_age;
  uint16_t relative_moment_of_measurement;
  PointLocation reference_point_location;
  int16_t reference_point_position_x;
  int16_t reference_point_position_y;
  uint16_t reference_point_position_sigma_x;
  uint16_t reference_point_position_sigma_y;
  int16_t contour_points_cog_x;
  int16_t contour_points_cog_y;
  uint16_t object_box_length;
  uint16_t object_box_width;
  int16_t object_box_orientation_angle;
  int16_t object_box_orientation_angle_sigma;
  int16_t absolute_velocity_x;
  int16_t absolute_velocity_y;
  uint16_t absolute_velocity_sigma_x;
  uint16_t absolute_velocity_sigma_y;
  int16_t relative_velocity_x;
  int16_t relative_velocity_y;
  uint16_t relative_velocity_sigma_x;
  uint16_t relative_velocity_sigma_y;
  Classification classification;
  TrackingModel tracking_model;
  bool mobile_detected;
  bool track_valid;
  uint16_t classification_age;
  uint16_t classification_confidence;
  uint16_t number_of_contour_points;
  std::vector<Point2Di> contour_point_list;

  void parse(uint8_t *in);
};

class Object2271
{
public:
  uint32_t id;
  bool untracked_properties_available;
  bool tracked_properties_available;
  UntrackedProperties untracked_properties;
  TrackedProperties tracked_properties;

  void parse(uint8_t *in);
};

class Object2280
{
public:
  uint16_t id;
  TrackingModel tracking_model;
  bool mobility_of_dyn_object_detected;
  bool motion_model_validated;
  uint32_t object_age;
  NTPTime timestamp;
  uint16_t object_prediction_age;
  Classification classification;
  uint8_t classification_certainty;
  uint32_t classification_age;
  Point2Df object_box_center;
  Point2Df object_box_center_sigma;
  Point2Df object_box_size;
  float object_box_orientation_angle;
  float object_box_orientation_angle_sigma;
  Point2Df relative_velocity;
  Point2Df relative_velocity_sigma;
  Point2Df absolute_velocity;
  Point2Df absolute_velocity_sigma;
  uint8_t number_of_contour_points;
  uint8_t closest_point_index;
  PointLocation reference_point_location;
  Point2Df reference_point_coordinate;
  Point2Df reference_point_coordinate_sigma;
  float reference_point_position_correction_coefficient;
  uint16_t object_priority;
  float object_existence_measurement;
  std::vector<Point2Df> contour_point_list;

  void parse(uint8_t *in);
};

class IbeoDataHeader  // LUX: LuxHeader_TX_Message
{
public:
  uint32_t previous_message_size;
  uint32_t message_size;
  uint8_t device_id;
  uint16_t data_type_id;
  NTPTime time;

  std::vector<uint8_t> encoded_data;

  void parse(uint8_t *in);
  void encode();
};

// Start the top-level messages.
class IbeoTxMessage
{
public:
  bool has_scan_points;
  bool has_contour_points;
  bool has_objects;
  IbeoDataHeader ibeo_header;
  uint16_t data_type;

  IbeoTxMessage();
  IbeoTxMessage(bool scan_points, bool contour_points, bool objects);

  static std::shared_ptr<IbeoTxMessage> make_message(const uint16_t& data_type);
  virtual void parse(uint8_t *in) = 0;
  virtual std::vector<Point3DL> get_scan_points();
  virtual std::vector<Point3D> get_contour_points();
  virtual std::vector<IbeoObject> get_objects();
};

class ErrorWarning : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  bool err_internal_error;
  bool err_motor_1_fault;
  bool err_buffer_error_xmt_incomplete;
  bool err_buffer_error_overflow;
  bool err_apd_over_temperature;
  bool err_apd_under_temperature;
  bool err_apd_temperature_sensor_defect;
  bool err_motor_2_fault;
  bool err_motor_3_fault;
  bool err_motor_4_fault;
  bool err_motor_5_fault;
  bool err_int_no_scan_data;
  bool err_int_communication_error;
  bool err_int_incorrect_scan_data;
  bool err_config_fpga_not_configurable;
  bool err_config_incorrect_config_data;
  bool err_config_contains_incorrect_params;
  bool err_timeout_data_processing;
  bool err_timeout_env_model_computation_reset;
  bool wrn_int_communication_error;
  bool wrn_low_temperature;
  bool wrn_high_temperature;
  bool wrn_int_motor_1;
  bool wrn_sync_error;
  bool wrn_laser_1_start_pulse_missing;
  bool wrn_laser_2_start_pulse_missing;
  bool wrn_can_interface_blocked;
  bool wrn_eth_interface_blocked;
  bool wrn_incorrect_can_data_rcvd;
  bool wrn_int_incorrect_scan_data;
  bool wrn_eth_unkwn_incomplete_data;
  bool wrn_incorrect_or_forbidden_cmd_rcvd;
  bool wrn_memory_access_failure;
  bool wrn_int_overflow;
  bool wrn_ego_motion_data_missing;
  bool wrn_incorrect_mounting_params;
  bool wrn_no_obj_comp_due_to_scan_freq;

  void parse(uint8_t *in);
};

class ScanData2202 : public IbeoTxMessage  // LUX: LuxScandata_TX_Message
{
public:
  static const int32_t DATA_TYPE;

  uint16_t scan_number;
  uint16_t scanner_status;  // Not yet available for ScaLa.
  uint16_t sync_phase_offset;
  NTPTime scan_start_time;
  NTPTime scan_end_time;
  uint16_t angle_ticks_per_rotation;
  int16_t start_angle_ticks;
  int16_t end_angle_ticks;
  uint16_t scan_points_count;
  int16_t mounting_yaw_angle_ticks;
  int16_t mounting_pitch_angle_ticks;
  int16_t mounting_roll_angle_ticks;
  int16_t mounting_position_x;
  int16_t mounting_position_y;
  int16_t mounting_position_z;
  bool ground_labeled;
  bool dirt_labeled;
  bool rain_labeled;
  MirrorSide mirror_side;
  std::vector<ScanPoint2202> scan_point_list;

  ScanData2202();

  void parse(uint8_t *in);
  std::vector<Point3DL> get_scan_points();
};

class ScanData2204 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime scan_start_time;
  uint32_t scan_end_time_offset;
  bool ground_labeled;
  bool dirt_labeled;
  bool rain_labeled;
  bool fused_scan;
  MirrorSide mirror_side;
  CoordinateSystem coordinate_system;
  uint16_t scan_number;
  uint16_t scan_points;
  uint16_t number_of_scanner_infos;
  std::vector<ScannerInfo2204> scanner_info_list;
  std::vector<ScanPoint2204> scan_point_list;

  ScanData2204();

  void parse(uint8_t *in);
  std::vector<Point3DL> get_scan_points();
};

class ScanData2205 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime scan_start_time;
  uint32_t scan_end_time_offset;
  bool fused_scan;
  MirrorSide mirror_side;
  CoordinateSystem coordinate_system;
  uint16_t scan_number;
  uint16_t scan_points;
  uint8_t number_of_scanner_infos;
  std::vector<ScannerInfo2205> scanner_info_list;
  std::vector<ScanPoint2205> scan_point_list;

  ScanData2205();

  void parse(uint8_t *in);
  std::vector<Point3DL> get_scan_points();
};

class ScanData2208 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  uint16_t scan_number;
  uint16_t scanner_type;
  bool motor_on;
  bool laser_on;
  bool frequency_locked;
  MotorRotatingDirection motor_rotating_direction;
  uint16_t angle_ticks_per_rotation;
  uint32_t scan_flags;
  int16_t mounting_yaw_angle_ticks;
  int16_t mounting_pitch_angle_ticks;
  int16_t mounting_roll_angle_ticks;
  int16_t mounting_position_x;
  int16_t mounting_position_y;
  int16_t mounting_position_z;
  uint8_t device_id;
  NTPTime scan_start_time;
  NTPTime scan_end_time;
  int16_t start_angle_ticks;
  int16_t end_angle_ticks;
  uint8_t subflags;
  MirrorSide mirror_side;
  int16_t mirror_tilt;
  uint16_t number_of_scan_points;
  std::vector<ScanPoint2208> scan_point_list;

  ScanData2208();

  void parse(uint8_t *in);
  std::vector<Point3DL> get_scan_points();
};

class ObjectData2221 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime scan_start_timestamp;
  uint16_t number_of_objects;
  std::vector<Object2221> object_list;

  ObjectData2221();

  void parse(uint8_t *in);
  std::vector<Point3D> get_contour_points();
  std::vector<IbeoObject> get_objects();
};

class ObjectData2225 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime mid_scan_timestamp;
  uint16_t number_of_objects;
  std::vector<Object2225> object_list;

  ObjectData2225();

  void parse(uint8_t *in);
  std::vector<Point3D> get_contour_points();
  std::vector<IbeoObject> get_objects();
};

class ObjectData2270 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime start_scan_timestamp;
  uint16_t object_list_number;
  uint16_t number_of_objects;
  std::vector<Object2270> object_list;

  ObjectData2270();

  void parse(uint8_t *in);
  std::vector<Point3D> get_contour_points();
  std::vector<IbeoObject> get_objects();
};

class ObjectData2271 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime start_scan_timestamp;
  uint16_t scan_number;
  uint16_t number_of_objects;
  std::vector<Object2271> object_list;

  ObjectData2271();

  void parse(uint8_t *in);
  std::vector<Point3D> get_contour_points();
  std::vector<IbeoObject> get_objects();
};

class ObjectData2280 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime mid_scan_timestamp;
  uint16_t number_of_objects;
  std::vector<Object2280> object_list;

  ObjectData2280();

  void parse(uint8_t *in);
  std::vector<Point3D> get_contour_points();
  std::vector<IbeoObject> get_objects();
};

class CameraImage : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  ImageFormat image_format;
  uint32_t us_since_power_on;
  NTPTime timestamp;
  uint8_t device_id;
  MountingPositionF mounting_position;
  double horizontal_opening_angle;
  double vertical_opening_angle;
  uint16_t image_width;
  uint16_t image_height;
  uint32_t compressed_size;
  std::vector<uint8_t> image_buffer;

  CameraImage();

  void parse(uint8_t *in);
};

class HostVehicleState2805 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime timestamp;
  uint16_t scan_number;
  uint16_t error_flags;
  int16_t longitudinal_velocity;
  int16_t steering_wheel_angle;
  int16_t front_wheel_angle;
  int32_t x_position;
  int32_t y_position;
  int16_t course_angle;
  uint16_t time_difference;
  int16_t x_difference;
  int16_t y_difference;
  int16_t heading_difference;
  int16_t current_yaw_rate;

  HostVehicleState2805();

  void parse(uint8_t *in);
};

class HostVehicleState2806 : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  NTPTime timestamp;
  int32_t distance_x;
  int32_t distance_y;
  float course_angle;
  float longitudinal_velocity;
  float yaw_rate;
  float steering_wheel_angle;
  float cross_acceleration;
  float front_wheel_angle;
  float vehicle_width;
  float vehicle_front_to_front_axle;
  float rear_axle_to_front_axle;
  float rear_axle_to_vehicle_rear;
  float steer_ratio_poly_0;
  float steer_ratio_poly_1;
  float steer_ratio_poly_2;
  float steer_ratio_poly_3;

  HostVehicleState2806();

  void parse(uint8_t *in);
};

class HostVehicleState2807 : public HostVehicleState2806
{
public:
  static const int32_t DATA_TYPE;

  NTPTime timestamp;
  int32_t distance_x;
  int32_t distance_y;
  float course_angle;
  float longitudinal_velocity;
  float yaw_rate;
  float steering_wheel_angle;
  float cross_acceleration;
  float front_wheel_angle;
  float vehicle_width;
  float vehicle_front_to_front_axle;
  float rear_axle_to_front_axle;
  float rear_axle_to_vehicle_rear;
  float steer_ratio_poly_0;
  float steer_ratio_poly_1;
  float steer_ratio_poly_2;
  float steer_ratio_poly_3;
  float longitudinal_acceleration;

  HostVehicleState2807();

  void parse(uint8_t *in);
};

class DeviceStatus : public IbeoTxMessage
{
public:
  static const int32_t DATA_TYPE;

  uint8_t scanner_type;
  float sensor_temperature;
  float frequency;

  DeviceStatus();

  void parse(uint8_t *in);
};

class CommandSetFilter
{
public:
  IbeoDataHeader ibeo_header;
  uint16_t command_identifier;
  uint16_t version;
  uint16_t begin_filter_range;
  uint16_t end_filter_range;

  std::vector<uint8_t> encoded_data;

  void encode();
};

}  // namespace Ibeo
}  // namespace Drivers
}  // namespace AS

#endif  // IBEO_CORE_IBEO_CORE_H
