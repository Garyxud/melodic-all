/*
 *   ibeo_core.cpp - Core parsing and encoding library for Ibeo drivers.
 *   Copyright (C) 2017 AutonomouStuff, Co.
 *
 *   This library is free software; you can redistribute it and/or
 *   modify it under the terms of the GNU Lesser General Public
 *   License as published by the Free Software Foundation; either
 *   version 2.1 of the License, or (at your option) any later version.
 *
 *   This library is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *   Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public
 *   License along with this library; if not, write to the Free Software
 *   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
 *   USA
 */

#include <vector>
#include <ibeo_core/ibeo_core.h>

using namespace AS::Network;  // NOLINT
using namespace AS::Drivers::Ibeo;  // NOLINT

const int32_t AS::Drivers::Ibeo::ErrorWarning::DATA_TYPE = 0x2030;
const int32_t AS::Drivers::Ibeo::ScanData2202::DATA_TYPE = 0x2202;
const int32_t AS::Drivers::Ibeo::ScanData2204::DATA_TYPE = 0x2204;
const int32_t AS::Drivers::Ibeo::ScanData2205::DATA_TYPE = 0x2205;
const int32_t AS::Drivers::Ibeo::ScanData2208::DATA_TYPE = 0x2208;
const int32_t AS::Drivers::Ibeo::ObjectData2221::DATA_TYPE = 0x2221;
const int32_t AS::Drivers::Ibeo::ObjectData2225::DATA_TYPE = 0x2225;
const int32_t AS::Drivers::Ibeo::ObjectData2270::DATA_TYPE = 0x2270;
const int32_t AS::Drivers::Ibeo::ObjectData2271::DATA_TYPE = 0x2271;
const int32_t AS::Drivers::Ibeo::ObjectData2280::DATA_TYPE = 0x2280;
const int32_t AS::Drivers::Ibeo::CameraImage::DATA_TYPE = 0x2403;
const int32_t AS::Drivers::Ibeo::HostVehicleState2805::DATA_TYPE = 0x2805;
const int32_t AS::Drivers::Ibeo::HostVehicleState2806::DATA_TYPE = 0x2806;
const int32_t AS::Drivers::Ibeo::HostVehicleState2807::DATA_TYPE = 0x2807;
const int32_t AS::Drivers::Ibeo::DeviceStatus::DATA_TYPE = 0x6301;

// Primitive parsing functions
void MountingPositionF::parse(uint8_t *in)
{
  yaw_angle = read_be<float>(in, 4, 0);
  pitch_angle = read_be<float>(in, 4, 4);
  roll_angle = read_be<float>(in, 4, 8);
  x_position = read_be<float>(in, 4, 12);
  y_position = read_be<float>(in, 4, 16);
  z_position = read_be<float>(in, 4, 20);
}

void ContourPointSigma::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<int16_t>(in, &x, &y, bo);
  parse_tuple<uint8_t>(in, &x_sigma, &y_sigma, bo);
}

void Point2Df::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<float>(in, &x, &y, bo);
}

void Point2Di::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<int16_t>(in, &x, &y, bo);
}

void Point2Dui::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<uint16_t>(in, &x, &y, bo);
}

void Sigma2D::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<uint16_t>(in, &sigma_x, &sigma_y, bo);
}

void Size2D::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<uint16_t>(in, &size_x, &size_y, bo);
}

void Velocity2D::parse(uint8_t *in, ByteOrder bo)
{
  parse_tuple<int16_t>(in, &velocity_x, &velocity_y, bo);
}

void ResolutionInfo::parse(uint8_t *in)
{
  resolution_start_angle = read_be<float>(in, 4, 0);
  resolution = read_be<float>(in, 4, 4);
}

void ScannerInfo2204::parse(uint8_t *in)
{
  device_id = read_be<uint8_t>(in, 1, 0);
  scanner_type = read_be<uint8_t>(in, 1, 1);
  scan_number = read_be<uint16_t>(in, 2, 2);
  start_angle = read_be<float>(in, 4, 8);
  end_angle = read_be<float>(in, 4, 12);
  yaw_angle = read_be<float>(in, 4, 16);
  pitch_angle = read_be<float>(in, 4, 20);
  roll_angle = read_be<float>(in, 4, 24);
  offset_x = read_be<float>(in, 4, 28);
  offset_y = read_be<float>(in, 4, 32);
  offset_z = read_be<float>(in, 4, 36);
}

void ScannerInfo2205::parse(uint8_t *in)
{
  device_id = read_be<uint8_t>(in, 1, 0);
  scanner_type = read_be<uint8_t>(in, 1, 1);
  scan_number = read_be<uint16_t>(in, 2, 2);
  start_angle = read_be<float>(in, 4, 8);
  end_angle = read_be<float>(in, 4, 12);
  scan_start_time = read_be<NTPTime>(in, 8, 16);
  scan_end_time = read_be<NTPTime>(in, 8, 24);
  scan_start_time_from_device = read_be<NTPTime>(in, 8, 32);
  scan_end_time_from_device = read_be<NTPTime>(in, 8, 40);
  scan_frequency = read_be<float>(in, 4, 48);
  beam_tilt = read_be<float>(in, 4, 52);
  scan_flags = read_be<float>(in, 4, 56);
  mounting_position.parse(&in[60]);
  resolutions[0].parse(&in[84]);
  resolutions[1].parse(&in[92]);
  resolutions[2].parse(&in[100]);
  resolutions[3].parse(&in[108]);
  resolutions[4].parse(&in[116]);
  resolutions[5].parse(&in[124]);
  resolutions[6].parse(&in[132]);
  resolutions[7].parse(&in[140]);
}

// Object parsing functions
void UntrackedProperties::parse(uint8_t *in)
{
  relative_time_of_measurement = read_be<uint16_t>(in, 2, 1);
  position_closest_point.parse(&in[3], BE);
  object_box_size.parse(&in[9], BE);
  object_box_size_sigma.parse(&in[13], BE);
  object_box_orientation = read_be<int16_t>(in, 2, 17);
  object_box_orientation_sigma = read_be<uint16_t>(in, 2, 19);
  tracking_point_coordinate.parse(&in[23], BE);
  tracking_point_coordinate_sigma.parse(&in[27], BE);
  number_of_contour_points = read_be<uint8_t>(in, 1, 34);

  if (number_of_contour_points == 255)
    number_of_contour_points = 0;

  for (uint8_t i = 0; i < number_of_contour_points; i++)
  {
    ContourPointSigma new_contour_point;
    new_contour_point.parse(&in[35 + i * 8], BE);
    contour_point_list.push_back(new_contour_point);
  }
}

void TrackedProperties::parse(uint8_t *in)
{
  object_age = read_be<uint16_t>(in, 2, 1);
  hidden_status_age = read_be<uint16_t>(in, 2, 3);
  uint8_t dynamic_flags = read_be<uint8_t>(in, 1, 5);
  object_phase = ((dynamic_flags & 0x01) > 0) ? TRACKING : INITIALIZATION;
  dynamic_property = static_cast<DynamicProperty>((dynamic_flags & 0x70) > 0);
  relative_time_of_measure = read_be<uint16_t>(in, 2, 6);
  position_closest_point.parse(&in[8], BE);
  relative_velocity.parse(&in[12], BE);
  relative_velocity_sigma.parse(&in[16], BE);
  classification = static_cast<Classification>(read_be<uint8_t>(in, 1, 20));
  classification_age = read_be<uint16_t>(in, 2, 22);
  object_box_size.parse(&in[26], BE);
  object_box_size_sigma.parse(&in[30], BE);
  object_box_orientation = read_be<int16_t>(in, 2, 34);
  object_box_orientation_sigma = read_be<uint16_t>(in, 2, 36);
  tracking_point_location = static_cast<PointLocation>(read_be<uint8_t>(in, 1, 39));
  tracking_point_coordinate.parse(&in[40], BE);
  tracking_point_coordinate_sigma.parse(&in[44], BE);
  velocity.parse(&in[51], BE);
  velocity_sigma.parse(&in[55], BE);
  acceleration.parse(&in[61], BE);
  acceleration_sigma.parse(&in[65], BE);
  yaw_rate = read_be<int16_t>(in, 2, 71);
  yaw_rate_sigma = read_be<uint16_t>(in, 2, 73);
  number_of_contour_points = read_be<uint8_t>(in, 1, 75);

  if (number_of_contour_points == 255)
    number_of_contour_points = 0;

  for (uint8_t i = 0; i < number_of_contour_points; i++)
  {
    ContourPointSigma new_contour_point;
    new_contour_point.parse(&in[76 + i * 8], BE);
    contour_point_list.push_back(new_contour_point);
  }
}

void ScanPoint2202::parse(uint8_t *in)
{
  uint8_t layer_and_offset = read_le<uint8_t>(in, 1, 0);
  std::cout << std::hex;
  layer = (layer_and_offset & 0x0F);
  echo = ((layer_and_offset & 0xF0) >> 4);
  uint8_t flags = read_le<uint8_t>(in, 1, 1);
  transparent_point = ((flags & 0x01) > 0);
  clutter_atmospheric = ((flags & 0x02) > 0);
  ground = ((flags & 0x04) > 0);
  dirt = ((flags & 0x08) > 0);
  horizontal_angle = read_le<int16_t>(in, 2, 2);
  radial_distance = read_le<uint16_t>(in, 2, 4);
  echo_pulse_width = read_le<uint16_t>(in, 2, 6);
}

void ScanPoint2204::parse(uint8_t *in)
{
  x_position = read_be<float>(in, 4, 0);
  y_position = read_be<float>(in, 4, 4);
  z_position = read_be<float>(in, 4, 8);
  echo_width = read_be<float>(in, 4, 12);
  device_id = read_be<uint8_t>(in, 1, 16);
  layer = read_be<uint8_t>(in, 1, 17);
  echo = read_be<uint8_t>(in, 1, 18);
  time_offset = read_be<uint32_t>(in, 4, 20);

  uint16_t flags = read_be<uint16_t>(in, 2, 24);

  ground = ((flags & 0x0001) > 0);
  dirt = ((flags & 0x0002) > 0);
  precipitation = ((flags & 0x0004) > 0);
}

void ScanPoint2205::parse(uint8_t *in)
{
  x_position = read_be<float>(in, 4, 0);
  y_position = read_be<float>(in, 4, 4);
  z_position = read_be<float>(in, 4, 8);
  echo_width = read_be<float>(in, 4, 12);
  device_id = read_be<uint8_t>(in, 1, 16);
  layer = read_be<uint8_t>(in, 1, 17);
  echo = read_be<uint8_t>(in, 1, 18);
  time_offset = read_be<uint8_t>(in, 4, 20);
  uint16_t flags = read_be<uint8_t>(in, 2, 24);
  ground = ((flags & 0x0001) > 0);
  dirt = ((flags & 0x0002) > 0);
  precipitation = ((flags & 0x0004) > 2);
  transparent = ((flags & 0x1000) > 12);
}

void ScanPoint2208::parse(uint8_t *in)
{
  echo = (read_be<uint8_t>(in, 1, 0) & 0x0C);
  layer = read_be<uint8_t>(in, 1, 1);  // & 0x03; < should be whole byte, but this gives unexpected results.
  uint16_t flags = read_be<uint16_t>(in, 2, 2);
  transparent_point = ((flags & 0x0001) > 0);
  clutter_atmospheric = ((flags & 0x0002) > 0);
  ground = ((flags & 0x0004) > 0);
  dirt = ((flags & 0x0008) > 0);
  horizontal_angle = read_be<int16_t>(in, 2, 4);
  radial_distance = read_be<uint16_t>(in, 2, 6);
  echo_pulse_width = read_be<uint16_t>(in, 2, 8);
}

void Object2221::parse(uint8_t *in)
{
  id = read_le<uint16_t>(in, 2, 0);
  age = read_le<uint16_t>(in, 2, 2);
  prediction_age = read_le<uint16_t>(in, 2, 4);
  relative_timestamp = read_le<uint16_t>(in, 2, 6);
  reference_point.x = read_le<int16_t>(in, 2, 8);
  reference_point.y = read_le<int16_t>(in, 2, 10);
  reference_point_sigma.x = read_le<int16_t>(in, 2, 12);
  reference_point_sigma.y = read_le<int16_t>(in, 2, 14);
  closest_point.x = read_le<int16_t>(in, 2, 16);
  closest_point.y = read_le<int16_t>(in, 2, 18);
  bounding_box_center.x = read_le<int16_t>(in, 2, 20);
  bounding_box_center.y = read_le<int16_t>(in, 2, 22);
  bounding_box_width = read_le<uint16_t>(in, 2, 24);
  bounding_box_length = read_le<uint16_t>(in, 2, 26);
  object_box_center.x = read_le<int16_t>(in, 2, 28);
  object_box_center.y = read_le<int16_t>(in, 2, 30);
  object_box_size.size_x = read_le<uint16_t>(in, 2, 32);
  object_box_size.size_y = read_le<uint16_t>(in, 2, 34);
  object_box_orientation = read_le<int16_t>(in, 2, 36);
  absolute_velocity.x = read_le<int16_t>(in, 2, 38);
  absolute_velocity.y = read_le<int16_t>(in, 2, 40);
  absolute_velocity_sigma.size_x = read_le<uint16_t>(in, 2, 42);
  absolute_velocity_sigma.size_y = read_le<uint16_t>(in, 2, 44);
  relative_velocity.x = read_le<int16_t>(in, 2, 46);
  relative_velocity.y = read_le<int16_t>(in, 2, 48);
  classification = static_cast<Classification>(read_le<uint8_t>(in, 1, 50));
  classification_age = read_le<uint16_t>(in, 2, 52);
  classification_certainty = read_le<uint16_t>(in, 2, 54);
  number_of_contour_points = read_le<uint16_t>(in, 2, 56);

  if (number_of_contour_points == 65535)
    number_of_contour_points = 0;

  for (uint16_t i = 0; i < number_of_contour_points; i++)
  {
    Point2Di contour_point;
    contour_point.parse(&in[58 + i * 4], LE);
    contour_point_list.push_back(contour_point);
  }
}

void Object2225::parse(uint8_t *in)
{
  id = read_be<uint16_t>(in, 2, 0);
  age = read_be<uint32_t>(in, 4, 4);
  timestamp = read_be<NTPTime>(in, 8, 8);
  hidden_status_age = read_be<uint16_t>(in, 2, 16);
  classification = static_cast<Classification>(read_be<uint8_t>(in, 1, 18));
  classification_certainty = read_be<uint8_t>(in, 1, 19);
  classification_age = read_be<uint32_t>(in, 4, 20);
  bounding_box_center.parse(&in[24], BE);
  bounding_box_size.parse(&in[32], BE);
  object_box_center.parse(&in[40], BE);
  object_box_center_sigma.parse(&in[48], BE);
  object_box_size.parse(&in[56], BE);
  yaw_angle = read_be<float>(in, 4, 72);
  relative_velocity.parse(&in[80], BE);
  relative_velocity_sigma.parse(&in[88], BE);
  absolute_velocity.parse(&in[96], BE);
  absolute_velocity_sigma.parse(&in[104], BE);
  number_of_contour_points = read_be<uint8_t>(in, 1, 130);
  closest_point_index = read_be<uint8_t>(in, 1, 131);

  if (number_of_contour_points == 255)
    number_of_contour_points = 0;

  for (uint8_t i = 0; i < number_of_contour_points; i++)
  {
    Point2Df contour_point;
    contour_point.parse(&in[132 + i * 8], BE);
    contour_point_list.push_back(contour_point);
  }
}

void Object2270::parse(uint8_t *in)
{
  id = read_le<uint16_t>(in, 2, 0);
  age = read_le<uint16_t>(in, 2, 2);
  prediction_age = read_le<uint16_t>(in, 2, 4);
  relative_moment_of_measurement = read_le<uint16_t>(in, 2, 6);
  reference_point_location = static_cast<PointLocation>(read_le<uint8_t>(in, 1, 9));
  reference_point_position_x = read_le<int16_t>(in, 2, 10);
  reference_point_position_y = read_le<int16_t>(in, 2, 12);
  reference_point_position_sigma_x = read_le<uint16_t>(in, 2, 14);
  reference_point_position_sigma_y = read_le<uint16_t>(in, 2, 16);
  contour_points_cog_x = read_le<int16_t>(in, 2, 36);
  contour_points_cog_y = read_le<int16_t>(in, 2, 38);
  object_box_length = read_le<uint16_t>(in, 2, 40);
  object_box_width = read_le<uint16_t>(in, 2, 42);
  object_box_orientation_angle = read_le<int16_t>(in, 2, 44);
  object_box_orientation_angle_sigma = read_le<int16_t>(in, 2, 50);
  absolute_velocity_x = read_le<int16_t>(in, 2, 52);
  absolute_velocity_y = read_le<int16_t>(in, 2, 54);
  absolute_velocity_sigma_x = read_le<uint16_t>(in, 2, 56);
  absolute_velocity_sigma_y = read_le<uint16_t>(in, 2, 58);
  relative_velocity_x = read_le<int16_t>(in, 2, 60);
  relative_velocity_y = read_le<int16_t>(in, 2, 62);
  relative_velocity_sigma_x = read_le<uint16_t>(in, 2, 64);
  relative_velocity_sigma_y = read_le<uint16_t>(in, 2, 66);
  classification = static_cast<Classification>(read_le<uint8_t>(in, 1, 68));
  uint8_t flags = read_le<uint8_t>(in, 1, 69);
  tracking_model = ((flags & 0x01) > 0) ? STATIC : DYNAMIC;
  mobile_detected = ((flags & 0x02) > 0);
  track_valid = ((flags & 0x04) > 0);
  classification_age = read_le<uint16_t>(in, 2, 70);
  classification_confidence = read_le<uint16_t>(in, 2, 72);
  number_of_contour_points = read_le<uint16_t>(in, 2, 74);

  if (number_of_contour_points == 65535)
    number_of_contour_points = 0;

  for (uint16_t i = 0; i < number_of_contour_points; i++)
  {
    Point2Di contour_point;
    contour_point.parse(&in[76 + i * 4], LE);
    contour_point_list.push_back(contour_point);
  }
}

void Object2271::parse(uint8_t *in)
{
  id = read_be<uint32_t>(in, 4, 0);
  uint8_t properties_available = read_be<uint8_t>(in, 1, 6);
  untracked_properties_available = ((properties_available & 0x02) > 0);
  tracked_properties_available = ((properties_available & 0x08) > 0);

  if (untracked_properties_available)
  {
    untracked_properties.parse(&in[7]);
  }
  else
  {
    untracked_properties.number_of_contour_points = 0;
  }

  if (tracked_properties_available)
  {
    tracked_properties.parse(&in[42 + untracked_properties.number_of_contour_points * 8]);
  }
  else
  {
    tracked_properties.number_of_contour_points = 0;
  }
}

void Object2280::parse(uint8_t *in)
{
  id = read_be<uint16_t>(in, 2, 0);

  uint16_t flags = read_be<uint16_t>(in, 2, 2);
  tracking_model = ((flags & 0x40) > 0) ? STATIC : DYNAMIC;
  mobility_of_dyn_object_detected = ((flags & 0x80) > 0);
  motion_model_validated = ((flags & 0x100) > 0);
  object_age = read_be<uint32_t>(in, 4, 4);
  timestamp = read_be<NTPTime>(in, 8, 8);
  object_prediction_age = read_be<uint16_t>(in, 2, 16);
  classification = static_cast<Classification>(read_be<uint8_t>(in, 1, 18));
  classification_certainty = read_be<uint8_t>(in, 1, 19);
  classification_age = read_be<uint32_t>(in, 4, 20);
  object_box_center.parse(&in[40], BE);
  object_box_center_sigma.parse(&in[48], BE);
  object_box_size.parse(&in[56], BE);
  object_box_orientation_angle = read_be<float>(in, 4, 72);
  object_box_orientation_angle_sigma = read_be<float>(in, 4, 76);
  relative_velocity.parse(&in[80], BE);
  relative_velocity_sigma.parse(&in[88], BE);
  absolute_velocity.parse(&in[96], BE);
  absolute_velocity_sigma.parse(&in[104], BE);
  number_of_contour_points = read_be<uint8_t>(in, 1, 130);
  closest_point_index = read_be<uint8_t>(in, 1, 131);
  reference_point_location = static_cast<PointLocation>(read_be<uint16_t>(in, 2, 132));
  reference_point_coordinate.parse(&in[134], BE);
  reference_point_coordinate_sigma.parse(&in[142], BE);
  reference_point_position_correction_coefficient = read_be<float>(in, 4, 150);
  object_priority = read_be<uint16_t>(in, 2, 162);
  object_existence_measurement = read_be<float>(in, 4, 164);

  if (number_of_contour_points == 255)
    number_of_contour_points = 0;

  for (uint8_t i = 0; i < number_of_contour_points; i++)
  {
    Point2Df new_contour_point;
    new_contour_point.parse(&in[168 + i * 8], BE);
    contour_point_list.push_back(new_contour_point);
  }
}

// High-level message parsers.
void IbeoDataHeader::parse(uint8_t *in)
{
  previous_message_size = read_be<uint32_t>(in, 4, 4);
  message_size = read_be<uint32_t>(in, 4, 8);
  device_id = read_be<uint8_t>(in, 1, 13);
  data_type_id = read_be<uint16_t>(in, 2, 14);
  time = read_be<NTPTime>(in, 8, 16);
}

void IbeoDataHeader::encode()
{
  encoded_data.clear();

  // Magic Word
  encoded_data.push_back(0xAF);
  encoded_data.push_back(0xFE);
  encoded_data.push_back(0xC0);
  encoded_data.push_back(0xC2);
  // size_of_previous_message (unused in live data)
  encoded_data.push_back(0x00);
  encoded_data.push_back(0x00);
  encoded_data.push_back(0x00);
  encoded_data.push_back(0x00);

  // message_size
  std::vector<uint8_t> encoded_message_size = write_be<uint32_t>(&message_size);
  encoded_data.insert(encoded_data.end(), encoded_message_size.begin(), encoded_message_size.end());
  // Reserved
  encoded_data.push_back(0x00);
  // device_id
  encoded_data.push_back(0x00);
  // data_type_id
  std::vector<uint8_t> encoded_data_type_id = write_be<uint16_t>(&data_type_id);
  encoded_data.insert(encoded_data.end(), encoded_data_type_id.begin(), encoded_data_type_id.end());
  // time
  std::vector<uint8_t> encoded_time = write_be<NTPTime>(&time);
  encoded_data.insert(encoded_data.end(), encoded_time.begin(), encoded_time.end());
}

IbeoTxMessage::IbeoTxMessage() :
  has_scan_points(false),
  has_contour_points(false),
  has_objects(false)
{}

IbeoTxMessage::IbeoTxMessage(bool scan_points, bool contour_points, bool objects) :
  has_scan_points(scan_points),
  has_contour_points(contour_points),
  has_objects(objects)
{}

std::shared_ptr<IbeoTxMessage> IbeoTxMessage::make_message(const uint16_t& data_type)
{
  switch (data_type)
  {
  case ErrorWarning::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ErrorWarning);
    break;
  case ScanData2202::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ScanData2202);
    break;
  case ScanData2205::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ScanData2205);
    break;
  case ScanData2208::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ScanData2208);
    break;
  case ObjectData2221::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ObjectData2221);
    break;
  case ObjectData2225::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ObjectData2225);
    break;
  case ObjectData2270::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ObjectData2270);
    break;
  case ObjectData2271::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ObjectData2271);
    break;
  case ObjectData2280::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new ObjectData2280);
    break;
  case CameraImage::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new CameraImage);
    break;
  case HostVehicleState2805::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new HostVehicleState2805);
    break;
  case HostVehicleState2806::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new HostVehicleState2806);
    break;
  case HostVehicleState2807::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new HostVehicleState2807);
    break;
  case DeviceStatus::DATA_TYPE:
    return std::shared_ptr<IbeoTxMessage>(new DeviceStatus);
    break;
  default:
    return NULL;
  }
}

std::vector<Point3DL> IbeoTxMessage::get_scan_points()  // <-- virtual
{
  return std::vector<Point3DL>();
}

std::vector<Point3D> IbeoTxMessage::get_contour_points()
{
  return std::vector<Point3D>();
}

std::vector<IbeoObject> IbeoTxMessage::get_objects()
{
  return std::vector<IbeoObject>();
}

void ErrorWarning::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  uint16_t err_reg_1 = read_le<uint16_t>(in, 2, 0);
  uint16_t err_reg_2 = read_le<uint16_t>(in, 2, 2);
  uint16_t wrn_reg_1 = read_le<uint16_t>(in, 2, 4);
  uint16_t wrn_reg_2 = read_le<uint16_t>(in, 2, 6);

  err_internal_error = ((err_reg_1 & 0x0001) > 0);
  err_motor_1_fault = ((err_reg_1 & 0x0002) > 0);
  err_buffer_error_xmt_incomplete = ((err_reg_1 & 0x0004) > 0);
  err_buffer_error_overflow = ((err_reg_1 & 0x0008) > 0);
  err_apd_over_temperature = ((err_reg_1 & 0x0100) > 0);
  err_apd_under_temperature = ((err_reg_1 & 0x0200) > 0);
  err_apd_temperature_sensor_defect = (err_apd_over_temperature && err_apd_under_temperature);
  err_motor_2_fault = ((err_reg_1 & 0x0400) > 0);
  err_motor_3_fault = ((err_reg_1 & 0x0800) > 0);
  err_motor_4_fault = ((err_reg_1 & 0x1000) > 0);
  err_motor_5_fault = ((err_reg_1 & 0x2000) > 0);

  err_int_no_scan_data = ((err_reg_2 & 0x0001) > 0);
  err_int_communication_error = ((err_reg_2 & 0x0002) > 0);
  err_int_incorrect_scan_data = ((err_reg_2 & 0x0004) > 0);
  err_config_fpga_not_configurable = ((err_reg_2 & 0x0008) > 0);
  err_config_incorrect_config_data = ((err_reg_2 & 0x0010) > 0);
  err_config_contains_incorrect_params = ((err_reg_2 & 0x0020) > 0);
  err_timeout_data_processing = ((err_reg_2 & 0x0040) > 0);
  err_timeout_env_model_computation_reset = ((err_reg_2 & 0x0080) > 0);

  wrn_int_communication_error = ((wrn_reg_1 & 0x0001) > 0);
  wrn_low_temperature = ((wrn_reg_1 & 0x0008) > 0);
  wrn_high_temperature = ((wrn_reg_1 & 0x0010) > 0);
  wrn_int_motor_1 = ((wrn_reg_1 & 0x0020) > 0);
  wrn_sync_error = ((wrn_reg_1 & 0x0080) > 0);
  wrn_laser_1_start_pulse_missing = ((wrn_reg_1 & 0x1000) > 0);
  wrn_laser_2_start_pulse_missing = ((wrn_reg_1 & 0x2000) > 0);

  wrn_can_interface_blocked = ((wrn_reg_2 & 0x0001) > 0);
  wrn_eth_interface_blocked = ((wrn_reg_2 & 0x0002) > 0);
  wrn_incorrect_can_data_rcvd = ((wrn_reg_2 & 0x0004) > 0);
  wrn_int_incorrect_scan_data = ((wrn_reg_2 & 0x0008) > 0);
  wrn_eth_unkwn_incomplete_data = ((wrn_reg_2 & 0x0010) > 0);
  wrn_incorrect_or_forbidden_cmd_rcvd = ((wrn_reg_2 & 0x0020) > 0);
  wrn_memory_access_failure = ((wrn_reg_2 & 0x0040) > 0);
  wrn_int_overflow = ((wrn_reg_2 & 0x0080) > 0);
  wrn_ego_motion_data_missing = ((wrn_reg_2 & 0x0100) > 0);
  wrn_incorrect_mounting_params = ((wrn_reg_2 & 0x0200) > 0);
  wrn_no_obj_comp_due_to_scan_freq = ((wrn_reg_2 & 0x0400) > 0);
}

ScanData2202::ScanData2202() :
  IbeoTxMessage(true, false, false)
{}

void ScanData2202::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  scan_number = read_le<uint16_t>(in, 2, 0);
  scanner_status = read_le<uint16_t>(in, 2, 2);
  sync_phase_offset  = read_le<uint16_t>(in, 2, 4);
  scan_start_time = read_le<NTPTime>(in, 8, 6);
  scan_end_time = read_le<NTPTime>(in, 8, 14);
  angle_ticks_per_rotation = read_le<uint16_t>(in, 2, 22);
  start_angle_ticks = read_le<int16_t>(in, 2, 24);
  end_angle_ticks = read_le<int16_t>(in, 2, 26);
  scan_points_count = read_le<uint16_t>(in, 2, 28);
  mounting_yaw_angle_ticks = read_le<int16_t>(in, 2, 30);
  mounting_pitch_angle_ticks = read_le<int16_t>(in, 2, 32);
  mounting_roll_angle_ticks = read_le<int16_t>(in, 2, 34);
  mounting_position_x = read_le<int16_t>(in, 2, 36);
  mounting_position_y = read_le<int16_t>(in, 2, 38);
  mounting_position_z = read_le<int16_t>(in, 2, 40);

  uint16_t flags = read_le<uint16_t>(in, 2, 42);
  ground_labeled = ((flags & 0x0001) > 0);
  dirt_labeled = ((flags & 0x0002) > 0);
  rain_labeled = ((flags & 0x0004) > 0);
  mirror_side = (((flags & 0x0400) > 0) ? REAR : FRONT);

  for (uint16_t i = 0; i < scan_points_count; i++)
  {
    ScanPoint2202 new_scan_point;
    new_scan_point.parse(&in[44 + i * 10]);
    scan_point_list.push_back(new_scan_point);
  }
}

std::vector<Point3DL> ScanData2202::get_scan_points()
{
  std::vector<Point3DL> v;
  double cm_to_m = 100.0;

  for (ScanPoint2202 sp : scan_point_list)
  {
    if (sp.echo == 0 && sp.layer < 4 && !sp.transparent_point && !sp.clutter_atmospheric && !sp.ground && !sp.dirt)
    {
      // Spherical coordinates to cartesian
      // This uses the following conventions (with right-hand rule applied):
      // phi = rotation about the Z axis (angle in radians from X = 0)
      // theta = angle away from Z+ (radians)
      // rho = radial distance (m)
      Point3DL p3d;
      double phi = ticks_to_angle(sp.horizontal_angle, angle_ticks_per_rotation);
      double theta;
      double rho = sp.radial_distance / cm_to_m;

      switch (sp.layer)
      {
      case 0:
        theta = (88.4 * (M_PI / 180.0));
        break;
      case 1:
        theta = (89.2 * (M_PI / 180.0));
        break;
      case 2:
        theta = (90.8 * (M_PI / 180.0));
        break;
      case 3:
        theta = (91.6 * (M_PI / 180.0));
        break;
      }

      p3d.x = rho * sin(theta) * cos(phi);
      p3d.y = rho * sin(theta) * sin(phi);
      p3d.z = rho * cos(theta);

      p3d.label = sp.layer;

      v.push_back(p3d);
    }
  }

  return v;
}

ScanData2204::ScanData2204() :
  IbeoTxMessage(true, false, false)
{}

void ScanData2204::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  scan_start_time = read_be<NTPTime>(in, 8, 0);
  scan_end_time_offset = read_be<uint32_t>(in, 4, 8);

  uint32_t flags = read_be<uint32_t>(in, 4, 12);
  ground_labeled = ((flags & 0x00000001) > 0);
  dirt_labeled = ((flags & 0x00000002) > 0);
  rain_labeled = ((flags & 0x00000004) > 0);
  fused_scan = ((flags & 0x00000100) > 0);
  mirror_side = ((flags & 0x00000200) > 0) ? REAR : FRONT;
  coordinate_system = ((flags & 0x00000400) > 0) ? VEHICLE : SCANNER;

  scan_number = read_be<uint16_t>(in, 2, 16);
  scan_points = read_be<uint16_t>(in, 2, 18);
  number_of_scanner_infos = read_be<uint8_t>(in, 1, 20);

  for (uint8_t i = 0; i < number_of_scanner_infos; i++)
  {
    ScannerInfo2204 scanner_info;
    scanner_info.parse(&in[24 + i * 40]);
    scanner_info_list.push_back(scanner_info);
  }

  for (uint16_t i = 0; i < scan_points; i++)
  {
    ScanPoint2204 scan_point;
    scan_point.parse(&in[24 + (number_of_scanner_infos * 40) + (i * 28)]);
    scan_point_list.push_back(scan_point);
  }
}

std::vector<Point3DL> ScanData2204::get_scan_points()
{
  std::vector<Point3DL> v;

  for (ScanPoint2204 sp : scan_point_list)
  {
    if (sp.echo == 0 && sp.layer < 4 && !sp.ground && !sp.dirt && !sp.precipitation)
    {
      Point3DL p3d;

      p3d.x = sp.x_position;
      p3d.y = sp.y_position;
      p3d.z = sp.z_position;
      p3d.label = sp.layer;
      v.push_back(p3d);
    }
  }

  return v;
}

ScanData2205::ScanData2205() :
  IbeoTxMessage(true, false, false)
{}

void ScanData2205::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  scan_start_time = read_be<NTPTime>(in, 8, 0);
  scan_end_time_offset = read_be<uint32_t>(in, 4, 8);

  uint32_t flags = read_be<uint32_t>(in, 4, 12);
  fused_scan = ((flags & 0x00000100) > 0);
  mirror_side = ((flags & 0x00000200) > 0) ? REAR : FRONT;
  coordinate_system = ((flags & 0x00000400) > 0) ? VEHICLE : SCANNER;

  scan_number = read_be<uint16_t>(in, 2, 16);
  scan_points = read_be<uint16_t>(in, 2, 18);
  number_of_scanner_infos = read_be<uint8_t>(in, 1, 20);

  // printf("%d SCAN POINTS REPORTED\n", scan_points);

  for (uint8_t i = 0; i < number_of_scanner_infos; i++)
  {
    ScannerInfo2205 new_scanner_info;
    new_scanner_info.parse(&in[24 + i * 148]);
    scanner_info_list.push_back(new_scanner_info);
  }

  for (uint16_t i = 0; i < scan_points; i++)
  {
    ScanPoint2205 new_scan_point;
    new_scan_point.parse(&in[24 + (number_of_scanner_infos * 148) + (i * 28)]);
    scan_point_list.push_back(new_scan_point);
  }
}

std::vector<Point3DL> ScanData2205::get_scan_points()
{
  std::vector<Point3DL> v;

  for (ScanPoint2205 sp : scan_point_list)
  {
    if (sp.echo == 0 && sp.layer < 4 && !sp.transparent && !sp.ground && !sp.dirt && !sp.precipitation)
    {
      Point3DL p3d;

      p3d.x = sp.x_position;
      p3d.y = sp.y_position;
      p3d.z = sp.z_position;
      p3d.label = sp.layer;
      v.push_back(p3d);
    }
  }

  return v;
}

ScanData2208::ScanData2208() :
  IbeoTxMessage(true, false, false)
{}

void ScanData2208::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  scan_number = read_be<uint16_t>(in, 2, 0);
  scanner_type = read_be<uint16_t>(in, 2, 2);
  uint16_t scanner_status = read_be<uint16_t>(in, 2, 4);
  motor_on = ((scanner_status & 0x0001) > 0);
  laser_on = ((scanner_status & 0x0002) > 0);
  frequency_locked = ((scanner_status & 0x0004) > 0);
  motor_rotating_direction = ((scanner_status & 0x0100) > 0) ? COUNTER_CLOCKWISE : CLOCKWISE;
  angle_ticks_per_rotation = read_be<uint16_t>(in, 2, 6);
  scan_flags = read_be<uint32_t>(in, 4, 8);
  mounting_yaw_angle_ticks = read_be<int16_t>(in, 2, 12);
  mounting_pitch_angle_ticks = read_be<int16_t>(in, 2, 14);
  mounting_roll_angle_ticks = read_be<int16_t>(in, 2, 16);
  mounting_position_x = read_be<int16_t>(in, 2, 18);
  mounting_position_y = read_be<int16_t>(in, 2, 20);
  mounting_position_z = read_be<int16_t>(in, 2, 22);
  device_id = read_be<uint8_t>(in, 1, 50);
  scan_start_time = read_be<NTPTime>(in, 8, 52);
  scan_end_time = read_be<NTPTime>(in, 8, 60);
  start_angle_ticks = read_be<int16_t>(in, 2, 68);
  end_angle_ticks = read_be<int16_t>(in, 2, 70);
  subflags = read_be<uint8_t>(in, 1, 72);
  mirror_side = ((read_be<uint8_t>(in, 1, 73) & 0x01) > 0) ? REAR : FRONT;
  mirror_tilt = read_be<int16_t>(in, 2, 78);
  number_of_scan_points = read_be<uint16_t>(in, 2, 86);

  for (uint16_t i = 0; i < number_of_scan_points; i++)
  {
    ScanPoint2208 new_scan_point;
    new_scan_point.parse(&in[88 + i * 11]);
    scan_point_list.push_back(new_scan_point);
  }
}

std::vector<Point3DL> ScanData2208::get_scan_points()
{
  std::vector<Point3DL> v;
  double cm_to_m = 100.0;

  for (ScanPoint2208 sp : scan_point_list)
  {
    if (sp.echo == 0 && sp.layer < 4 && !sp.transparent_point && !sp.clutter_atmospheric && !sp.ground && !sp.dirt)
    {
      // Spherical coordinates to cartesian
      // This uses the following conventions (with right-hand rule applied):
      // phi = rotation about the Z axis (angle in radians from X = 0)
      // theta = angle away from Z+ (radians)
      // rho = radial distance (m)
      Point3DL p3d;
      double phi = ticks_to_angle(sp.horizontal_angle, angle_ticks_per_rotation);
      double theta;
      double rho = sp.radial_distance / cm_to_m;

      switch (sp.layer)
      {
      case 0:
        theta = (88.4 * (M_PI / 180.0));
        break;
      case 1:
        theta = (89.2 * (M_PI / 180.0));
        break;
      case 2:
        theta = (90.8 * (M_PI / 180.0));
        break;
      case 3:
        theta = (91.6 * (M_PI / 180.0));
        break;
      }

      p3d.x = rho * sin(theta) * cos(phi);
      p3d.y = rho * sin(theta) * sin(phi);
      p3d.z = rho * cos(theta);

      p3d.label = sp.layer;

      p3d.label = sp.layer;

      v.push_back(p3d);
    }
  }

  return v;
}

ObjectData2221::ObjectData2221() :
  IbeoTxMessage(false, true, true)
{}

void ObjectData2221::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  scan_start_timestamp = read_le<NTPTime>(in, 8, 0);
  number_of_objects = read_le<uint16_t>(in, 2, 8);

  uint32_t offset = 10;

  for (uint16_t i = 0; i < number_of_objects; i++)
  {
    Object2221 new_object;
    new_object.parse(&in[offset]);
    object_list.push_back(new_object);
    offset += 58;  // Size of object without contour points.
    offset += new_object.number_of_contour_points * 4;  // Size of object's contour points.
  }
}

std::vector<Point3D> ObjectData2221::get_contour_points()
{
  std::vector<Point3D> contour_points;

  for (Object2221 o : object_list)
  {
    for (Point2Di p2d : o.contour_point_list)
    {
      Point3D p3d;
      p3d.x = static_cast<double>(p2d.x) / 100.0;
      p3d.y = static_cast<double>(p2d.y) / 100.0;
      p3d.z = 0.0;
      contour_points.push_back(p3d);
    }
  }

  return contour_points;
}

std::vector<IbeoObject> ObjectData2221::get_objects()
{
  std::vector<IbeoObject> io_list;

  for (Object2221 o : object_list)
  {
    IbeoObject io;

    io.id = o.id;
    io.age = o.age;
    io.prediction_age = o.prediction_age;
    io.relative_timestamp = o.relative_timestamp;
    io.reference_point.x = o.reference_point.x / 100.0;  // In cm
    io.reference_point.y = o.reference_point.y / 100.0;  // In cm
    io.reference_point_sigma.x = o.reference_point_sigma.x / 100.0;  // In cm
    io.reference_point_sigma.y = o.reference_point_sigma.y / 100.0;  // In cm
    io.closest_point.x = o.closest_point.x / 100.0;  // In cm
    io.closest_point.y = o.closest_point.y / 100.0;  // In cm

    io.bounding_box_center.x = o.bounding_box_center.x / 100.0;  // In cm
    io.bounding_box_center.y = o.bounding_box_center.y / 100.0;  // In cm
    io.bounding_box_size.size_x = o.bounding_box_width / 100.0;  // In cm
    io.bounding_box_size.size_y = o.bounding_box_length / 100.0;  // In cm

    io.object_box_center.x = o.object_box_center.x / 100.0;  // In cm
    io.object_box_center.y = o.object_box_center.y / 100.0;  // In cm
    io.object_box_size.size_x = o.object_box_size.size_x / 100.0;  // In cm
    io.object_box_size.size_y = o.object_box_size.size_y / 100.0;  // In cm
    io.object_box_orientation = o.object_box_orientation / 100.0 * (M_PI / 180.0);  // In 1/100 degrees.

    io.absolute_velocity.x = o.absolute_velocity.x / 100.0;  // In cm/s
    io.absolute_velocity.y = o.absolute_velocity.y / 100.0;  // In cm/s
    io.absolute_velocity_sigma.x = o.absolute_velocity_sigma.size_x / 100.0;  // In cm/s
    io.absolute_velocity_sigma.y = o.absolute_velocity_sigma.size_y / 100.0;  // In cm/s
    io.relative_velocity.x = o.relative_velocity.x / 100.0;  // In cm/s
    io.relative_velocity.y = o.relative_velocity.y / 100.0;  // In cm/s

    io.classification = static_cast<uint16_t>(o.classification);
    io.classification_age = o.classification_age;
    io.classification_certainty = o.classification_certainty;

    io.number_of_contour_points = o.number_of_contour_points;

    io.contour_point_list = get_contour_points();

    io_list.push_back(io);
  }

  return io_list;
}

ObjectData2225::ObjectData2225() :
  IbeoTxMessage(false, true, true)
{}

void ObjectData2225::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  mid_scan_timestamp = read_be<NTPTime>(in, 8, 0);
  number_of_objects = read_be<uint16_t>(in, 2, 8);

  uint32_t offset = 10;

  for (uint16_t i = 0; i < number_of_objects; i++)
  {
    Object2225 new_object;
    new_object.parse(&in[offset]);
    object_list.push_back(new_object);
    offset += 132;                                      // Add size of single object without contour points.
    offset += new_object.number_of_contour_points * 8;  // Add size of just-parsed object's contour points.
  }
}

std::vector<Point3D> ObjectData2225::get_contour_points()
{
  std::vector<Point3D> contour_points;

  for (Object2225 o : object_list)
  {
    for (Point2Df p2d : o.contour_point_list)
    {
      Point3D p3d;
      p3d.x = static_cast<double>(p2d.x) / 100.0;
      p3d.y = static_cast<double>(p2d.y) / 100.0;
      p3d.z = 0.0;
      contour_points.push_back(p3d);
    }
  }

  return contour_points;
}

std::vector<IbeoObject> ObjectData2225::get_objects()
{
  std::vector<IbeoObject> io_list;

  for (Object2225 o : object_list)
  {
    IbeoObject io;
    io.id = o.id;
    io.age = o.age;
    io.prediction_age = 0;
    io.relative_timestamp = 0;
    io.object_coordinate_system = VEHICLE;

    io.classification = o.classification;
    io.classification_age = o.classification_age;
    io.classification_certainty = o.classification_certainty;

    io.bounding_box_center.x = o.bounding_box_center.x;
    io.bounding_box_center.y = o.bounding_box_center.y;
    io.bounding_box_size.size_x = o.bounding_box_size.x;
    io.bounding_box_size.size_y = o.bounding_box_size.y;

    io.object_box_center.x = o.object_box_center.x;
    io.object_box_center.y = o.object_box_center.y;
    io.object_box_size.size_x = o.bounding_box_size.x;
    io.object_box_size.size_y = o.bounding_box_size.y;
    io.object_box_orientation = o.yaw_angle;

    io.absolute_velocity.x = o.absolute_velocity.x;
    io.absolute_velocity.y = o.absolute_velocity.y;
    io.absolute_velocity_sigma.x = o.absolute_velocity_sigma.x;
    io.absolute_velocity_sigma.y = o.absolute_velocity_sigma.y;
    io.relative_velocity.x = o.relative_velocity.x;
    io.relative_velocity.y = o.relative_velocity.y;

    io.contour_point_list = get_contour_points();

    // In theory, this should still hold.
    io.closest_point.x = io.contour_point_list[o.closest_point_index].x;
    io.closest_point.y = io.contour_point_list[o.closest_point_index].y;

    io.number_of_contour_points = (uint16_t) io.contour_point_list.size();
    io_list.push_back(io);
  }

  return io_list;
}

ObjectData2270::ObjectData2270() :
  IbeoTxMessage(false, true, true)
{}

void ObjectData2270::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  start_scan_timestamp = read_le<NTPTime>(in, 8, 0);
  object_list_number = read_le<uint16_t>(in, 2, 8);
  number_of_objects = read_le<uint16_t>(in, 2, 10);

  uint32_t offset = 12;

  for (uint16_t i = 0; i < number_of_objects; i++)
  {
    Object2270 new_object;
    new_object.parse(&in[offset]);
    object_list.push_back(new_object);

    offset += 76;                                       // Add size of single object without contour points.
    offset += new_object.number_of_contour_points * 4;  // Add size of just-parsed object's contour points.
  }
}

std::vector<Point3D> ObjectData2270::get_contour_points()
{
  std::vector<Point3D> contour_points;

  for (Object2270 o : object_list)
  {
    for (Point2Di p2d : o.contour_point_list)
    {
      Point3D p3d;
      p3d.x = static_cast<double>(p2d.x) / 100.0;
      p3d.y = static_cast<double>(p2d.y) / 100.0;
      p3d.z = 0.0;
      contour_points.push_back(p3d);
    }
  }

  return contour_points;
}

std::vector<IbeoObject> ObjectData2270::get_objects()
{
  std::vector<IbeoObject> io_list;

  for (Object2270 o : object_list)
  {
    IbeoObject io;
    io.id = o.id;
    io.age = o.age;
    io.prediction_age = o.prediction_age;
    io.relative_timestamp = 0;

    io.object_coordinate_system = VEHICLE;

    io.reference_point_location = o.reference_point_location;
    io.reference_point.x = o.reference_point_position_x / 100.0;  // In cm
    io.reference_point.y = o.reference_point_position_y / 100.0;  // In cm
    io.reference_point_sigma.x = o.reference_point_position_sigma_x / 100.0;  // In cm
    io.reference_point_sigma.y = o.reference_point_position_sigma_y / 100.0;  // In cm

    io.object_box_size.size_x = o.object_box_length / 100.0;  // In cm
    io.object_box_size.size_y = o.object_box_width / 100.0;  // In cm
    io.object_box_orientation = o.object_box_orientation_angle / 100.0 * (M_PI / 180.0);  // In 1/100 of a degree

    float half_box_x = io.object_box_size.size_x / 2.0;
    float half_box_y = io.object_box_size.size_y / 2.0;

    switch (o.reference_point_location)
    {
    case COG:
      io.object_box_center.x = o.contour_points_cog_x / 100.0;  // In cm
      io.object_box_center.y = o.contour_points_cog_y / 100.0;  // In cm
      break;
    case TOP_FRONT_LEFT_CORNER:
      io.object_box_center.x = io.reference_point.x - half_box_x;
      io.object_box_center.y = io.reference_point.y - half_box_y;
      break;
    case TOP_FRONT_RIGHT_CORNER:
      io.object_box_center.x = io.reference_point.x - half_box_x;
      io.object_box_center.y = io.reference_point.y + half_box_y;
      break;
    case BOTTOM_REAR_RIGHT_CORNER:
      io.object_box_center.x = io.reference_point.x + half_box_x;
      io.object_box_center.y = io.reference_point.y + half_box_y;
      break;
    case BOTTOM_REAR_LEFT_CORNER:
      io.object_box_center.x = io.reference_point.x + half_box_x;
      io.object_box_center.y = io.reference_point.y - half_box_y;
      break;
    case CENTER_OF_TOP_FRONT_EDGE:
      io.object_box_center.x = io.reference_point.x - half_box_x;
      io.object_box_center.y = io.reference_point.y;
      break;
    case CENTER_OF_RIGHT_EDGE:
      io.object_box_center.x = io.reference_point.x;
      io.object_box_center.y = io.reference_point.y + half_box_y;
      break;
    case CENTER_OF_BOTTOM_REAR_EDGE:
      io.object_box_center.x = io.reference_point.x + half_box_x;
      io.object_box_center.y = io.reference_point.y;
      break;
    case CENTER_OF_LEFT_EDGE:
      io.object_box_center.x = io.reference_point.x;
      io.object_box_center.y = io.reference_point.y - half_box_y;
      break;
    case BOX_CENTER:
    case INVALID:
      io.object_box_center.x = io.reference_point.x;
      io.object_box_center.y = io.reference_point.y;
      break;
    }

    io.absolute_velocity.x = o.absolute_velocity_x / 100.0;  // In cm/s
    io.absolute_velocity.y = o.absolute_velocity_y / 100.0;  // In cm/s
    io.absolute_velocity_sigma.x = o.absolute_velocity_sigma_x / 100.0;  // In cm/s
    io.absolute_velocity_sigma.y = o.absolute_velocity_sigma_y / 100.0;  // In cm/s
    io.relative_velocity.x = o.relative_velocity_x / 100.0;  // In cm/s
    io.relative_velocity.y = o.relative_velocity_y / 100.0;  // In cm/s

    io.contour_point_list = get_contour_points();
    io.number_of_contour_points = (uint16_t) io.contour_point_list.size();

    io_list.push_back(io);
  }

  return io_list;
}

ObjectData2271::ObjectData2271() :
  IbeoTxMessage(false, true, true)
{}

void ObjectData2271::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  start_scan_timestamp = read_be<NTPTime>(in, 8, 0);
  scan_number = read_be<uint16_t>(in, 2, 8);
  number_of_objects = read_be<uint16_t>(in, 2, 18);

  uint32_t offset = 20;

  for (uint16_t i = 0; i < number_of_objects; i++)
  {
    Object2271 new_object;
    new_object.parse(&in[offset]);
    object_list.push_back(new_object);

    // calculate offset for the next object (i+1)
    offset += (118 + 4);  // Add size of single object without contour points.
    // Add size of just-parsed object's untracked contour points.
    offset += new_object.untracked_properties.number_of_contour_points * 8;
    // Add size of just-parsed object's tracked contour points.
    offset += new_object.tracked_properties.number_of_contour_points * 8;
  }
}

std::vector<Point3D> ObjectData2271::get_contour_points()
{
  std::vector<Point3D> points;

  double m_to_cm = 100.0;
  int i = 0;

  for (Object2271 o : object_list)
  {
    if (o.untracked_properties_available)
    {
      for (ContourPointSigma cp : o.untracked_properties.contour_point_list)
      {
        Point3D p;
        p.x = cp.x / m_to_cm;
        p.y = cp.y / m_to_cm;
        points.push_back(p);
      }
    }

    if (o.tracked_properties_available)
    {
      for (ContourPointSigma cp : o.tracked_properties.contour_point_list)
      {
        Point3D p;
        p.x = cp.x / m_to_cm;
        p.y = cp.y / m_to_cm;
        p.z = 0;
        points.push_back(p);
      }
    }

    i++;
  }

  return points;
}

std::vector<IbeoObject> ObjectData2271::get_objects()
{
  std::vector<IbeoObject> io_list;

  for (Object2271 o : object_list)
  {
    IbeoObject io;
    io.id = o.id;
    io.age = 0;
    io.prediction_age = 0;
    io.relative_timestamp = 0;
    io.bounding_box_size.size_x = 0;
    io.bounding_box_size.size_y = 0;
    io.object_box_orientation = 0;
    io.classification = UNCLASSIFIED;
    io.classification_age = 0;
    io.classification_certainty = 0;

    if (o.tracked_properties_available)
    {
      io.age = o.tracked_properties.object_age;
      io.relative_timestamp = o.tracked_properties.relative_time_of_measure;

      io.closest_point.x = o.tracked_properties.position_closest_point.x / 100.0;  // In cm
      io.closest_point.y = o.tracked_properties.position_closest_point.y / 100.0;  // In cm

      io.object_box_size.size_x = o.tracked_properties.object_box_size.x / 100.0;  // In cm
      io.object_box_size.size_y = o.tracked_properties.object_box_size.y / 100.0;  // In cm
      io.object_box_orientation = o.tracked_properties.object_box_orientation /
                                  100.0 * (M_PI / 180.0);  // In 1/100 of a degree

      float half_box_x = io.object_box_size.size_x / 2.0;
      float half_box_y = io.object_box_size.size_y / 2.0;

      switch (o.tracked_properties.tracking_point_location)
      {
      case TOP_FRONT_LEFT_CORNER:
        io.object_box_center.x = io.reference_point.x - half_box_x;
        io.object_box_center.y = io.reference_point.y - half_box_y;
        break;
      case TOP_FRONT_RIGHT_CORNER:
        io.object_box_center.x = io.reference_point.x - half_box_x;
        io.object_box_center.y = io.reference_point.y + half_box_y;
        break;
      case BOTTOM_REAR_RIGHT_CORNER:
        io.object_box_center.x = io.reference_point.x + half_box_x;
        io.object_box_center.y = io.reference_point.y + half_box_y;
        break;
      case BOTTOM_REAR_LEFT_CORNER:
        io.object_box_center.x = io.reference_point.x + half_box_x;
        io.object_box_center.y = io.reference_point.y - half_box_y;
        break;
      case CENTER_OF_TOP_FRONT_EDGE:
        io.object_box_center.x = io.reference_point.x - half_box_x;
        io.object_box_center.y = io.reference_point.y;
        break;
      case CENTER_OF_RIGHT_EDGE:
        io.object_box_center.x = io.reference_point.x;
        io.object_box_center.y = io.reference_point.y + half_box_y;
        break;
      case CENTER_OF_BOTTOM_REAR_EDGE:
        io.object_box_center.x = io.reference_point.x + half_box_x;
        io.object_box_center.y = io.reference_point.y;
        break;
      case CENTER_OF_LEFT_EDGE:
        io.object_box_center.x = io.reference_point.x;
        io.object_box_center.y = io.reference_point.y - half_box_y;
        break;
      case COG:
      case BOX_CENTER:
      case INVALID:
        io.object_box_center.x = io.reference_point.x;
        io.object_box_center.y = io.reference_point.y;
        break;
      }

      io.absolute_velocity.x = o.tracked_properties.velocity.x / 100.0;  // In cm/s
      io.absolute_velocity.y = o.tracked_properties.velocity.y / 100.0;  // In cm/s
      io.absolute_velocity_sigma.x = o.tracked_properties.velocity_sigma.x / 100.0;  // In cm/s
      io.absolute_velocity_sigma.y = o.tracked_properties.velocity_sigma.y / 100.0;  // In cm/s
      io.relative_velocity.x = o.tracked_properties.relative_velocity.x / 100.0;  // In cm/s
      io.relative_velocity.y = o.tracked_properties.relative_velocity.y / 100.0;  // In cm/s

      io.classification = o.tracked_properties.classification;
      io.classification_age = o.tracked_properties.classification_age;
    }

    if (o.untracked_properties_available)
    {
      io.relative_timestamp = o.untracked_properties.relative_time_of_measurement;

      io.closest_point.x = o.untracked_properties.position_closest_point.x / 100.0;  // In cm
      io.closest_point.y = o.untracked_properties.position_closest_point.y / 100.0;  // In cm

      io.object_box_size.size_x = o.untracked_properties.object_box_size.x / 100.0;  // In cm
      io.object_box_size.size_y = o.untracked_properties.object_box_size.y / 100.0;  // In cm
      io.object_box_orientation = o.untracked_properties.object_box_orientation /
                                  100.0 * (M_PI / 180.0);  // In 1/100 of a degree

      io.object_box_center.x = o.untracked_properties.tracking_point_coordinate.x / 100.0;  // In cm
      io.object_box_center.y = o.untracked_properties.tracking_point_coordinate.y / 100.0;  // In cm

      io.reference_point.x = o.untracked_properties.tracking_point_coordinate.x / 100.0;  // In cm
      io.reference_point.y = o.untracked_properties.tracking_point_coordinate.y / 100.0;  // In cm

      io.reference_point_sigma.x = o.untracked_properties.tracking_point_coordinate_sigma.x / 100.0;  // In cm
      io.reference_point_sigma.y = o.untracked_properties.tracking_point_coordinate_sigma.y / 100.0;  // In cm
    }

    io.contour_point_list = get_contour_points();

    io.number_of_contour_points = (uint16_t) io.contour_point_list.size();
    io_list.push_back(io);
  }

  return io_list;
}

ObjectData2280::ObjectData2280() :
  IbeoTxMessage(false, true, true)
{}

void ObjectData2280::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  mid_scan_timestamp = read_be<NTPTime>(in, 8, 0);
  number_of_objects = read_be<uint16_t>(in, 2, 8);

  uint32_t offset = 10;

  for (uint16_t i = 0; i < number_of_objects; i++)
  {
    Object2280 new_object;
    new_object.parse(&in[offset]);

    object_list.push_back(new_object);

    offset += 168;                                      // Add size of single object without contour points.
    offset += new_object.number_of_contour_points * 8;  // Add size of just-parsed object's contour points.
  }
}

std::vector<Point3D> ObjectData2280::get_contour_points()
{
  std::vector<Point3D> contour_points;

  for (Object2280 o : object_list)
  {
    for (Point2Df p2d : o.contour_point_list)
    {
      if (!std::isnan(p2d.x) && !std::isnan(p2d.y) && fabs(p2d.x) < 300.0 && fabs(p2d.y) < 300.0)
      {
        Point3D p3d;
        p3d.x = static_cast<double>(p2d.x);
        p3d.y = static_cast<double>(p2d.y);
        p3d.z = 0.0;
        contour_points.push_back(p3d);
      }
    }
  }

  return contour_points;
}

std::vector<IbeoObject> ObjectData2280::get_objects()
{
  std::vector<IbeoObject> io_list;

  for (Object2280 o : object_list)
  {
    IbeoObject io;
    io.id = o.id;
    io.age = o.object_age;
    io.prediction_age = o.object_prediction_age;
    io.classification = o.classification;
    io.classification_age = o.classification_age;
    io.classification_certainty = o.classification_certainty;

    io.relative_timestamp = 0;
    io.bounding_box_size.size_x = o.object_box_size.x;
    io.bounding_box_size.size_y = o.object_box_size.y;
    io.object_box_orientation = o.object_box_orientation_angle;

    io.object_box_center.x = o.object_box_center.x;
    io.object_box_center.y = o.object_box_center.y;

    io.absolute_velocity.x = o.absolute_velocity.x;
    io.absolute_velocity.y = o.absolute_velocity.y;

    io.absolute_velocity_sigma.x = o.absolute_velocity_sigma.x;
    io.absolute_velocity_sigma.y = o.absolute_velocity_sigma.y;

    io.relative_velocity.x = o.relative_velocity.x;
    io.relative_velocity.y = o.relative_velocity.y;

    io.contour_point_list = get_contour_points();
    io.number_of_contour_points = o.number_of_contour_points;

    io.closest_point.x = io.contour_point_list[o.closest_point_index].x;
    io.closest_point.y = io.contour_point_list[o.closest_point_index].y;

    io_list.push_back(io);
  }

  return io_list;
}

CameraImage::CameraImage() :
  IbeoTxMessage()
{}

void CameraImage::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  image_format = static_cast<ImageFormat>(read_be<uint16_t>(in, 2, 0));
  us_since_power_on = read_be<uint32_t>(in, 4, 2);
  timestamp = read_be<NTPTime>(in, 8, 6);
  device_id = read_be<uint8_t>(in, 1, 14);
  mounting_position.parse(&in[15]);
  horizontal_opening_angle = read_be<double>(in, 8, 39);
  vertical_opening_angle = read_be<double>(in, 8, 47);
  image_width = read_be<uint16_t>(in, 2, 55);
  image_height = read_be<uint16_t>(in, 2, 57);
  compressed_size = read_be<uint32_t>(in, 4, 59);

  for (uint32_t i = 0; i < compressed_size; i++)
  {
    image_buffer.push_back(in[63 + i]);
  }
}

HostVehicleState2805::HostVehicleState2805() :
  IbeoTxMessage()
{}

void HostVehicleState2805::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  timestamp = read_le<NTPTime>(in, 8, 0);
  scan_number = read_le<uint16_t>(in, 2, 8);
  error_flags = read_le<uint16_t>(in, 2, 10);
  longitudinal_velocity = read_le<int16_t>(in, 2, 12);
  steering_wheel_angle = read_le<int16_t>(in, 2, 14);
  front_wheel_angle = read_le<int16_t>(in, 2, 16);
  x_position = read_le<int32_t>(in, 4, 20);
  y_position = read_le<int32_t>(in, 4, 24);
  course_angle = read_le<int16_t>(in, 2, 28);
  time_difference = read_le<uint16_t>(in, 2, 30);
  x_difference = read_le<int16_t>(in, 2, 32);
  y_difference = read_le<int16_t>(in, 2, 34);
  heading_difference = read_le<int16_t>(in, 2, 36);
  current_yaw_rate = read_le<int16_t>(in, 2, 40);
}

HostVehicleState2806::HostVehicleState2806() :
  IbeoTxMessage()
{}

void HostVehicleState2806::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  timestamp = read_le<NTPTime>(in, 8, 4);
  distance_x = read_le<int32_t>(in, 4, 12);
  distance_y = read_le<int32_t>(in, 4, 16);
  course_angle = read_le<float>(in, 4, 24);
  longitudinal_velocity = read_le<float>(in, 4, 28);
  yaw_rate = read_le<float>(in, 4, 24);
  steering_wheel_angle = read_le<float>(in, 4, 32);
  cross_acceleration = read_le<float>(in, 4, 36);
  front_wheel_angle = read_le<float>(in, 4, 40);
  vehicle_width = read_le<float>(in, 4, 46);
  vehicle_front_to_front_axle = read_le<float>(in, 4, 54);
  rear_axle_to_front_axle = read_le<float>(in, 4, 58);
  rear_axle_to_vehicle_rear = read_le<float>(in, 4, 62);
  steer_ratio_poly_0 = read_le<float>(in, 4, 70);
  steer_ratio_poly_1 = read_le<float>(in, 4, 74);
  steer_ratio_poly_2 = read_le<float>(in, 4, 78);
  steer_ratio_poly_3 = read_le<float>(in, 4, 82);
}

HostVehicleState2807::HostVehicleState2807() :
  HostVehicleState2806()
{}

void HostVehicleState2807::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  timestamp = read_le<NTPTime>(in, 8, 4);
  distance_x = read_le<int32_t>(in, 4, 12);
  distance_y = read_le<int32_t>(in, 4, 16);
  course_angle = read_le<float>(in, 4, 24);
  longitudinal_velocity = read_le<float>(in, 4, 28);
  yaw_rate = read_le<float>(in, 4, 24);
  steering_wheel_angle = read_le<float>(in, 4, 32);
  cross_acceleration = read_le<float>(in, 4, 36);
  front_wheel_angle = read_le<float>(in, 4, 40);
  vehicle_width = read_le<float>(in, 4, 46);
  vehicle_front_to_front_axle = read_le<float>(in, 4, 54);
  rear_axle_to_front_axle = read_le<float>(in, 4, 58);
  rear_axle_to_vehicle_rear = read_le<float>(in, 4, 62);
  steer_ratio_poly_0 = read_le<float>(in, 4, 70);
  steer_ratio_poly_1 = read_le<float>(in, 4, 74);
  steer_ratio_poly_2 = read_le<float>(in, 4, 78);
  steer_ratio_poly_3 = read_le<float>(in, 4, 82);

  longitudinal_acceleration = read_le<float>(in, 4, 86 + IBEO_HEADER_SIZE);
}

DeviceStatus::DeviceStatus() :
  IbeoTxMessage()
{}

void DeviceStatus::parse(uint8_t *in)
{
  ibeo_header.parse(in);

  in = &in[IBEO_HEADER_SIZE];

  scanner_type = read_le<uint8_t>(in, 1, 6);
  sensor_temperature = read_le<float>(in, 4, 36);
  frequency = read_le<float>(in, 4, 40);
}

void CommandSetFilter::encode()
{
  ibeo_header.message_size = 8;
  ibeo_header.data_type_id = 0x2010;

  struct timeval tv;
  struct tm *tm;
  gettimeofday(&tv, NULL);
  tm = localtime(&tv.tv_sec);  // NOLINT
  ibeo_header.time = unix_time_to_ntp(tm, &tv);
  ibeo_header.encode();

  encoded_data.insert(encoded_data.end(), ibeo_header.encoded_data.begin(), ibeo_header.encoded_data.end());

  command_identifier = 0x0005;
  version = 0x0002;
  begin_filter_range = 0x0000;
  end_filter_range = 0xFFFF;

  std::vector<uint8_t> encoded_command_identifier = write_be<uint16_t>(&command_identifier);
  std::vector<uint8_t> encoded_version = write_be<uint16_t>(&version);
  std::vector<uint8_t> encoded_begin_filter_range = write_be<uint16_t>(&begin_filter_range);
  std::vector<uint8_t> encoded_end_filter_range = write_be<uint16_t>(&end_filter_range);

  encoded_data.insert(encoded_data.end(), encoded_command_identifier.begin(), encoded_command_identifier.end());
  encoded_data.insert(encoded_data.end(), encoded_version.begin(), encoded_version.end());
  encoded_data.insert(encoded_data.end(), encoded_begin_filter_range.begin(), encoded_begin_filter_range.end());
  encoded_data.insert(encoded_data.end(), encoded_end_filter_range.begin(), encoded_end_filter_range.end());
}
