// Copyright (c) 2019 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_SCANNER_FRAMES_H
#define PSEN_SCAN_SCANNER_FRAMES_H

#include <cstdint>
#include <array>

#include <psen_scan/psen_scan_internal_angle.h>

namespace psen_scan
{
#pragma pack(push, 1)  // Don't allow padding

/**
 * @brief Frame containing all necessary fields for a Start Monitoring Command
 *
 */
typedef struct StartMonitoringFrame
{
  uint32_t crc_;                  /**< A CRC32 of all the following fields. Byte order: little endian */
  uint32_t const RESERVED_;       /**< Use all zeros */
  char password_[8];              /**< Device Password in ASCII, filled with zeros for non-existing characters */
  uint32_t const OPCODE_;         /**< Constant 0x09. Byte order: little endian */
  uint32_t host_ip_;              /** Target IP address. Byte order: big endian */
  uint32_t host_udp_port_;        /** Target UDP port. Byte order: little endian */
  uint64_t const FIXED_SEQUENCE_; /**< Use byte sequence 10001000 */
  std::array<uint16_t, 12> const RESERVED2_; /**< Use all zeros */

  StartMonitoringFrame(const std::string& password = "",
                       const uint32_t& host_ip = 0,
                       const uint32_t& host_udp_port = 0);

} StartMonitoringFrame;

/**
 * @brief Frame containing all necessary fields for a Stop Monitoring Command
 *
 */
typedef struct StopMonitoringFrame
{
  uint32_t crc_;            /**< A CRC32 of all the following fields. Byte order: little endian */
  uint32_t const RESERVED_; /**< Use all zeros */
  char password_[8];        /**< Device Password in ASCII, filled with zeros for non-existing characters */
  uint32_t const OPCODE_;   /**< Constant 0x12. Byte order: little endian */

  StopMonitoringFrame(const std::string& password = "");

} StopMonitoringFrame;

/**
 * @brief Physical Inputs Field for InputStateArea
 *
 */
typedef struct PhysicalInputs
{
  uint32_t time_stamp_;                   /**< Incremental counter. Byte order: little endian */
  std::array<uint8_t, 10> input_signals_; /**< Byte array representing the physical input values */
} PhysicalInputs;

/**
 * @brief Logical Inputs Field for InputStateArea
 *
 */
typedef struct LogicalInputs
{
  uint32_t time_stamp_;                  /**< Incremental counter. Byte order: little endian */
  std::array<uint8_t, 8> input_signals_; /**< Byte array representing logical input values */
} LogicalInputs;

/**
 * @brief InputStateArea Field for MonitoringFrame
 *
 */
typedef struct InputStateArea
{
  std::array<PhysicalInputs, 3> physical_inputs_;
  LogicalInputs logical_inputs_;
} InputStateArea;

/**
 * @brief OutputStateArea Field for MonitoringFrame
 *
 */
typedef struct OutputStateArea
{
  uint32_t time_stamp_; /**< Incremental counter. Byte order: little endian */
  uint32_t outputs_;    /**< Bitmask representing output values */
} OutputStateArea;

/**
 * @brief DiagnosticInformation Bitfield for DiagnosticArea
 *
 */
typedef struct DiagnosticInformation
{
  uint8_t internal_error_1_ : 5;
  uint8_t integrity_check_problem_on_any_ossd_ : 1;
  uint8_t short_circuit_at_least_two_ossd_ : 1;
  uint8_t ossd1_short_circuit_ : 1;

  uint8_t : 2;
  uint8_t internal_error_2_ : 2;
  uint8_t dust_circuit_failure_ : 1;
  uint8_t network_problem_ : 1;
  uint8_t power_supply_problem_ : 1;
  uint8_t window_cleaning_alarm_ : 1;

  uint8_t window_cleaning_warning_ : 1;
  uint8_t zone_invalid_input_configuration_connection_ : 1;
  uint8_t zone_invalid_input_transition_or_integrity_ : 1;
  uint8_t incoherence_data_ : 1;
  uint8_t internal_error_3_ : 3;
  uint8_t measure_problem_ : 1;

  uint8_t temperature_measurement_problem_ : 1;
  uint8_t internal_error_5_ : 2;
  uint8_t display_communication_problem_ : 1;
  uint8_t generic_error_ : 1;
  uint8_t internal_error_4_ : 2;
  uint8_t internal_communication_problem_ : 1;

  uint8_t temperature_range_error_ : 1;
  uint8_t out_of_range_error_ : 1;
  uint8_t configuration_error_ : 1;
  uint8_t : 5;

  std::array<uint8_t, 15> unused;
} DiagnosticInformation;

/**
 * @brief DiagnosticArea Field for MonitoringFrame
 *
 */
typedef struct DiagnosticArea
{
  uint32_t time_stamp_; /**< Incremental counter */
  DiagnosticInformation diagnostic_information_;
} DiagnosticArea;

/**
 * @brief MonitoringFrame as coming from Laserscanner
 *
 */
typedef struct MonitoringFrame
{
  uint32_t device_status_;            /**< Bit mask representing the device status. Byte order: little endian */
  uint32_t opcode_;                   /**< Constant 0xC9. Byte order: little endian */
  uint32_t working_mode_;             /**< Online = 0x00, Offline Test = 0x02, Byte order: little endian */
  uint32_t transaction_type_;         /**< GUI monitoring transaction = 0x05. Byte order: little endian */
  InputStateArea input_state_area_;   /**< Area representing the state of the inputs */
  OutputStateArea output_state_area_; /**< Area representing the state of the outputs */
  DiagnosticArea diagnostic_area_;    /**< Area representing diagnostic fault errors */
  uint8_t scanner_id_;                /**< 0 = master/standalone, [1..3] = slaves */
  uint8_t resolution_;                /**< Angle between measures in tenths of degree */
  /**< The initial angle in tenths of degree. Angle increases left to right clockwise. Zero is leftmost. Byte order:
   * little endian */
  uint16_t from_theta_;
  /**< The number of samples in the "measures" field below. Max value is 550 (0x226). Byte order: little endian */
  uint16_t number_of_samples_;
  /**< An array of little endian 16-bit unsigned integer representing measures in [mm]. The actual number of samples is
   * in the "number_of_samples" field above. Remaining samples are meaningless */
  std::array<uint16_t, 550> measures_;
  /**< Counter indicating the number of rounds that the motor has performed since power-up. This field is only available
   * from Version 2.1.0 or later. Byte order: little endian */
  uint32_t scan_counter_;
  PSENscanInternalAngle to_theta() const
  {
    return PSENscanInternalAngle(from_theta_ + static_cast<uint16_t>(resolution_) * number_of_samples_);
  }
} MonitoringFrame;

#pragma pack(pop)
}  // namespace psen_scan

#endif  // PSEN_SCAN_SCANNER_FRAMES_H