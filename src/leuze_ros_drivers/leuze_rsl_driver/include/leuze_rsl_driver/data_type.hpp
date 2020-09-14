#ifndef LEUZE_DATA_TYPE_H
#define LEUZE_DATA_TYPE_H

#include <cstdio>

struct PacketHeader
{
    virtual std::size_t get_header_size() = 0;
    virtual std::size_t get_packet_size() = 0;
};


struct Frame
    /** The logical organisation of the structs is based on the UDP Specification   (available in the leuze website)
     *  Frame includes 2 header and 3 other information fields
     */

{
  struct Header1
  {
    uint32_t total_length;
    uint8_t header_size;
    uint8_t follow_flag;
    uint16_t request_id;
  };

  struct Header2
  {
    uint8_t size;
    uint8_t destination;
    uint8_t source;
    uint8_t telegram_type;
  };

  Header1 h1;
  Header2 h2;
  uint16_t id;
  uint16_t block;
  uint32_t scan_number;
};

struct DatagramExtendedStatusProfile
{
  struct StatusProfile
  {
    uint8_t byte_0;
    uint8_t byte_1;
    uint8_t msg_and_ossd_2;
    uint8_t emergency_stop_3;
    uint8_t electrical_signals_byte_4;
    uint8_t electrical_signals_byte_5;
    uint8_t electrical_signals_byte_6;
    uint8_t electrical_signals_byte_7;
    uint32_t scan_number;
    uint8_t protec_func_a_12;
    uint8_t fp_sel_a_byte_13;
    uint8_t fp_sel_a_byte_14;
    uint8_t indic_a_15;
    uint8_t protec_func_b_16;
//    uint16_t fp_sel_b;    // Using one 16t instead of two 8ts causes a problem in that the next bytes are not properly read (start,stop and interval). Need to figure out why
    uint8_t fp_sel_b_byte_17;
    uint8_t fp_sel_b_byte_18;
    uint8_t indic_b_19;
  };

  struct MeasurementContourDescription
  {
    uint16_t start_index;
    uint16_t stop_index;
    uint16_t index_interval;
    uint8_t reseved;
  };


  Frame frame;
  StatusProfile status_profile;
  MeasurementContourDescription measurement_contour_descritption;

  int getBeamCount()
  {
    return (1 + (int)ceil(
              (measurement_contour_descritption.stop_index - measurement_contour_descritption.start_index)
              /(double)measurement_contour_descritption.index_interval)
            );
    //As per the formula from UDP spec sheet pg 13 below table 3.4 as well as the expression used in udpstateimage.h_ex line 102
  }
};

// Assuming variable length datagrams, length to be determined runtime

struct DatagramMeasurementDataType
{
  Frame *frame;
  std::vector<uint16_t> data_distance;
  std::vector<uint16_t> data_signal_strength;
};




#endif
