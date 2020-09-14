# The header
Header header

# The Status Profile part of the message
# Refer UDP specification pg 19 Table 3.3
uint8 byte_0
uint8 byte_1
uint8 msg_and_ossd_2
uint8 emergency_stop_3
uint8 electrical_signals_byte_4
uint8 electrical_signals_byte_5
uint8 electrical_signals_byte_6
uint8 electrical_signals_byte_7
uint32 scan_number
uint8 protec_func_a_12
uint8 fp_sel_a_byte_13
uint8 fp_sel_a_byte_14
uint8 indic_a_15
uint8 protec_func_b_16
uint8 fp_sel_b_byte_17
uint8 fp_sel_b_byte_18
uint8 indic_b_19

#The Measurement Contour Description part of the message
# Refer UDP specification pg 13 Table 3.4
uint16 start_index
uint16 stop_index
uint16 index_interval
uint16 reserved
