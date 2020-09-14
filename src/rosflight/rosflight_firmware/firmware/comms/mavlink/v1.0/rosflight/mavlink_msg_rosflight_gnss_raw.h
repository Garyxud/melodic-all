// MESSAGE ROSFLIGHT_GNSS_RAW PACKING

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW 198

typedef struct __mavlink_rosflight_gnss_raw_t
{
 uint64_t rosflight_timestamp; /*< */
 uint32_t time_of_week; /*< */
 uint32_t t_acc; /*< */
 int32_t nano; /*< */
 int32_t lon; /*< */
 int32_t lat; /*< */
 int32_t height; /*< */
 int32_t height_msl; /*< */
 uint32_t h_acc; /*< */
 uint32_t v_acc; /*< */
 int32_t vel_n; /*< */
 int32_t vel_e; /*< */
 int32_t vel_d; /*< */
 int32_t g_speed; /*< */
 int32_t head_mot; /*< */
 uint32_t s_acc; /*< */
 uint32_t head_acc; /*< */
 uint16_t year; /*< */
 uint16_t p_dop; /*< */
 uint8_t month; /*< */
 uint8_t day; /*< */
 uint8_t hour; /*< */
 uint8_t min; /*< */
 uint8_t sec; /*< */
 uint8_t valid; /*< */
 uint8_t fix_type; /*< */
 uint8_t num_sat; /*< */
} mavlink_rosflight_gnss_raw_t;

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN 84
#define MAVLINK_MSG_ID_198_LEN 84

#define MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC 84
#define MAVLINK_MSG_ID_198_CRC 84



#define MAVLINK_MESSAGE_INFO_ROSFLIGHT_GNSS_RAW { \
	"ROSFLIGHT_GNSS_RAW", \
	27, \
	{  { "rosflight_timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_rosflight_gnss_raw_t, rosflight_timestamp) }, \
         { "time_of_week", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_rosflight_gnss_raw_t, time_of_week) }, \
         { "t_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_rosflight_gnss_raw_t, t_acc) }, \
         { "nano", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_rosflight_gnss_raw_t, nano) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_rosflight_gnss_raw_t, lon) }, \
         { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 24, offsetof(mavlink_rosflight_gnss_raw_t, lat) }, \
         { "height", NULL, MAVLINK_TYPE_INT32_T, 0, 28, offsetof(mavlink_rosflight_gnss_raw_t, height) }, \
         { "height_msl", NULL, MAVLINK_TYPE_INT32_T, 0, 32, offsetof(mavlink_rosflight_gnss_raw_t, height_msl) }, \
         { "h_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_rosflight_gnss_raw_t, h_acc) }, \
         { "v_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 40, offsetof(mavlink_rosflight_gnss_raw_t, v_acc) }, \
         { "vel_n", NULL, MAVLINK_TYPE_INT32_T, 0, 44, offsetof(mavlink_rosflight_gnss_raw_t, vel_n) }, \
         { "vel_e", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_rosflight_gnss_raw_t, vel_e) }, \
         { "vel_d", NULL, MAVLINK_TYPE_INT32_T, 0, 52, offsetof(mavlink_rosflight_gnss_raw_t, vel_d) }, \
         { "g_speed", NULL, MAVLINK_TYPE_INT32_T, 0, 56, offsetof(mavlink_rosflight_gnss_raw_t, g_speed) }, \
         { "head_mot", NULL, MAVLINK_TYPE_INT32_T, 0, 60, offsetof(mavlink_rosflight_gnss_raw_t, head_mot) }, \
         { "s_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 64, offsetof(mavlink_rosflight_gnss_raw_t, s_acc) }, \
         { "head_acc", NULL, MAVLINK_TYPE_UINT32_T, 0, 68, offsetof(mavlink_rosflight_gnss_raw_t, head_acc) }, \
         { "year", NULL, MAVLINK_TYPE_UINT16_T, 0, 72, offsetof(mavlink_rosflight_gnss_raw_t, year) }, \
         { "p_dop", NULL, MAVLINK_TYPE_UINT16_T, 0, 74, offsetof(mavlink_rosflight_gnss_raw_t, p_dop) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_rosflight_gnss_raw_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_rosflight_gnss_raw_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_rosflight_gnss_raw_t, hour) }, \
         { "min", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_rosflight_gnss_raw_t, min) }, \
         { "sec", NULL, MAVLINK_TYPE_UINT8_T, 0, 80, offsetof(mavlink_rosflight_gnss_raw_t, sec) }, \
         { "valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 81, offsetof(mavlink_rosflight_gnss_raw_t, valid) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 82, offsetof(mavlink_rosflight_gnss_raw_t, fix_type) }, \
         { "num_sat", NULL, MAVLINK_TYPE_UINT8_T, 0, 83, offsetof(mavlink_rosflight_gnss_raw_t, num_sat) }, \
         } \
}


/**
 * @brief Pack a rosflight_gnss_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_of_week 
 * @param year 
 * @param month 
 * @param day 
 * @param hour 
 * @param min 
 * @param sec 
 * @param valid 
 * @param t_acc 
 * @param nano 
 * @param fix_type 
 * @param num_sat 
 * @param lon 
 * @param lat 
 * @param height 
 * @param height_msl 
 * @param h_acc 
 * @param v_acc 
 * @param vel_n 
 * @param vel_e 
 * @param vel_d 
 * @param g_speed 
 * @param head_mot 
 * @param s_acc 
 * @param head_acc 
 * @param p_dop 
 * @param rosflight_timestamp 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_of_week, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t valid, uint32_t t_acc, int32_t nano, uint8_t fix_type, uint8_t num_sat, int32_t lon, int32_t lat, int32_t height, int32_t height_msl, uint32_t h_acc, uint32_t v_acc, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t g_speed, int32_t head_mot, uint32_t s_acc, uint32_t head_acc, uint16_t p_dop, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, rosflight_timestamp);
	_mav_put_uint32_t(buf, 8, time_of_week);
	_mav_put_uint32_t(buf, 12, t_acc);
	_mav_put_int32_t(buf, 16, nano);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, height);
	_mav_put_int32_t(buf, 32, height_msl);
	_mav_put_uint32_t(buf, 36, h_acc);
	_mav_put_uint32_t(buf, 40, v_acc);
	_mav_put_int32_t(buf, 44, vel_n);
	_mav_put_int32_t(buf, 48, vel_e);
	_mav_put_int32_t(buf, 52, vel_d);
	_mav_put_int32_t(buf, 56, g_speed);
	_mav_put_int32_t(buf, 60, head_mot);
	_mav_put_uint32_t(buf, 64, s_acc);
	_mav_put_uint32_t(buf, 68, head_acc);
	_mav_put_uint16_t(buf, 72, year);
	_mav_put_uint16_t(buf, 74, p_dop);
	_mav_put_uint8_t(buf, 76, month);
	_mav_put_uint8_t(buf, 77, day);
	_mav_put_uint8_t(buf, 78, hour);
	_mav_put_uint8_t(buf, 79, min);
	_mav_put_uint8_t(buf, 80, sec);
	_mav_put_uint8_t(buf, 81, valid);
	_mav_put_uint8_t(buf, 82, fix_type);
	_mav_put_uint8_t(buf, 83, num_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#else
	mavlink_rosflight_gnss_raw_t packet;
	packet.rosflight_timestamp = rosflight_timestamp;
	packet.time_of_week = time_of_week;
	packet.t_acc = t_acc;
	packet.nano = nano;
	packet.lon = lon;
	packet.lat = lat;
	packet.height = height;
	packet.height_msl = height_msl;
	packet.h_acc = h_acc;
	packet.v_acc = v_acc;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.g_speed = g_speed;
	packet.head_mot = head_mot;
	packet.s_acc = s_acc;
	packet.head_acc = head_acc;
	packet.year = year;
	packet.p_dop = p_dop;
	packet.month = month;
	packet.day = day;
	packet.hour = hour;
	packet.min = min;
	packet.sec = sec;
	packet.valid = valid;
	packet.fix_type = fix_type;
	packet.num_sat = num_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
}

/**
 * @brief Pack a rosflight_gnss_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_of_week 
 * @param year 
 * @param month 
 * @param day 
 * @param hour 
 * @param min 
 * @param sec 
 * @param valid 
 * @param t_acc 
 * @param nano 
 * @param fix_type 
 * @param num_sat 
 * @param lon 
 * @param lat 
 * @param height 
 * @param height_msl 
 * @param h_acc 
 * @param v_acc 
 * @param vel_n 
 * @param vel_e 
 * @param vel_d 
 * @param g_speed 
 * @param head_mot 
 * @param s_acc 
 * @param head_acc 
 * @param p_dop 
 * @param rosflight_timestamp 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rosflight_gnss_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_of_week,uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec,uint8_t valid,uint32_t t_acc,int32_t nano,uint8_t fix_type,uint8_t num_sat,int32_t lon,int32_t lat,int32_t height,int32_t height_msl,uint32_t h_acc,uint32_t v_acc,int32_t vel_n,int32_t vel_e,int32_t vel_d,int32_t g_speed,int32_t head_mot,uint32_t s_acc,uint32_t head_acc,uint16_t p_dop,uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, rosflight_timestamp);
	_mav_put_uint32_t(buf, 8, time_of_week);
	_mav_put_uint32_t(buf, 12, t_acc);
	_mav_put_int32_t(buf, 16, nano);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, height);
	_mav_put_int32_t(buf, 32, height_msl);
	_mav_put_uint32_t(buf, 36, h_acc);
	_mav_put_uint32_t(buf, 40, v_acc);
	_mav_put_int32_t(buf, 44, vel_n);
	_mav_put_int32_t(buf, 48, vel_e);
	_mav_put_int32_t(buf, 52, vel_d);
	_mav_put_int32_t(buf, 56, g_speed);
	_mav_put_int32_t(buf, 60, head_mot);
	_mav_put_uint32_t(buf, 64, s_acc);
	_mav_put_uint32_t(buf, 68, head_acc);
	_mav_put_uint16_t(buf, 72, year);
	_mav_put_uint16_t(buf, 74, p_dop);
	_mav_put_uint8_t(buf, 76, month);
	_mav_put_uint8_t(buf, 77, day);
	_mav_put_uint8_t(buf, 78, hour);
	_mav_put_uint8_t(buf, 79, min);
	_mav_put_uint8_t(buf, 80, sec);
	_mav_put_uint8_t(buf, 81, valid);
	_mav_put_uint8_t(buf, 82, fix_type);
	_mav_put_uint8_t(buf, 83, num_sat);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#else
	mavlink_rosflight_gnss_raw_t packet;
	packet.rosflight_timestamp = rosflight_timestamp;
	packet.time_of_week = time_of_week;
	packet.t_acc = t_acc;
	packet.nano = nano;
	packet.lon = lon;
	packet.lat = lat;
	packet.height = height;
	packet.height_msl = height_msl;
	packet.h_acc = h_acc;
	packet.v_acc = v_acc;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.g_speed = g_speed;
	packet.head_mot = head_mot;
	packet.s_acc = s_acc;
	packet.head_acc = head_acc;
	packet.year = year;
	packet.p_dop = p_dop;
	packet.month = month;
	packet.day = day;
	packet.hour = hour;
	packet.min = min;
	packet.sec = sec;
	packet.valid = valid;
	packet.fix_type = fix_type;
	packet.num_sat = num_sat;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
}

/**
 * @brief Encode a rosflight_gnss_raw struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_gnss_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_gnss_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rosflight_gnss_raw_t* rosflight_gnss_raw)
{
	return mavlink_msg_rosflight_gnss_raw_pack(system_id, component_id, msg, rosflight_gnss_raw->time_of_week, rosflight_gnss_raw->year, rosflight_gnss_raw->month, rosflight_gnss_raw->day, rosflight_gnss_raw->hour, rosflight_gnss_raw->min, rosflight_gnss_raw->sec, rosflight_gnss_raw->valid, rosflight_gnss_raw->t_acc, rosflight_gnss_raw->nano, rosflight_gnss_raw->fix_type, rosflight_gnss_raw->num_sat, rosflight_gnss_raw->lon, rosflight_gnss_raw->lat, rosflight_gnss_raw->height, rosflight_gnss_raw->height_msl, rosflight_gnss_raw->h_acc, rosflight_gnss_raw->v_acc, rosflight_gnss_raw->vel_n, rosflight_gnss_raw->vel_e, rosflight_gnss_raw->vel_d, rosflight_gnss_raw->g_speed, rosflight_gnss_raw->head_mot, rosflight_gnss_raw->s_acc, rosflight_gnss_raw->head_acc, rosflight_gnss_raw->p_dop, rosflight_gnss_raw->rosflight_timestamp);
}

/**
 * @brief Encode a rosflight_gnss_raw struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rosflight_gnss_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rosflight_gnss_raw_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rosflight_gnss_raw_t* rosflight_gnss_raw)
{
	return mavlink_msg_rosflight_gnss_raw_pack_chan(system_id, component_id, chan, msg, rosflight_gnss_raw->time_of_week, rosflight_gnss_raw->year, rosflight_gnss_raw->month, rosflight_gnss_raw->day, rosflight_gnss_raw->hour, rosflight_gnss_raw->min, rosflight_gnss_raw->sec, rosflight_gnss_raw->valid, rosflight_gnss_raw->t_acc, rosflight_gnss_raw->nano, rosflight_gnss_raw->fix_type, rosflight_gnss_raw->num_sat, rosflight_gnss_raw->lon, rosflight_gnss_raw->lat, rosflight_gnss_raw->height, rosflight_gnss_raw->height_msl, rosflight_gnss_raw->h_acc, rosflight_gnss_raw->v_acc, rosflight_gnss_raw->vel_n, rosflight_gnss_raw->vel_e, rosflight_gnss_raw->vel_d, rosflight_gnss_raw->g_speed, rosflight_gnss_raw->head_mot, rosflight_gnss_raw->s_acc, rosflight_gnss_raw->head_acc, rosflight_gnss_raw->p_dop, rosflight_gnss_raw->rosflight_timestamp);
}

/**
 * @brief Send a rosflight_gnss_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param time_of_week 
 * @param year 
 * @param month 
 * @param day 
 * @param hour 
 * @param min 
 * @param sec 
 * @param valid 
 * @param t_acc 
 * @param nano 
 * @param fix_type 
 * @param num_sat 
 * @param lon 
 * @param lat 
 * @param height 
 * @param height_msl 
 * @param h_acc 
 * @param v_acc 
 * @param vel_n 
 * @param vel_e 
 * @param vel_d 
 * @param g_speed 
 * @param head_mot 
 * @param s_acc 
 * @param head_acc 
 * @param p_dop 
 * @param rosflight_timestamp 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rosflight_gnss_raw_send(mavlink_channel_t chan, uint32_t time_of_week, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t valid, uint32_t t_acc, int32_t nano, uint8_t fix_type, uint8_t num_sat, int32_t lon, int32_t lat, int32_t height, int32_t height_msl, uint32_t h_acc, uint32_t v_acc, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t g_speed, int32_t head_mot, uint32_t s_acc, uint32_t head_acc, uint16_t p_dop, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN];
	_mav_put_uint64_t(buf, 0, rosflight_timestamp);
	_mav_put_uint32_t(buf, 8, time_of_week);
	_mav_put_uint32_t(buf, 12, t_acc);
	_mav_put_int32_t(buf, 16, nano);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, height);
	_mav_put_int32_t(buf, 32, height_msl);
	_mav_put_uint32_t(buf, 36, h_acc);
	_mav_put_uint32_t(buf, 40, v_acc);
	_mav_put_int32_t(buf, 44, vel_n);
	_mav_put_int32_t(buf, 48, vel_e);
	_mav_put_int32_t(buf, 52, vel_d);
	_mav_put_int32_t(buf, 56, g_speed);
	_mav_put_int32_t(buf, 60, head_mot);
	_mav_put_uint32_t(buf, 64, s_acc);
	_mav_put_uint32_t(buf, 68, head_acc);
	_mav_put_uint16_t(buf, 72, year);
	_mav_put_uint16_t(buf, 74, p_dop);
	_mav_put_uint8_t(buf, 76, month);
	_mav_put_uint8_t(buf, 77, day);
	_mav_put_uint8_t(buf, 78, hour);
	_mav_put_uint8_t(buf, 79, min);
	_mav_put_uint8_t(buf, 80, sec);
	_mav_put_uint8_t(buf, 81, valid);
	_mav_put_uint8_t(buf, 82, fix_type);
	_mav_put_uint8_t(buf, 83, num_sat);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
#else
	mavlink_rosflight_gnss_raw_t packet;
	packet.rosflight_timestamp = rosflight_timestamp;
	packet.time_of_week = time_of_week;
	packet.t_acc = t_acc;
	packet.nano = nano;
	packet.lon = lon;
	packet.lat = lat;
	packet.height = height;
	packet.height_msl = height_msl;
	packet.h_acc = h_acc;
	packet.v_acc = v_acc;
	packet.vel_n = vel_n;
	packet.vel_e = vel_e;
	packet.vel_d = vel_d;
	packet.g_speed = g_speed;
	packet.head_mot = head_mot;
	packet.s_acc = s_acc;
	packet.head_acc = head_acc;
	packet.year = year;
	packet.p_dop = p_dop;
	packet.month = month;
	packet.day = day;
	packet.hour = hour;
	packet.min = min;
	packet.sec = sec;
	packet.valid = valid;
	packet.fix_type = fix_type;
	packet.num_sat = num_sat;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, (const char *)&packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rosflight_gnss_raw_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_of_week, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint8_t valid, uint32_t t_acc, int32_t nano, uint8_t fix_type, uint8_t num_sat, int32_t lon, int32_t lat, int32_t height, int32_t height_msl, uint32_t h_acc, uint32_t v_acc, int32_t vel_n, int32_t vel_e, int32_t vel_d, int32_t g_speed, int32_t head_mot, uint32_t s_acc, uint32_t head_acc, uint16_t p_dop, uint64_t rosflight_timestamp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint64_t(buf, 0, rosflight_timestamp);
	_mav_put_uint32_t(buf, 8, time_of_week);
	_mav_put_uint32_t(buf, 12, t_acc);
	_mav_put_int32_t(buf, 16, nano);
	_mav_put_int32_t(buf, 20, lon);
	_mav_put_int32_t(buf, 24, lat);
	_mav_put_int32_t(buf, 28, height);
	_mav_put_int32_t(buf, 32, height_msl);
	_mav_put_uint32_t(buf, 36, h_acc);
	_mav_put_uint32_t(buf, 40, v_acc);
	_mav_put_int32_t(buf, 44, vel_n);
	_mav_put_int32_t(buf, 48, vel_e);
	_mav_put_int32_t(buf, 52, vel_d);
	_mav_put_int32_t(buf, 56, g_speed);
	_mav_put_int32_t(buf, 60, head_mot);
	_mav_put_uint32_t(buf, 64, s_acc);
	_mav_put_uint32_t(buf, 68, head_acc);
	_mav_put_uint16_t(buf, 72, year);
	_mav_put_uint16_t(buf, 74, p_dop);
	_mav_put_uint8_t(buf, 76, month);
	_mav_put_uint8_t(buf, 77, day);
	_mav_put_uint8_t(buf, 78, hour);
	_mav_put_uint8_t(buf, 79, min);
	_mav_put_uint8_t(buf, 80, sec);
	_mav_put_uint8_t(buf, 81, valid);
	_mav_put_uint8_t(buf, 82, fix_type);
	_mav_put_uint8_t(buf, 83, num_sat);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, buf, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
#else
	mavlink_rosflight_gnss_raw_t *packet = (mavlink_rosflight_gnss_raw_t *)msgbuf;
	packet->rosflight_timestamp = rosflight_timestamp;
	packet->time_of_week = time_of_week;
	packet->t_acc = t_acc;
	packet->nano = nano;
	packet->lon = lon;
	packet->lat = lat;
	packet->height = height;
	packet->height_msl = height_msl;
	packet->h_acc = h_acc;
	packet->v_acc = v_acc;
	packet->vel_n = vel_n;
	packet->vel_e = vel_e;
	packet->vel_d = vel_d;
	packet->g_speed = g_speed;
	packet->head_mot = head_mot;
	packet->s_acc = s_acc;
	packet->head_acc = head_acc;
	packet->year = year;
	packet->p_dop = p_dop;
	packet->month = month;
	packet->day = day;
	packet->hour = hour;
	packet->min = min;
	packet->sec = sec;
	packet->valid = valid;
	packet->fix_type = fix_type;
	packet->num_sat = num_sat;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW, (const char *)packet, MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE ROSFLIGHT_GNSS_RAW UNPACKING


/**
 * @brief Get field time_of_week from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_raw_get_time_of_week(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field year from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_rosflight_gnss_raw_get_year(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  72);
}

/**
 * @brief Get field month from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_month(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field day from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_day(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  77);
}

/**
 * @brief Get field hour from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_hour(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  78);
}

/**
 * @brief Get field min from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_min(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Get field sec from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_sec(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  80);
}

/**
 * @brief Get field valid from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_valid(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  81);
}

/**
 * @brief Get field t_acc from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_raw_get_t_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field nano from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_nano(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field fix_type from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_fix_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  82);
}

/**
 * @brief Get field num_sat from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint8_t mavlink_msg_rosflight_gnss_raw_get_num_sat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  83);
}

/**
 * @brief Get field lon from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field lat from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  24);
}

/**
 * @brief Get field height from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_height(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  28);
}

/**
 * @brief Get field height_msl from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_height_msl(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  32);
}

/**
 * @brief Get field h_acc from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_raw_get_h_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  36);
}

/**
 * @brief Get field v_acc from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_raw_get_v_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  40);
}

/**
 * @brief Get field vel_n from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_vel_n(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  44);
}

/**
 * @brief Get field vel_e from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_vel_e(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  48);
}

/**
 * @brief Get field vel_d from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_vel_d(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  52);
}

/**
 * @brief Get field g_speed from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_g_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  56);
}

/**
 * @brief Get field head_mot from rosflight_gnss_raw message
 *
 * @return 
 */
static inline int32_t mavlink_msg_rosflight_gnss_raw_get_head_mot(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  60);
}

/**
 * @brief Get field s_acc from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_raw_get_s_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  64);
}

/**
 * @brief Get field head_acc from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint32_t mavlink_msg_rosflight_gnss_raw_get_head_acc(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  68);
}

/**
 * @brief Get field p_dop from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint16_t mavlink_msg_rosflight_gnss_raw_get_p_dop(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  74);
}

/**
 * @brief Get field rosflight_timestamp from rosflight_gnss_raw message
 *
 * @return 
 */
static inline uint64_t mavlink_msg_rosflight_gnss_raw_get_rosflight_timestamp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Decode a rosflight_gnss_raw message into a struct
 *
 * @param msg The message to decode
 * @param rosflight_gnss_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rosflight_gnss_raw_decode(const mavlink_message_t* msg, mavlink_rosflight_gnss_raw_t* rosflight_gnss_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	rosflight_gnss_raw->rosflight_timestamp = mavlink_msg_rosflight_gnss_raw_get_rosflight_timestamp(msg);
	rosflight_gnss_raw->time_of_week = mavlink_msg_rosflight_gnss_raw_get_time_of_week(msg);
	rosflight_gnss_raw->t_acc = mavlink_msg_rosflight_gnss_raw_get_t_acc(msg);
	rosflight_gnss_raw->nano = mavlink_msg_rosflight_gnss_raw_get_nano(msg);
	rosflight_gnss_raw->lon = mavlink_msg_rosflight_gnss_raw_get_lon(msg);
	rosflight_gnss_raw->lat = mavlink_msg_rosflight_gnss_raw_get_lat(msg);
	rosflight_gnss_raw->height = mavlink_msg_rosflight_gnss_raw_get_height(msg);
	rosflight_gnss_raw->height_msl = mavlink_msg_rosflight_gnss_raw_get_height_msl(msg);
	rosflight_gnss_raw->h_acc = mavlink_msg_rosflight_gnss_raw_get_h_acc(msg);
	rosflight_gnss_raw->v_acc = mavlink_msg_rosflight_gnss_raw_get_v_acc(msg);
	rosflight_gnss_raw->vel_n = mavlink_msg_rosflight_gnss_raw_get_vel_n(msg);
	rosflight_gnss_raw->vel_e = mavlink_msg_rosflight_gnss_raw_get_vel_e(msg);
	rosflight_gnss_raw->vel_d = mavlink_msg_rosflight_gnss_raw_get_vel_d(msg);
	rosflight_gnss_raw->g_speed = mavlink_msg_rosflight_gnss_raw_get_g_speed(msg);
	rosflight_gnss_raw->head_mot = mavlink_msg_rosflight_gnss_raw_get_head_mot(msg);
	rosflight_gnss_raw->s_acc = mavlink_msg_rosflight_gnss_raw_get_s_acc(msg);
	rosflight_gnss_raw->head_acc = mavlink_msg_rosflight_gnss_raw_get_head_acc(msg);
	rosflight_gnss_raw->year = mavlink_msg_rosflight_gnss_raw_get_year(msg);
	rosflight_gnss_raw->p_dop = mavlink_msg_rosflight_gnss_raw_get_p_dop(msg);
	rosflight_gnss_raw->month = mavlink_msg_rosflight_gnss_raw_get_month(msg);
	rosflight_gnss_raw->day = mavlink_msg_rosflight_gnss_raw_get_day(msg);
	rosflight_gnss_raw->hour = mavlink_msg_rosflight_gnss_raw_get_hour(msg);
	rosflight_gnss_raw->min = mavlink_msg_rosflight_gnss_raw_get_min(msg);
	rosflight_gnss_raw->sec = mavlink_msg_rosflight_gnss_raw_get_sec(msg);
	rosflight_gnss_raw->valid = mavlink_msg_rosflight_gnss_raw_get_valid(msg);
	rosflight_gnss_raw->fix_type = mavlink_msg_rosflight_gnss_raw_get_fix_type(msg);
	rosflight_gnss_raw->num_sat = mavlink_msg_rosflight_gnss_raw_get_num_sat(msg);
#else
	memcpy(rosflight_gnss_raw, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_ROSFLIGHT_GNSS_RAW_LEN);
#endif
}
