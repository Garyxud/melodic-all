// MESSAGE OFFBOARD_CONTROL PACKING

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL 180

typedef struct __mavlink_offboard_control_t
{
 float x; /*< x control channel interpreted according to mode*/
 float y; /*< y control channel, interpreted according to mode*/
 float z; /*< z control channel, interpreted according to mode*/
 float F; /*< F control channel, interpreted according to mode*/
 uint8_t mode; /*< Offboard control mode, see OFFBOARD_CONTROL_MODE*/
 uint8_t ignore; /*< Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE*/
} mavlink_offboard_control_t;

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN 18
#define MAVLINK_MSG_ID_180_LEN 18

#define MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC 93
#define MAVLINK_MSG_ID_180_CRC 93



#define MAVLINK_MESSAGE_INFO_OFFBOARD_CONTROL { \
	"OFFBOARD_CONTROL", \
	6, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_offboard_control_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_offboard_control_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_offboard_control_t, z) }, \
         { "F", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_offboard_control_t, F) }, \
         { "mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_offboard_control_t, mode) }, \
         { "ignore", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_offboard_control_t, ignore) }, \
         } \
}


/**
 * @brief Pack a offboard_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param mode Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param x x control channel interpreted according to mode
 * @param y y control channel, interpreted according to mode
 * @param z z control channel, interpreted according to mode
 * @param F F control channel, interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t mode, uint8_t ignore, float x, float y, float z, float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, ignore);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
	mavlink_offboard_control_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.F = F;
	packet.mode = mode;
	packet.ignore = ignore;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}

/**
 * @brief Pack a offboard_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param mode Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param x x control channel interpreted according to mode
 * @param y y control channel, interpreted according to mode
 * @param z z control channel, interpreted according to mode
 * @param F F control channel, interpreted according to mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_offboard_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t mode,uint8_t ignore,float x,float y,float z,float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, ignore);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#else
	mavlink_offboard_control_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.F = F;
	packet.mode = mode;
	packet.ignore = ignore;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_OFFBOARD_CONTROL;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}

/**
 * @brief Encode a offboard_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
	return mavlink_msg_offboard_control_pack(system_id, component_id, msg, offboard_control->mode, offboard_control->ignore, offboard_control->x, offboard_control->y, offboard_control->z, offboard_control->F);
}

/**
 * @brief Encode a offboard_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param offboard_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_offboard_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_offboard_control_t* offboard_control)
{
	return mavlink_msg_offboard_control_pack_chan(system_id, component_id, chan, msg, offboard_control->mode, offboard_control->ignore, offboard_control->x, offboard_control->y, offboard_control->z, offboard_control->F);
}

/**
 * @brief Send a offboard_control message
 * @param chan MAVLink channel to send the message
 *
 * @param mode Offboard control mode, see OFFBOARD_CONTROL_MODE
 * @param ignore Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 * @param x x control channel interpreted according to mode
 * @param y y control channel, interpreted according to mode
 * @param z z control channel, interpreted according to mode
 * @param F F control channel, interpreted according to mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_offboard_control_send(mavlink_channel_t chan, uint8_t mode, uint8_t ignore, float x, float y, float z, float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, ignore);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#else
	mavlink_offboard_control_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.F = F;
	packet.mode = mode;
	packet.ignore = ignore;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_offboard_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t mode, uint8_t ignore, float x, float y, float z, float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 16, mode);
	_mav_put_uint8_t(buf, 17, ignore);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, buf, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#else
	mavlink_offboard_control_t *packet = (mavlink_offboard_control_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->F = F;
	packet->mode = mode;
	packet->ignore = ignore;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN, MAVLINK_MSG_ID_OFFBOARD_CONTROL_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OFFBOARD_CONTROL, (const char *)packet, MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE OFFBOARD_CONTROL UNPACKING


/**
 * @brief Get field mode from offboard_control message
 *
 * @return Offboard control mode, see OFFBOARD_CONTROL_MODE
 */
static inline uint8_t mavlink_msg_offboard_control_get_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field ignore from offboard_control message
 *
 * @return Bitfield specifying which fields should be ignored, see OFFBOARD_CONTROL_IGNORE
 */
static inline uint8_t mavlink_msg_offboard_control_get_ignore(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field x from offboard_control message
 *
 * @return x control channel interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from offboard_control message
 *
 * @return y control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from offboard_control message
 *
 * @return z control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field F from offboard_control message
 *
 * @return F control channel, interpreted according to mode
 */
static inline float mavlink_msg_offboard_control_get_F(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a offboard_control message into a struct
 *
 * @param msg The message to decode
 * @param offboard_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_offboard_control_decode(const mavlink_message_t* msg, mavlink_offboard_control_t* offboard_control)
{
#if MAVLINK_NEED_BYTE_SWAP
	offboard_control->x = mavlink_msg_offboard_control_get_x(msg);
	offboard_control->y = mavlink_msg_offboard_control_get_y(msg);
	offboard_control->z = mavlink_msg_offboard_control_get_z(msg);
	offboard_control->F = mavlink_msg_offboard_control_get_F(msg);
	offboard_control->mode = mavlink_msg_offboard_control_get_mode(msg);
	offboard_control->ignore = mavlink_msg_offboard_control_get_ignore(msg);
#else
	memcpy(offboard_control, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_OFFBOARD_CONTROL_LEN);
#endif
}
