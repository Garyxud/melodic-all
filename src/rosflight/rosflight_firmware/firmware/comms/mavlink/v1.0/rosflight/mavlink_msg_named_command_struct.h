// MESSAGE NAMED_COMMAND_STRUCT PACKING

#define MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT 186

typedef struct __mavlink_named_command_struct_t
{
 float x; /*< x value in the command struct*/
 float y; /*< y value in the command struct*/
 float z; /*< z value in the command struct*/
 float F; /*< F value in the command struct*/
 char name[10]; /*< Name of the command struct*/
 uint8_t type; /*< Type of command struct*/
 uint8_t ignore; /*< Type of command struct*/
} mavlink_named_command_struct_t;

#define MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN 28
#define MAVLINK_MSG_ID_186_LEN 28

#define MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC 169
#define MAVLINK_MSG_ID_186_CRC 169

#define MAVLINK_MSG_NAMED_COMMAND_STRUCT_FIELD_NAME_LEN 10

#define MAVLINK_MESSAGE_INFO_NAMED_COMMAND_STRUCT { \
	"NAMED_COMMAND_STRUCT", \
	7, \
	{  { "x", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_named_command_struct_t, x) }, \
         { "y", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_named_command_struct_t, y) }, \
         { "z", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_named_command_struct_t, z) }, \
         { "F", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_named_command_struct_t, F) }, \
         { "name", NULL, MAVLINK_TYPE_CHAR, 10, 16, offsetof(mavlink_named_command_struct_t, name) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_named_command_struct_t, type) }, \
         { "ignore", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_named_command_struct_t, ignore) }, \
         } \
}


/**
 * @brief Pack a named_command_struct message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param name Name of the command struct
 * @param type Type of command struct
 * @param ignore Type of command struct
 * @param x x value in the command struct
 * @param y y value in the command struct
 * @param z z value in the command struct
 * @param F F value in the command struct
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_command_struct_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       const char *name, uint8_t type, uint8_t ignore, float x, float y, float z, float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, ignore);
	_mav_put_char_array(buf, 16, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#else
	mavlink_named_command_struct_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.F = F;
	packet.type = type;
	packet.ignore = ignore;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
}

/**
 * @brief Pack a named_command_struct message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param name Name of the command struct
 * @param type Type of command struct
 * @param ignore Type of command struct
 * @param x x value in the command struct
 * @param y y value in the command struct
 * @param z z value in the command struct
 * @param F F value in the command struct
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_named_command_struct_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           const char *name,uint8_t type,uint8_t ignore,float x,float y,float z,float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, ignore);
	_mav_put_char_array(buf, 16, name, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#else
	mavlink_named_command_struct_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.F = F;
	packet.type = type;
	packet.ignore = ignore;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
}

/**
 * @brief Encode a named_command_struct struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param named_command_struct C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_command_struct_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_named_command_struct_t* named_command_struct)
{
	return mavlink_msg_named_command_struct_pack(system_id, component_id, msg, named_command_struct->name, named_command_struct->type, named_command_struct->ignore, named_command_struct->x, named_command_struct->y, named_command_struct->z, named_command_struct->F);
}

/**
 * @brief Encode a named_command_struct struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param named_command_struct C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_named_command_struct_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_named_command_struct_t* named_command_struct)
{
	return mavlink_msg_named_command_struct_pack_chan(system_id, component_id, chan, msg, named_command_struct->name, named_command_struct->type, named_command_struct->ignore, named_command_struct->x, named_command_struct->y, named_command_struct->z, named_command_struct->F);
}

/**
 * @brief Send a named_command_struct message
 * @param chan MAVLink channel to send the message
 *
 * @param name Name of the command struct
 * @param type Type of command struct
 * @param ignore Type of command struct
 * @param x x value in the command struct
 * @param y y value in the command struct
 * @param z z value in the command struct
 * @param F F value in the command struct
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_named_command_struct_send(mavlink_channel_t chan, const char *name, uint8_t type, uint8_t ignore, float x, float y, float z, float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN];
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, ignore);
	_mav_put_char_array(buf, 16, name, 10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#else
	mavlink_named_command_struct_t packet;
	packet.x = x;
	packet.y = y;
	packet.z = z;
	packet.F = F;
	packet.type = type;
	packet.ignore = ignore;
	mav_array_memcpy(packet.name, name, sizeof(char)*10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)&packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_named_command_struct_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const char *name, uint8_t type, uint8_t ignore, float x, float y, float z, float F)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, x);
	_mav_put_float(buf, 4, y);
	_mav_put_float(buf, 8, z);
	_mav_put_float(buf, 12, F);
	_mav_put_uint8_t(buf, 26, type);
	_mav_put_uint8_t(buf, 27, ignore);
	_mav_put_char_array(buf, 16, name, 10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, buf, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#else
	mavlink_named_command_struct_t *packet = (mavlink_named_command_struct_t *)msgbuf;
	packet->x = x;
	packet->y = y;
	packet->z = z;
	packet->F = F;
	packet->type = type;
	packet->ignore = ignore;
	mav_array_memcpy(packet->name, name, sizeof(char)*10);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT, (const char *)packet, MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE NAMED_COMMAND_STRUCT UNPACKING


/**
 * @brief Get field name from named_command_struct message
 *
 * @return Name of the command struct
 */
static inline uint16_t mavlink_msg_named_command_struct_get_name(const mavlink_message_t* msg, char *name)
{
	return _MAV_RETURN_char_array(msg, name, 10,  16);
}

/**
 * @brief Get field type from named_command_struct message
 *
 * @return Type of command struct
 */
static inline uint8_t mavlink_msg_named_command_struct_get_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field ignore from named_command_struct message
 *
 * @return Type of command struct
 */
static inline uint8_t mavlink_msg_named_command_struct_get_ignore(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field x from named_command_struct message
 *
 * @return x value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_x(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field y from named_command_struct message
 *
 * @return y value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_y(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field z from named_command_struct message
 *
 * @return z value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_z(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field F from named_command_struct message
 *
 * @return F value in the command struct
 */
static inline float mavlink_msg_named_command_struct_get_F(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Decode a named_command_struct message into a struct
 *
 * @param msg The message to decode
 * @param named_command_struct C-struct to decode the message contents into
 */
static inline void mavlink_msg_named_command_struct_decode(const mavlink_message_t* msg, mavlink_named_command_struct_t* named_command_struct)
{
#if MAVLINK_NEED_BYTE_SWAP
	named_command_struct->x = mavlink_msg_named_command_struct_get_x(msg);
	named_command_struct->y = mavlink_msg_named_command_struct_get_y(msg);
	named_command_struct->z = mavlink_msg_named_command_struct_get_z(msg);
	named_command_struct->F = mavlink_msg_named_command_struct_get_F(msg);
	mavlink_msg_named_command_struct_get_name(msg, named_command_struct->name);
	named_command_struct->type = mavlink_msg_named_command_struct_get_type(msg);
	named_command_struct->ignore = mavlink_msg_named_command_struct_get_ignore(msg);
#else
	memcpy(named_command_struct, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NAMED_COMMAND_STRUCT_LEN);
#endif
}
