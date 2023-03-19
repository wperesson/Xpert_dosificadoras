/*
 * PIC_GPRS_MQTTClient
 * http://www.electronicwings.com
 *
 */

#include "board.h"
#include "Definiciones.h"

#define MQTT_PROTOCOL_LEVEL		4

#define MQTT_CTRL_CONNECT		0x1
#define MQTT_CTRL_CONNECTACK	0x2
#define MQTT_CTRL_PUBLISH		0x3
#define MQTT_CTRL_PUBACK		0x4
#define MQTT_CTRL_PUBREC		0x5
#define MQTT_CTRL_PUBREL		0x6
#define MQTT_CTRL_PUBCOMP		0x7
#define MQTT_CTRL_SUBSCRIBE		0x8
#define MQTT_CTRL_SUBACK		0x9
#define MQTT_CTRL_UNSUBSCRIBE	0xA
#define MQTT_CTRL_UNSUBACK		0xB
#define MQTT_CTRL_PINGREQ		0xC
#define MQTT_CTRL_PINGRESP		0xD
#define MQTT_CTRL_DISCONNECT	0xE

#define MQTT_QOS_2				0x2
#define MQTT_QOS_1				0x1
#define MQTT_QOS_0				0x0

/* Adjust as necessary, in seconds */
#define MQTT_CONN_KEEPALIVE		120 //1hora

#define MQTT_CONN_USERNAMEFLAG	0x80
#define MQTT_CONN_PASSWORDFLAG	0x40
#define MQTT_CONN_WILLRETAIN	0x20
#define MQTT_CONN_WILLQOS_1		0x08
#define MQTT_CONN_WILLQOS_2		0x18
#define MQTT_CONN_WILLFLAG		0x04
#define MQTT_CONN_CLEANSESSION	0x02

//Connection Accepted
#define MQTT_CONN_RC_CONN_ACC	0x00

//Connection Refused, unacceptable protocol version.
//The Server does not support the level of the MQTT protocol requested by the Client
#define MQTT_CONN_RC_CONN_REF_CON	0x01

//Connection Refused, identifier rejected
//The Client identifier is correct UTF-8 but not allowed by the Server
#define MQTT_CONN_RC_CONN_REF_UPV	0x02

//Connection Refused, Server unavailable
//The Network Connection has been made but the MQTT service is unavailable
#define MQTT_CONN_RC_CONN_REF_IR	0x03

//Connection Refused, bad user name or password
//The data in the user name or password is malformed
#define MQTT_CONN_RC_CONN_REF_SU	0x04

//Connection Refused, not authorized
//The Client is not authorized to connect
#define MQTT_CONN_RC_CONN_REF_NA	0x05

uint16_t StringToUint16(uint8_t *_String);
uint8_t* AddStringToBuf(uint8_t *_buf, const char *_string);
bool sendPacket(uint8_t *buffer, uint16_t len);
uint16_t readPacket(uint8_t *buffer, int16_t _timeout);

uint16_t MQTT_connectpacket(uint8_t *packet, char *cl_ID, char *acc_key,
		char *acc_sec);
uint16_t MQTT_publishPacket(uint8_t *packet, const char *topic, char *data,
		uint8_t qos);
uint16_t MQTT_subscribePacket(uint8_t *packet, const char *topic, uint8_t qos);
uint16_t MQTT_PingReqPacket(uint8_t *packet);
uint16_t MQTT_DisconnectPacket(uint8_t *packet);

