/*
 * PIC_GPRS_MQTTClient
 * http://www.electronicwings.com
 *
 */

//#include "math.h"
#include "GPRS_MQTTClient.h"
#include "Definiciones.h"

int16_t packet_id_counter = 0;

char will_topic[] = "will"; //todo: revisar esto
char will_payload[] = "unexpected exit";
uint8_t will_qos = 0;
uint8_t will_retain = 0; //Losant no permite mensajes retenidos

uint16_t StringToUint16(uint8_t *_String) {
	uint16_t value = 0;
	while ('0' == *_String || *_String == ' ' || *_String == '"') {
		_String++;
	}
	while ('0' <= *_String && *_String <= '9') {
		value *= 10;
		value += *_String - '0';
		_String++;
	}
	return value;
}

uint8_t* AddStringToBuf(uint8_t *_buf, const char *_string) {
	uint16_t _length = strlen(_string);
	_buf[0] = _length >> 8;
	_buf[1] = _length & 0xFF;
	_buf += 2;
	strncpy((char*) _buf, _string, _length);
	return _buf + _length;
}

uint16_t MQTT_connectpacket(uint8_t *packet, char *cl_ID, char *acc_key,
		char *acc_sec) {
	uint8_t *_packet = packet;
	uint16_t _length;

	_packet[0] = (MQTT_CTRL_CONNECT << 4);
	//_packet+=2;
	_packet += 3;
	_packet = AddStringToBuf(_packet, "MQTT");
	_packet[0] = MQTT_PROTOCOL_LEVEL;
	_packet++;

	_packet[0] = MQTT_CONN_CLEANSESSION;
	if (strlen(will_topic) != 0) {
		_packet[0] |= MQTT_CONN_WILLFLAG;
		if (will_qos == 1)
			_packet[0] |= MQTT_CONN_WILLQOS_1;
		else if (will_qos == 2)
			_packet[0] |= MQTT_CONN_WILLQOS_2;
		if (will_retain == 1)
			_packet[0] |= MQTT_CONN_WILLRETAIN;
	}
//	if (strlen(AIO_USERNAME) != 0)
	if (strlen(acc_key) != 0)
		_packet[0] |= MQTT_CONN_USERNAMEFLAG;
//	if (strlen(AIO_KEY) != 0)
	if (strlen(acc_sec) != 0)
		_packet[0] |= MQTT_CONN_PASSWORDFLAG;
	_packet++;

	_packet[0] = MQTT_CONN_KEEPALIVE >> 8;
//	_packet[0] = (SP.mqtt_ref_rate * 20) >> 8;
	_packet++;
	_packet[0] = MQTT_CONN_KEEPALIVE & 0xFF;
//	_packet[0] = (SP.mqtt_ref_rate * 20) & 0xFF;
	_packet++;
	if (strlen(cl_ID) != 0) {
		_packet = AddStringToBuf(_packet, cl_ID);
	} else {
		_packet[0] = 0x0;
		_packet++;
		_packet[0] = 0x0;
		_packet++;
	}
	if (strlen(will_topic) != 0) {
		_packet = AddStringToBuf(_packet, will_topic);
		_packet = AddStringToBuf(_packet, will_payload);
	}

//	if (strlen(AIO_USERNAME) != 0) {
	if (strlen(acc_key) != 0) {
//		_packet = AddStringToBuf(_packet, AIO_USERNAME);
		_packet = AddStringToBuf(_packet, acc_key);
	}
//	if (strlen(AIO_KEY) != 0) {
	if (strlen(acc_sec) != 0) {
//		_packet = AddStringToBuf(_packet, AIO_KEY);
		_packet = AddStringToBuf(_packet, acc_sec);
	}
//	_length = _packet - packet;
	_length = _packet - packet - 2;

	//packet[1] = _length-2;
	if (_length <= 127) {
		packet[1] = _length;
	} else {
		packet[1] = 0x80 | (0x3F & _length);
		packet[2] = 0x3F & (_length >> 7);
	}

//	return _length;
	return _length + 3;
}

uint16_t MQTT_publishPacket(uint8_t *packet, const char *topic, char *data,
		uint8_t qos) {
	uint8_t *_packet = packet;
	uint16_t _length = 0;
	uint16_t Datalen = strlen(data);

	_length += 2;
	_length += strlen(topic);
	if (qos > 0) {
		_length += 2;
	}
	_length += Datalen;
	_packet[0] = MQTT_CTRL_PUBLISH << 4 | qos << 1;
	_packet++;
	do {
		uint8_t encodedByte = _length % 128;
		_length /= 128;
		if (_length > 0) {
			encodedByte |= 0x80;
		}
		_packet[0] = encodedByte;
		_packet++;
	} while (_length > 0);
	_packet = AddStringToBuf(_packet, topic);
	if (qos > 0) {
		_packet[0] = (packet_id_counter >> 8) & 0xFF;
		_packet[1] = packet_id_counter & 0xFF;
		_packet += 2;
		packet_id_counter++;
	}
	memmove(_packet, data, Datalen);
	_packet += Datalen;
	_length = _packet - packet;

	return _length;
}

uint16_t MQTT_subscribePacket(uint8_t *packet, const char *topic, uint8_t qos) {
	uint8_t *_packet = packet;
	uint16_t _length;

	_packet[0] = MQTT_CTRL_SUBSCRIBE << 4 | MQTT_QOS_1 << 1;
	_packet += 2;

	_packet[0] = (packet_id_counter >> 8) & 0xFF;
	_packet[1] = packet_id_counter & 0xFF;
	_packet += 2;
	packet_id_counter++;

	_packet = AddStringToBuf(_packet, topic);

	_packet[0] = qos;
	_packet++;

	_length = _packet - packet;
	packet[1] = _length - 2;

	return _length;
}

uint16_t MQTT_PingReqPacket(uint8_t *packet) {
	uint8_t *_packet = packet;
	uint16_t _length;

	_packet[0] = MQTT_CTRL_PINGREQ << 4 | MQTT_QOS_1 << 1;
	_packet[1] = 0;
	_packet += 2;

	_length = _packet - packet;
	return _length;
}

uint16_t MQTT_DisconnectPacket(uint8_t *packet) {
	uint8_t *_packet = packet;
	uint16_t _length;

	_packet[0] = MQTT_CTRL_DISCONNECT << 4;
	_packet[1] = 0;
	_packet += 2;

	_length = _packet - packet;
	return _length;
}
