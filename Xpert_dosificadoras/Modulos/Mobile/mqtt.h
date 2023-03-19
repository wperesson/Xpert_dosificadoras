/*
 * mqtt.h
 *
 *  Created on: 3 ene. 2022
 *      Author: Walter Peresson
 */

#ifndef MODULOS_MOBILE_MQTT_H_
#define MODULOS_MOBILE_MQTT_H_

#include "Mobile_types.h"

bool mqtt_check_broker_connection(MOBILE_T*);
BROKER_CONN_REPLY_ENUM mqtt_config_conn_to_broker(MOBILE_T*, uint16_t, uint16_t);
bool mqtt_connect_to_broker(MOBILE_T*);
bool mqtt_disconnect_from_broker(MOBILE_T*);
bool mqtt_publish(MOBILE_T*, uint16_t, char*);
bool mqtt_subscribe(MOBILE_T*, char*, uint8_t);
bool mqtt_unsubscribe(MOBILE_T*, char*) ;

#endif /* MODULOS_MOBILE_MQTT_H_ */
