/*
 * TcpIp.h
 *
 *  Created on: Jan 03, 2022
 *      Author: Walter Peresson
 */

#ifndef TcpIp_H_
#define TcpIp_H_

#include "Mobile_types.h"

#define JWT "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpYXQiOjE2NTE2MTk3ODR9.Uskc-krvLxP0gk3hGjTluJosl9GbuxiWjC-uc541ayc"

bool tcpip_PDP_configure(MOBILE_T*);
bool tcpip_get_ip(MOBILE_T*);
bool tcpip_network_active_control(MOBILE_T*, char);
bool tcpip_activate_connection(MOBILE_T*);
bool tcpipActivateNetworkConnection(MOBILE_T*);
bool LTE_TCP_Disconnect(MOBILE_T*);
bool LTE_mqtt_subscribe(MOBILE_T*);
bool LTE_mqtt_publish(MOBILE_T*, uint16_t, char*, uint16_t*);
uint16_t https_get_endpoint(MOBILE_T*);
uint16_t https_get_credentials(MOBILE_T*, volatile DEVICE_T*);

#endif /* TcpIp_H_ */
