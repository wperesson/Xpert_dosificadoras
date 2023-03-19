/*
 Copyright (c) 2017 Edge Instruments.

 Permission is hereby granted, free of charge, to any person obtaining
 a copy of this software and associated documentation files (the
 "Software"), to deal in the Software without restriction, including
 without limitation the rights to use, copy, modify, merge, publish,
 distribute, sublicense, and/or sell copies of the Software, and to
 permit persons to whom the Software is furnished to do so, subject to
 the following conditions:

 The above copyright notice and this permission notice shall be
 included in all copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

 Original Maker: Walter Peressón - Edge Instruments
 Modified and Maintened by: Walter Peressón

 */

#ifndef __FTP_SIM868_H_
#define __FTP_SIM868_H_

//#include <SoftwareSerial.h>
#include <stdlib.h>
//#include "Arduino.h"
#include "board.h"
#include "Definiciones.h"

#define DEFAULT_BUFFER_SIZE      64
#define FONA_RX 2
#define FONA_TX 3
#define FONA_RST 4
//#define SERVER "translate.ubidots.com"
//#define PORT "9012"
#define USER_AGENT "GOFSCO"
#define VERSION "1.0"

bool ftp_sim868_set_conn(LPC_USART_T *, char*, uint16_t, char*, char*, char*,
		char*, char*);
bool ftp_sim868_get_stat(LPC_USART_T *, char*, uint16_t);
int16_t ftp_sim868_GET_FILE(LPC_USART_T *, char*, uint16_t, char*, char*,
		uint16_t);
bool ftp_sim868_PUT(LPC_USART_T *, char*, uint16_t, uint16_t);

#endif  // __FTP_SIM868_H_
