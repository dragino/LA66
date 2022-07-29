/*
 * Copyright (C) 2015-2017 Alibaba Group Holding Limited
 */

#ifndef LINKWAN_AT_H
#define LINKWAN_AT_H

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#define AT_CMD "AT"

#define AT_ERROR         "AT_ERROR"
#define AT_PARAM_ERROR   "AT_PARAM_ERROR"
#define AT_BUSY_ERROR    "AT_BUSY_ERROR"
#define AT_NO_NET_JOINED "AT_NO_NET_JOINED"

#define LWAN_SUCCESS  0
#define LWAN_ERROR   -1    
#define LWAN_PARAM_ERROR   -2    
#define LWAN_BUSY_ERROR   -3
#define LWAN_NO_NET_JOINED   -4

//// mandatory
#define AT_DEBUG      "+DEBUG"
#define AT_RESET      "Z"
#define AT_FDR        "+FDR"
#define AT_DEUI       "+DEUI"  
#define AT_APPEUI     "+APPEUI"  
#define AT_APPKEY     "+APPKEY"  
#define AT_DEVADDR    "+DADDR"  
#define AT_APPSKEY    "+APPSKEY"  
#define AT_NWKSKEY    "+NWKSKEY"
#define AT_ADR        "+ADR"
#define AT_TXP        "+TXP"
#define AT_DR         "+DR"
#define AT_DCS        "+DCS"
#define AT_PNM        "+PNM"
#define AT_RX2FQ      "+RX2FQ"
#define AT_RX2DR      "+RX2DR"
#define AT_RX1DL      "+RX1DL"
#define AT_RX2DL      "+RX2DL"
#define AT_JN1DL      "+JN1DL"
#define AT_JN2DL      "+JN2DL"
#define AT_NJM        "+NJM"
#define AT_NWKID      "+NWKID"
#define AT_FCU        "+FCU"
#define AT_FCD        "+FCD"
#define AT_CLASS      "+CLASS"
#define AT_JOIN       "+JOIN"
#define AT_NJS        "+NJS"
#define AT_SEND       "+SEND"
#define AT_SENDB      "+SENDB"
#define AT_RECVB      "+RECVB"
#define AT_RECV       "+RECV"
#define AT_VER        "+VER"
#define AT_CFM        "+CFM"
#define AT_CFS        "+CFS"
#define AT_SNR        "+SNR"
#define AT_RSSI       "+RSSI"
#define AT_PORT       "+PORT"
#define AT_CHS        "+CHS"
#define AT_CHE        "+CHE"
#define AT_SLEEP      "+SLEEP"
#define AT_CFG        "+CFG"
#define AT_BAT        "+BAT"
#define AT_DWELLT     "+DWELLT"
#define AT_RJTDC      "+RJTDC"
#define AT_RPL        "+RPL"
#define AT_DEBUG      "+DEBUG"
#define AT_TIMESTAMP  "+TIMESTAMP"
#define AT_LEAPSEC    "+LEAPSEC"
#define AT_SYNCMOD    "+SYNCMOD"
#define AT_SYNCTDC    "+SYNCTDC"
#define AT_DDETECT    "+DDETECT"
#define AT_SETMAXNBTRANS    "+SETMAXNBTRANS"
#define AT_UUID       "+UUID" 
#define AT_DEVICETIMEREQ  "+DEVICETIMEREQ"
#define AT_CHSIGNALDETECT "+GETCHANSTAT"

#ifdef LA66_HARDWARE_TEST
#define AT_GPIOTEST "+GPIOTEST"
#define AT_CTXCW    "+CTXCW"
#endif

void set_at_receive(uint8_t AppPort, uint8_t* Buff, uint8_t BuffSize);
void linkwan_at_init(void);
void linkwan_at_process(void);
void linkwan_serial_input(uint8_t cmd);
int linkwan_serial_output(uint8_t *buffer, int len);
void linkwan_at_prompt_print();
#endif
