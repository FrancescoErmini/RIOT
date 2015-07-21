

#ifndef KEYEXC_H_
#define KEYEXC_H_

#include <stdint.h>
 #include <stdio.h>
#include <stdint.h>
#include "periph_conf.h"
#include "periph/gpio.h"
#include "mutex.h"
#include <pn532.h>
#include "xbee.h"
#ifdef __cplusplus
 extern "C" {
#endif


typedef enum{
	KEYEXC_IDLE,
	KEYEXC_PROTO,
	KEYEXC_CHANNEL,
	KEYEXC_PANID,
	KEYEXC_SHORTADDRESS,
	KEYEXC_LONGADDRESS,
	KEYEXC_KEY,
	KEYEXC_DONE,
	KEYEXC_RPLDATA1,
	KEYEXC_RPLDATA2,
	KEYEXC_RPLDATA3,
	KEYEXC_COAPDATA1,
	KEYEXC_COAPDATA2,
	KEYEXC_COAPDATA3,
} keyexc_status_t;

typedef enum {
	UNDEF,
	IEEE802154,
	RPL,
	COAP,
}keyexc_proto_t;

typedef enum {
	GATEWAY,
	SENSOR,
}keyexc_node_t;

typedef struct {
//uint8_t id;
//uint8_t proto;
uint8_t flag;
uint8_t channel;
uint8_t panid[2];
uint8_t short_address[2];
uint8_t long_address[8];
uint8_t key[16];
}keyexc_802154_t;

typedef struct {
	uint8_t flag;
	uint8_t dataRPL1[2];
	uint8_t dataRPL2[2];
	uint8_t dataRPL3[2];
}keyexc_RPL_t;

typedef struct {
	uint8_t flag;
	uint8_t dataCOAP1[2];
	uint8_t dataCOAP2[2];
	uint8_t dataCOAP3[2];
}keyexc_COAP_t;

typedef struct {
			keyexc_node_t node_type;
			uint8_t id;
			uint8_t ack;
			pn532_t * dev;
			keyexc_proto_t proto;
			//unsigned long keyexc_timeout;
			keyexc_status_t status;
			//uint8_t * data;
			int rx_count;
            keyexc_802154_t * data802154;
            keyexc_RPL_t * dataRPL;
            keyexc_COAP_t * dataCOAP;

}keyexc_t;


int keyexc(keyexc_t * keyexc);

void keyexc_init(keyexc_t * keyexc, pn532_t * pn532, keyexc_node_t node_type, uint8_t id, keyexc_802154_t * data802154, keyexc_RPL_t * dataRPL, keyexc_COAP_t * dataCOAP);
void keyexc_set_xbee(xbee_t * dev, keyexc_t * keyexc);
int timeout_test(void);

#ifdef __cplusplus
}
#endif

#endif /* keyexc */
/** @} */
