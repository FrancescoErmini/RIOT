

#ifndef KEYEXC_H_
#define KEYEXC_H_

#include <stdint.h>
 #include <stdio.h>
#include <stdint.h>
#include "periph_conf.h"
#include "periph/gpio.h"
#include "mutex.h"
#include <pn532.h>

#ifdef __cplusplus
 extern "C" {
#endif


typedef enum{
	KEYEXC_IDLE,
	KEYEXC_ACK_RECEIVED,
	KEYEXC_ACK_FAILED,

	KEYEXC_GATEWAY_ID_REQUEST_RECEIVED,
	KEYEXC_GATEWAY_ID_REQUEST_FAILED,
	KEYEXC_GATEWAY_ID_VERIFIED,
	KEYEXC_GATEWAY_ID_FAILED,

	KEYEXC_NODE_ID_REQUEST_RECEIVED,
	KEYEXC_NODE_ID_REQUEST_FAILED,
	KEYEXC_NODE_ID_VERIFIED,
	KEYEXC_NODE_ID_FAILED,

	KEYEXC_TYPE_REQUEST_RECEIVED,
	KEYEXC_TYPE_REQUEST_FAILED,
	KEYEXC_TYPE_VERIFIED,
	KEYEXC_TYPE_FAILED,

	KEYEXC_PAYLOAD_REQUEST_RECEIVED,
		KEYEXC_PAYLOAD_REQUEST_FAILED,
		KEYEXC_PAYLOAD_VERIFIED,
		KEYEXC_PAYLOAD_FAILED,
} keyexc_status_t;



typedef enum{
	KEY_802154,
	KEY_RPL,
	KEY_COAP,
}keyexc_type_t;

typedef enum {
	GATEWAY,
	SENSOR,
}keyexc_node_t;


typedef struct {


	keyexc_node_t node_type;
	uint8_t * node_id;
	uint8_t * gateway_id;
	unsigned long keyexc_timeout;
	keyexc_type_t keyexc_type;
	keyexc_status_t keyexc_status;
	uint8_t * key_payload;
	unsigned int key_size;
	pn532_t * dev;
}keyexc_t;




int keyexc(keyexc_t * keyexc);
void keyexc_init(keyexc_t * keyexc, pn532_t * pn532, keyexc_node_t node_type, keyexc_type_t keyexc_type, uint8_t * node_id, uint8_t * gateway_id, uint8_t * keypayload );

int timeout_test(void);

#ifdef __cplusplus
}
#endif

#endif /* keyexc */
/** @} */
