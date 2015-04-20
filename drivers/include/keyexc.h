

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

	ACK_RECEIVE,
	ID_VERIFIED,



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
	uint8_t node_id[4];
	unsigned long keyexc_timeout;
	keyexc_type_t keyexc_type;
	keyexc_status_t keyexc_status;

}keyexc_t;




void keyexc(keyexc_t * keyexc, pn532_t * pn532);
void keyexc_init(keyexc_t * keyexc, pn532_t * pn532, keyexc_node_t node_type, keyexc_type_t keyexc_type);



//int timeout_test (void);



#ifdef __cplusplus
}
#endif

#endif /* keyexc */
/** @} */
