/*
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**

 * @brief
 *
 *
 * @author
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pn532.h>
#include <xbee.h>
#include <keyexc.h>
#ifndef _SENSOR
#define _SENSOR (1)
#endif
#ifndef _GATEWAY
#define _GATEWAY (0)
#endif

static keyexc_t key;
static pn532_t pn532;

void keyexc_cb(void *arg){
		printf("THE NODE IS ASKING A NEW KEY ");
		keyexc(&key);
	}
void keyexc_set_key(void)
{

}
int main (void)
{

uint8_t node_id []  = {0xAA,0xAA,0xAA,0xAA};
uint8_t gateway_id[] = {0x01,0x01,0x01,0x01};
#if _SENSOR
	/* Define empty key buffer. */
	uint8_t key_buf[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
						  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
	keyexc_init(&key, &pn532,SENSOR, KEY_802154, node_id, gateway_id, key_buf);
	gpio_init_int(GPIO_15, GPIO_NOPULL, GPIO_RISING, keyexc_cb, 0);
	/* define the callback that ask for a key when a toggle switch is set */
#endif
#if _GATEWAY
uint8_t key_buf[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
				  0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

	keyexc_init(&key, &pn532,GATEWAY, KEY_802154, node_id, gateway_id, key_buf);
    keyexc(&key);

#endif

return 0;

}
