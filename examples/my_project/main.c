/*
 * Copyright (C) 2014 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Hello World application
 *
 * @author      Kaspar Schleiser <kaspar@schleiser.de>
 * @author      Ludwig Ortmann <ludwig.ortmann@fu-berlin.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pn532.h>
#include <xbee.h>
#include <keyexc.h>
#define _SENSOR 1
#define _GATEWAY 0
static keyexc_t key;
static pn532_t pn532;

int main (void)
{
	uint8_t node_id []  = {0xAA,0xAA,0xAA,0xAA};
	uint8_t gateway_id[] = {0x01,0x01,0x01,0x01};


#if _SENSOR


	/* Define empty key buffer. */
	uint8_t key_buf[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
						  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};
	/* define the callback that ask for a key when a toggle switch is set */

keyexc_init(&key, &pn532,SENSOR, KEY_802154, node_id, gateway_id, key_buf);
keyexc(&key, &pn532);

#endif


#if _GATEWAY
uint8_t key_buf[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
				  0x09, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16};

	keyexc_init(&key, &pn532,GATEWAY, KEY_802154, node_id, gateway_id, key_buf);
    keyexc(&key, &pn532);

#endif
printf("controllo ritornato al main.");
//timeout_test();


#if _SENSOR
    for (int i=0; i<16; i++)
    {
 printf("key_buf[%i]: %02x",i,key_buf[i]);
    }
#endif
timeout_test();
return 0;

}
