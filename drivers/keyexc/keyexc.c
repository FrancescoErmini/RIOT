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
#include <keyexc.h>
 #include <xbee.h>
#include <hwtimer.h>
#include <periph_conf.h>
#include <periph/spi.h>
#include <periph/gpio.h>
#include <hwtimer.h>

#define delay(X)	(hwtimer_wait(1000*X))



#define TIMEOUT_S (20)
#define TIMEOUT_US (TIMEOUT_S * 1000 * 1000)
#define TIMEOUT (HWTIMER_TICKS(TIMEOUT_US))




void pn532_initialization(pn532_t * pn532){
    puts("PN532 init");
    hwtimer_wait(10*1000*1000);
    pn532_init_master(pn532, SPI_0, SPI_CONF_FIRST_RISING, SPI_SPEED_1MHZ, GPIO_0 );

    	printf("End initialize!\n");

    	printf("Begin. I'll wake up the PN532\n");
    	pn532_begin(pn532);
    	printf("End begin().\n");

    	printf("\nStarting getFirmware version\n");

    	uint8_t response, ic, version, rev, support;
    	pn532_get_firmware_version(pn532, &response, &ic, &version, &rev, &support);

    	if( response != 0x03 ){
    		printf("Response byte of Get Firmware Version wrong! HALT\n");
    		while(1);
    	}
    	if( ic != 0x32 ){
    		printf("Didn't find PN53x board.\n HALT!\n IC version: %X\n", ic);
    		while (1); // halt
    	}
    	if( version != 0x01 ){
    		printf("Version received not expected!\n");
    	}
    	printf("Here it is the frame version:\n| %X | %X | %X | %X | %X |\n", response, ic, version, rev, support);

}


int p2p_initiator(pn532_t * pn532, uint8_t * message,uint8_t size_mess){
	//uint8_t message[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
	uint8_t data[255];
	uint8_t dlen;
	uint8_t atr_res[255];
	uint8_t alen = 0;
	//uint8_t size_mess = sizeof(message);
	if(! pn532_SAM_config(pn532) ){
					printf("Configuration SAM didn't end well! HALT");
					while(1);
				}
delay(3000);
printf("\n\n\nInitializing PN532 as Initiator!\n\n\n");
	uint8_t actpass = 0x01;	//Active mode
	uint8_t br = 0x02;		//424 kbps
	uint8_t next = 0x00;
	uint8_t status_initiator = pn532_in_jump_for_dep(pn532, actpass, br, next, atr_res, &alen );
	if( status_initiator != 1 ) {
		printf("Something went wrong!\n");
		printf("Here it is response frame to detect the error:\n");
		//pn532_print_hex( readbuffer, 3 );
	}else{
			uint8_t mi = 0;
			do{
				pn532_in_data_exchange(pn532, message, size_mess, 1, &mi, data, &dlen );
			}while( mi == 1 );
			delay(1);
			mi = 0;
			//pn532_in_data_exchange( &pn532, message, 0, 1, &mi, data, &dlen );

			puts("Sended. stop.");
			//return 1;
	}
return 1;
}



int p2p_target(pn532_t * pn532, uint8_t * response){


	uint8_t data[255];
	uint8_t dlen = 0;
	uint8_t responsetmp[255];

		uint8_t nad, did, atr, rats, picc;
		nad = 0x00;
		did = 0x00;
		atr = 0x00;
		rats = 0x01;
		picc = 0x01;
		if(! pn532_SAM_config(pn532) ){
						printf("Configuration SAM didn't end well! HALT");
						while(1);
					}
		printf("\nInitialization as target!\n\n");
		//pn532_tg_set_parameters( &pn532, nad, did, atr, rats, picc );
		pn532_set_parameters( pn532, nad, did, atr, rats, picc );
		uint8_t mode = 0x02;	//DEP only
		uint8_t status_target = pn532_tg_init_as_target(pn532, mode );
		uint8_t mi = 0;
		if( status_target != 1) {
			printf("Initialization didn't successfully finish! HALT\n");
			delay(10);
		}else{
			uint8_t gt[] = { 0x8C, 0x29, 0x0E, 0x04, 0x34, 0xD7, 0x90, 0xE5, 0x61, 0xE0 };
			pn532_tg_set_general_bytes(pn532, gt, 10 );
			delay(300);
			//pn532_tg_get_to_initiator(&pn532, data, &dlen );
			pn532_tg_get_initiator_command(pn532, data, &dlen );
			//*response_size = dlen-7;

			for(int k=7; k < dlen ;k++ ){
				printf("\n received[i-7]: %02x",data[k]);
				responsetmp[k-7]=data[k];
				response[k-7]=data[k];
			}

			delay(150);
			pn532_tg_get_target_status(pn532);
			puts("received. stop.");

		}
return 1;
}






/*	static void callback(void * ptr)
	{
	   done = 1;

	}*/


void p2p_keyexc (keyexc_t * keyexc, pn532_t * pn532, uint8_t * message,uint8_t size_mess, uint8_t * response) {
	  if(keyexc->node_type == 0) {
		  puts("THIS IS GATEWAY");
		  p2p_target(pn532,response);
		  p2p_initiator(pn532,message,sizeof(message));
	  }

	  if(keyexc->node_type == 1) {
		  	  puts("THIS IS SENSOR");
	 		  p2p_initiator(pn532,message,sizeof(message));
	 		  p2p_target(pn532,response);
	 	  }


  }

static void callback(void *done_)
{
    volatile int *done = done_;
    *done = 1;
}

void keyexc(keyexc_t * keyexc, pn532_t * pn532)
   {
	//keyexc->node_type = GATEWAY;
	//keyexc->keyexc_type= KEY_802154;
	  pn532_initialization(pn532);


	uint8_t ack[] =  { 0x06, 0x07, 0x08, 0x09, 0x10 };
	uint8_t node_id[] = {0xAA,0xAA,0xAA,0xAA,0xAA};
	///uint8_t key_type = (uint8_t) keyexc->keyexc_type;


//	uint8_t ack_buf=0x00;
	///uint8_t id_buf[] ={0x00,0x00,0x00,0x00 };

	uint8_t resp2[] ={0x00,0x00,0x00,0x00,0x00 };
	uint8_t resp1[] ={0x00,0x00,0x00,0x00,0x00 };
	//uint8_t resp1_size;
	//uint8_t resp2_size;
	//unsigned long timeout = 30;


	volatile int done = 0;

      hwtimer_set(TIMEOUT, callback, (void *) &done);
       do {
    	   printf("start timeout loop");

    	   p2p_keyexc(keyexc, pn532, ack, sizeof(ack), resp1);
    	   p2p_keyexc(keyexc, pn532, node_id,sizeof(node_id),resp2);
    	   printf("stop timeout loop");
       } while (done == 0);

int l;
    	for(l=0;l< (sizeof(resp1)) ;l++)
    	{
    		printf("resp1[%i]: %02x",l,resp1[l]);
    	}

    	for(l=0;l< (sizeof(resp2));l++)
    	{
    		printf("resp2[%i]: %02x",l,resp2[l]);
    	}

   }


/**
 * @TODO use void * dev instead of pn532
 */
void keyexc_init(keyexc_t * keyexc, pn532_t * pn532, keyexc_node_t node_type,keyexc_type_t keyexc_type)
 {
keyexc->node_type = node_type;
keyexc->keyexc_type= keyexc_type;
/**
 * @TODO timeout variable
 */
/**
 * @TODO how to add new id?
 */
//keyexc->node_id=node_id;


 }
int timeout_test (void)
{
	volatile int done = 0;

	      hwtimer_set(TIMEOUT, callback, (void *) &done);
	       do {
	    	   puts("timeout running");
	       } while (done == 0);

printf("TIMEOUT OCCURSSS!!");
int a = 3*5;
printf("TIMEOUT OCCURSSS!! %i",a);
printf("TIMEOUT OCCURSSS!! %i",a);
printf("TIMEOUT OCCURSSS!! %i",a);
return 1;
}
