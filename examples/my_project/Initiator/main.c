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
#include <hwtimer.h>
#include <periph/spi.h>
#include <periph/gpio.h>
#include <hwtimer.h>
#define delay(X)	(hwtimer_wait(1000*X))


static pn532_t pn532;


int pn532_initialization(pn532_t * pn532){
    puts("PN532 initialization");
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


int p2p_initiator(void){
	uint8_t message[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
	uint8_t data[255];
	uint8_t dlen;
	uint8_t atr_res[255];
	uint8_t alen = 0;
	uint8_t size_mess = sizeof(message);

printf("\n\n\nInitializing PN532 as Initiator!\n\n\n");
	uint8_t actpass = 0x01;	//Active mode
	uint8_t br = 0x02;		//424 kbps
	uint8_t next = 0x00;
	uint8_t status_initiator = pn532_in_jump_for_dep(&pn532, actpass, br, next, atr_res, &alen );
	if( status_initiator != 1 ) {
		printf("Something went wrong!\n");
		printf("Here it is response frame to detect the error:\n");
		//pn532_print_hex( readbuffer, 3 );
	}else{
			uint8_t mi = 0;
			do{
				pn532_in_data_exchange(&pn532, message, size_mess, 1, &mi, data, &dlen );
			}while( mi == 1 );
			delay(1);
			mi = 0;
			//pn532_in_data_exchange( &pn532, message, 0, 1, &mi, data, &dlen );

			puts("Sended. stop.");
	}
return 1;
}



int p2p_target(void){

	printf("\nInitialization as target!\n\n");
	uint8_t data[255];
	uint8_t dlen = 0;
	uint8_t response[255];

		uint8_t nad, did, atr, rats, picc;
		nad = 0x00;
		did = 0x00;
		atr = 0x00;
		rats = 0x01;
		picc = 0x01;
		//pn532_tg_set_parameters( &pn532, nad, did, atr, rats, picc );
		pn532_set_parameters( &pn532, nad, did, atr, rats, picc );
		uint8_t mode = 0x02;	//DEP only
		uint8_t status_target = pn532_tg_init_as_target(&pn532, mode );
		uint8_t mi = 0;
		if( status_target != 1) {
			printf("Initialization didn't successfully finish! HALT\n");
			delay(10);
		}else{
			uint8_t gt[] = { 0x8C, 0x29, 0x0E, 0x04, 0x34, 0xD7, 0x90, 0xE5, 0x61, 0xE0 };
			pn532_tg_set_general_bytes(&pn532, gt, 10 );
			delay(300);
			//pn532_tg_get_to_initiator(&pn532, data, &dlen );
			pn532_tg_get_initiator_command(&pn532, data, &dlen );
			for(int i=7; i < dlen ;i++ ){
				printf("\n received[i-7]: %02x",data[i]);
				response[i-7]=data[i];
			}

			delay(150);
			pn532_tg_get_target_status(&pn532);
			puts("received. stop.");
		}
return 1;
}

int main(void)
{
	printf("\nWelcome to RIOT, this is the Gateway\n");
		delay(5);
	 pn532_initialization(&pn532);
	 if(! pn532_SAM_config(&pn532) ){
	 		printf("Configuration SAM didn't end well! HALT");
	 		while(1);
	 	}

	// p2p_target();
	 delay(20000);
		////pn532_initialization(&pn532);


	 p2p_initiator();
	 delay(2000);
	 puts("SWITCHING GATEWAY!!!!");
	 p2p_target();
	 delay(20000);
	 puts("SWITCHING GATEWAY!!!!");
	 p2p_initiator();
	 for(int i=0;i<10;i++)
	 {
	 	printf("stoppe.");
	 }
}

	 /**
	 //int p2p_gateway(uint8_t * in, uint8_t * out){


		//uint8_t message[] = { 'h', 'e', 'l', 'l', 'o' };
			uint8_t message[] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE };
			uint8_t response[255];
			uint8_t data[255];
			uint8_t dlen=0;
			uint8_t atr_res[255];
			uint8_t alen = 0;
			uint8_t size_mess = sizeof(message);


    	printf("\nInitialization as target!\n\n");
    		uint8_t nad, did, atr, rats, picc;
    		nad = 0x00;
    		did = 0x00;
    		atr = 0x00;
    		rats = 0x01;
    		picc = 0x01;
    		//pn532_tg_set_parameters( &pn532, nad, did, atr, rats, picc );
    		pn532_set_parameters( &pn532, nad, did, atr, rats, picc );
    		uint8_t mode = 0x02;	//DEP only
    		uint8_t status_target = pn532_tg_init_as_target(&pn532, mode );
    		uint8_t mi = 0;
    		if( status_target != 1) {
    			printf("Initialization didn't successfully finish! HALT\n");
    			delay(10);
    		}else{
    			uint8_t gt[] = { 0x8C, 0x29, 0x0E, 0x04, 0x34, 0xD7, 0x90, 0xE5, 0x61, 0xE0 };
    			pn532_tg_set_general_bytes(&pn532, gt, 10 );
    			delay(300);
    			//pn532_tg_get_to_initiator(&pn532, data, &dlen );
    			pn532_tg_get_initiator_command(&pn532, data, &dlen );
    			for(int i=7; i < dlen ;i++ ){
    				printf("\n received[i-7]: %02x",data[i]);
    				response[i-7]=data[i];
    			}


    			delay(150);
    			pn532_tg_get_target_status(&pn532);
    		}

    		delay(2000);

    	printf("\n\n\nInitializing PN532 as Initiator!\n\n\n");
    	uint8_t actpass = 0x01;	//Active mode
    	uint8_t br = 0x02;		//424 kbps
    	uint8_t next = 0x00;
    	uint8_t status_initiator = pn532_in_jump_for_dep(&pn532, actpass, br, next, atr_res, &alen );
    	if( status_initiator != 1 ) {
    		printf("Something went wrong!\n");
    		printf("Here it is response frame to detect the error:\n");
    		//pn532_print_hex( readbuffer, 3 );
    	}else{
    			uint8_t mi = 0;
    			do{
    				pn532_in_data_exchange(&pn532, message, size_mess, 1, &mi, data, &dlen );
    			}while( mi == 1 );
    			delay(1);
    			mi = 0;
    			//pn532_in_data_exchange( &pn532, message, 0, 1, &mi, data, &dlen );

    			puts("Sended. stop.");
    	}

    	}**/
