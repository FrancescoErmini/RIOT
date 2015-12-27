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
#include <periph/uart.h>
#include <hwtimer.h>
#include "cpu.h"
#include "msg.h"
#include "kernel.h"
#include "thread.h"
#include "board.h"
#include "vtimer.h"
#include "ringbuffer.h"
#include <stdint.h>
#include <inttypes.h>
#define delay(X)	(hwtimer_wait(1000*X))



#define TIMEOUT_S (40)
#define TIMEOUT_US (TIMEOUT_S * 1000 * 1000)
#define TIMEOUT (HWTIMER_TICKS(TIMEOUT_US))
/*
static int _tx_cb(void *arg)
{
	 keyexc_t * keyexc = (keyexc_t *)arg;

        char c = (char)(keyexc->id);
        uart_write(UART_0, c);

    //mutex_unlock(&(dev->tx_lock));
    return 0;
}

static void _rx_cb(void *arg, char c)
{
    keyexc_t * keyexc = (keyexc_t *)arg;
    msg_t msg;
    char rx;
    char risp[30];
    switch (keyexc->status) {
            case KEYEXC_IDLE:
            	rx =  c;
            	//printf("ack: char: %c int: %i  hex: %02x !",c,keyexc->ack,keyexc->ack);
printf("ack:%c",rx);
            	keyexc->status = KEYEXC_802154;
            	break;
            case KEYEXC_802154:

                   	if(keyexc->rx_count < 30) { //number of byte in the Mysql row

                   //	keyexc->data[keyexc->rx_count++] = (uint8_t)c;
                   			risp[keyexc->rx_count++]=c;
                   	printf("data[%i]:%c",keyexc->rx_count, risp[keyexc->rx_count]);
                   	}
                    break;
    }
}
*/

/*
#define DEV             UART_0
#define BAUD            115200

static volatile int main_pid;

static char uart_stack[THREAD_STACKSIZE_MAIN];

static char rx_mem[128];
static char tx_mem[128];
static ringbuffer_t rx_buf;
static ringbuffer_t tx_buf;


void rx(void *ptr, char data)
{
    msg_t msg;

    ringbuffer_add_one(&rx_buf, data);

    if (data == '\n') {
        msg_send(&msg, main_pid);
    }
}

int tx(void *ptr)
{
    if (tx_buf.avail > 0) {
       char data = ringbuffer_get_one(&tx_buf);

        uart_write(DEV, data);
       return 1;
    }

    return 0;
}

void *uart_thread(void *arg)
{ printf("thread uart");
    keyexc_t * keyexc = (keyexc_t *) arg ;
    char *status = "1\n";
    ringbuffer_add(&tx_buf, status, strlen(status));
   while(keyexc->rx_count < 10){
	   printf("\nsendeeeuart");
        uart_tx_begin(DEV);
        keyexc->rx_count++;
        vtimer_usleep(2000ul * 1000ul);
   }
    return 0;
}
*/
    /*switch (keyexc->status) {
        case KEYEXC_IDLE:
        	keyexc->ack = (uint8_t) c;
        	printf("rx uart: char: %c int: %i  hex: %02x !",c,keyexc->ack,c);

        	keyexc->status = KEYEXC_802154;
        	break;
       case KEYEXC_PROTO:
        	keyexc->proto = (uint8_t)c;
        	if (keyexc->proto == 0 ) // IEEE802154
        			{ keyexc->status = KEYEXC_802154; }
        	if (keyexc->proto == 1 ) // RPL
                	{ keyexc->status = KEYEXC_RPL; }
        	if (keyexc->proto == 2 ) // COAP
                	{ keyexc->status = KEYEXC_COAP; }
         break;
        case KEYEXC_802154:

        	if(keyexc->rx_count < 30) { //number of byte in the Mysql row
        	keyexc->data[keyexc->rx_count++] = (uint8_t)c;

        	printf("data[%i]:0x%02x",keyexc->rx_count, keyexc->data[keyexc->rx_count]);
        	}
         break;
        case KEYEXC_RPL:
//TODO
         break;
        case KEYEXC_COAP:
//TODO
         break;




    }*/


#define DEV             UART_0
#define BAUD            115200

static volatile int main_pid;

static char uart_stack[THREAD_STACKSIZE_MAIN];

static char rx_mem[128];
static char tx_mem[128];
static ringbuffer_t rx_buf;
static ringbuffer_t tx_buf;


void rx(void *ptr, char data)
{ //printf("reveing\n");
    msg_t msg;

    ringbuffer_add_one(&rx_buf, data);
    if (data == '\n') {
        msg_send(&msg, main_pid);
    }
}

int tx(void *ptr)
{
    if (tx_buf.avail > 0) {
        char data = ringbuffer_get_one(&tx_buf);
        uart_write(DEV, data);
        return 1;
    }

    return 0;
}

void *uart_thread(void *arg)
{
    keyexc_t * keyexc = (keyexc_t *) arg;
char * id1 = "1\n";
char * id2 = "2\n";
char * id3 = "3\n";
char * str;
if (keyexc->id == 0x01) {
	str = id1;
}
if (keyexc->id == 0x02) {
	str = id2;
}
if (keyexc->id == 0x03) {
	str = id3;
}
    while (keyexc->rx_count < 2) {
    	keyexc->rx_count++;
        ringbuffer_add(&tx_buf, str, strlen(str));
        uart_tx_begin(DEV);

        vtimer_usleep(2000ul * 1000ul);
    }

    return 0;
}

void pn532_initialization(pn532_t * pn532){
    //puts("PN532 wake up");
    hwtimer_wait(10*1000*1000);
    pn532_init_master(pn532, SPI_0, SPI_CONF_FIRST_RISING, SPI_SPEED_1MHZ, GPIO_17 );

    	//printf("End initialize!\n");

    	//printf("PN532 wake up\n");
    printf("SPI initialized. ");
    printf("NFC initialized.");
    	pn532_begin(pn532);
    	//printf("End begin().\n");

    	//printf("\nStarting getFirmware version\n");

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
    	//printf("Here it is the frame version:\n| %X | %X | %X | %X | %X |\n", response, ic, version, rev, support);

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
delay(1000);
//printf("\n\n\nInitializing PN532 as Initiator!\n\n\n");
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

			printf("	...sending");
			//return 1;
	}
return 1;
}



int p2p_target(pn532_t * pn532, uint8_t * response){


	uint8_t data[255];
	uint8_t dlen = 0;
	//uint8_t responsetmp[255];

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
		//printf("\nInitialization as target!\n\n");
		//pn532_tg_set_parameters( &pn532, nad, did, atr, rats, picc );
		pn532_set_parameters( pn532, nad, did, atr, rats, picc );
		uint8_t mode = 0x02;	//DEP only
		uint8_t status_target = pn532_tg_init_as_target(pn532, mode );
	//	uint8_t mi = 0;
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
				printf("	...received");
				//printf("\n received: %02x",data[k]);
				//responsetmp[k-7]=data[k];
				response[k-7]=data[k];
			}

			delay(150);
			pn532_tg_get_target_status(pn532);
			//puts("received. stop.");

		}
return 1;
}


void print802154 (keyexc_t * keyexc) {
			//printf("\n ID: %i" , keyexc->id);
  	  	   // printf("\n PROTOCOL: %i" , keyexc->proto);
	printf("\n Display received data for IEEE802154:");
    		printf("\n	CHANNEL: %02x" , keyexc->data802154->channel);
    		printf("\n	PAN ID: ");
    		for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->data802154->panid[i]);
    		}
    		printf("\n	SHORT ADDRESS: ");
    		for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->data802154->short_address[i]);
    		}
    		printf("\n	LONG ADDRESS: ");
    		for(int i=0;i<8;i++){
    		printf("%02x" , keyexc->data802154->long_address[i]);
    		}
    		printf("\n	KEY: ");
    		for(int i=0;i<16;i++){
    		printf("%02x" , keyexc->data802154->key[i]);
    		}

}
void printRPL (keyexc_t * keyexc) {

			printf("\n Display received data for RPL:");
			printf("\n	RPLDATA1:");
	        for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->dataRPL->dataRPL1[i]);
    		}
			printf("\n	RPLDATA2:");
	        for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->dataRPL->dataRPL2[i]);
    		}
			printf("\n	RPLDATA3:");
	        for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->dataRPL->dataRPL3[i]);
    		}
}
void printCOAP (keyexc_t * keyexc) {

			printf("\n Display received data for COAP:");
			printf("\n	COAPDATA1:");
	        for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->dataCOAP->dataCOAP1[i]);
    		}
			printf("\n	COAPDATA2:");
	        for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->dataCOAP->dataCOAP2[i]);
    		}
			printf("\n	COAPDATA3:");
	        for(int i=0;i<2;i++){
    		printf("%02x" , keyexc->dataCOAP->dataCOAP3[i]);
    		}
}
void keyexc_print(keyexc_t * keyexc) {
	//if(keyexc->data802154->flag == 1) {
		print802154(keyexc);
	//}
	//if(keyexc->dataRPL->flag == 1) {
		printRPL(keyexc);
	//}
	//if(keyexc->dataCOAP->flag == 1) {
		printCOAP(keyexc);
	//}
	//else
	//	printf("\nNo data received for any of known protocol");
}
static void callback(void *done_)
{
    volatile int *done = done_;
    *done = 1;
}

int keyexc(keyexc_t * keyexc)
{
	uint8_t ackrx;
	uint8_t acktx = keyexc->ack;



	 main_pid = thread_getpid();

	    printf("\nUART initialization");

	    ringbuffer_init(&rx_buf, rx_mem, 128);
	    ringbuffer_init(&tx_buf, tx_mem, 128);

	    if (uart_init(DEV, BAUD, rx, tx, 0) >= 0) {
	           puts("   ...done");
	       }
	       else {
	           puts("   ...failed");
	           return 1;
	       }



	    pn532_t * pn532 = keyexc->dev;
		pn532_initialization(pn532);
		volatile int done = 0;
      hwtimer_set(TIMEOUT, callback, (void *) &done);
  do {

    	   if(keyexc->node_type == 0) {
    	  			  puts("\n\nTHIS IS GATEWAY");
    	  			 puts("\nWaiting node ID");
    	  			 p2p_target(pn532,&(keyexc->id));	// wait to hear from NFC the ID of the Sensor...
    	  			//keyexc->id=1; // da rimovere

    	  			 hwtimer_wait(6000000);// wait resp
    	  			// printf("\n\nRIOT Gateway --> Linux MySQL Database: ");

    	  			 thread_create(uart_stack, THREAD_STACKSIZE_MAIN, THREAD_PRIORITY_MAIN - 1,
    	  				                    0, uart_thread, keyexc, "uart");
printf("\n Received data from MySQL database for protocols:");
		 // the _rx_cb function will be call when Linux send data. at the end of the process all parameters will be saved.
int fsmstop = 0;
   while (fsmstop == 0) {
		 msg_t msg;
		 msg_receive(&msg);

		 char buf[128];
		 int res = ringbuffer_get(&rx_buf, buf, rx_buf.avail);
		 buf[res] = '\0';
		 int protocol;
		 switch(keyexc->status) {

			 case KEYEXC_IDLE:
				 memcpy(&(keyexc->id),(uint8_t*)buf,1);
				 keyexc->status=KEYEXC_PROTO;
				 break;
			 case KEYEXC_PROTO:

				 //memcpy(&(keyexc->proto),(int*)buf,1);
				 protocol=atoi(buf);
				// printf("\n\nproto ricevuto come int: %i, come hex: %02x, come char: %c.",keyexc->proto,keyexc->proto,keyexc->proto);
				 //printf("\n\nproto ricevuto come int: %i, come hex: %02x, come char: %c.",protocol,protocol,protocol);
				// keyexc->proto = protocol;
				 if(protocol == 1) {
					 printf("\n	IEEE802154");
					 keyexc->data802154->flag = 1;
					 keyexc->status=KEYEXC_CHANNEL;
					 break;

				 }
				 if(protocol == 2) {
				   printf("\n	RPL");
				   keyexc->dataRPL->flag = 1;
				   keyexc->status=KEYEXC_RPLDATA1;
				   break;
				}
				 if(protocol == 3) {
					 printf("\n	COAP");
					 keyexc->dataCOAP->flag = 1;
				     keyexc->status=KEYEXC_COAPDATA1;
				     break;
				}
				 if(protocol == 0){
					 printf("\n	PROTOUNDEF");
					 keyexc->status = KEYEXC_PROTO;
					 break;
				 }


				 else {
					 printf("\n stop receiving.");
					 fsmstop = 1;
					 break;
					// keyexc->status = KEYEXC_FSMSTOP;
				 }



			 case KEYEXC_CHANNEL:
				 memcpy(&(keyexc->data802154->channel),(uint8_t*)buf,1);
				 keyexc->status=KEYEXC_PANID;
				 break;
			 case KEYEXC_PANID:
				 memcpy(keyexc->data802154->panid,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_SHORTADDRESS;
				 break;
			 case KEYEXC_SHORTADDRESS:
				 memcpy(keyexc->data802154->short_address,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_LONGADDRESS;
				 break;
			 case KEYEXC_LONGADDRESS:
				 memcpy(keyexc->data802154->long_address,(uint8_t*)buf,8);
				 keyexc->status=KEYEXC_KEY;
				 break;
			  case KEYEXC_KEY:
				 memcpy(keyexc->data802154->key,(uint8_t*)buf,16);
				 keyexc->status=KEYEXC_PROTO;
				 break;
			  case KEYEXC_RPLDATA1:
			  	 memcpy(keyexc->dataRPL->dataRPL1,(uint8_t*)buf,2);
			  	 keyexc->status=KEYEXC_RPLDATA2;
			     break;
			  case KEYEXC_RPLDATA2:
				 memcpy(keyexc->dataRPL->dataRPL2,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_RPLDATA3;
				 break;
			  case KEYEXC_RPLDATA3:
				 memcpy(keyexc->dataRPL->dataRPL3,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_PROTO;
				 break;
			  case KEYEXC_COAPDATA1:
				 memcpy(keyexc->dataCOAP->dataCOAP1,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_COAPDATA2;
				 break;
			  case KEYEXC_COAPDATA2:
				 memcpy(keyexc->dataCOAP->dataCOAP2,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_COAPDATA3;
				 break;
			  case KEYEXC_COAPDATA3:
				 memcpy(keyexc->dataCOAP->dataCOAP3,(uint8_t*)buf,2);
				 keyexc->status=KEYEXC_PROTO;
				 break;
			   } //switch


 } //while
    	  		     //printf("\nLinux MySQL Database --> RIOT Gateway");

    	  		     keyexc_print(keyexc); //Print received value form MySQL to GAteway!
    	  		     delay(1000);
    	  		     printf("\n Sending data through NFC...\n");
    	  		     p2p_initiator(pn532,&(keyexc->id),1);

    	  			p2p_target(pn532,&ackrx);
    	  			 p2p_initiator(pn532,&(keyexc->proto),1);
    	  			p2p_target(pn532,&ackrx);
    	  			 p2p_initiator(pn532,&(keyexc->data802154->channel),1);
    	  			p2p_target(pn532,&ackrx);
    	  			 p2p_initiator(pn532,keyexc->data802154->panid,2);
    	  			p2p_target(pn532,&ackrx);
    	  			 p2p_initiator(pn532,keyexc->data802154->short_address,2);
    	  			p2p_target(pn532,&ackrx);
    	  			 p2p_initiator(pn532,keyexc->data802154->long_address,8);
    	  			p2p_target(pn532,&ackrx);
    	  			 p2p_initiator(pn532,keyexc->data802154->key,16);
    	  			p2p_target(pn532,&ackrx);
    	  			p2p_initiator(pn532,keyexc->dataRPL->dataRPL1,2);
    	  			p2p_target(pn532,&ackrx);
					p2p_initiator(pn532,keyexc->dataRPL->dataRPL2,2);
    	  			p2p_target(pn532,&ackrx);
					p2p_initiator(pn532,keyexc->dataRPL->dataRPL3,2);
    	  			p2p_target(pn532,&ackrx);
					p2p_initiator(pn532,keyexc->dataCOAP->dataCOAP1,2);
    	  			p2p_target(pn532,&ackrx);
					p2p_initiator(pn532,keyexc->dataCOAP->dataCOAP2,2);
    	  			p2p_target(pn532,&ackrx);
					p2p_initiator(pn532,keyexc->dataCOAP->dataCOAP3,2);


    	 } //if node type 0
           if(keyexc->node_type == 1) {
              	  		  puts("\n\nTHIS IS SENSOR");

              	  		 p2p_initiator(pn532,&(keyexc->id),1); // sensore manda il proprio id

              	  		 printf("\n Receiving data through NFC...");
              	  		 p2p_target(pn532,&(keyexc->id));
              	  	     p2p_initiator(pn532,&acktx,1);
              	  		 p2p_target(pn532,&(keyexc->proto));
              	  		p2p_initiator(pn532,&acktx,1);
              	  		 p2p_target(pn532,&(keyexc->data802154->channel));
              	  		p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->data802154->panid);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->data802154->short_address);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->data802154->long_address);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->data802154->key);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->dataRPL->dataRPL1);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->dataRPL->dataRPL2);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->dataRPL->dataRPL3);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->dataCOAP->dataCOAP1);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->dataCOAP->dataCOAP2);
						 p2p_initiator(pn532,&acktx,1);
						 p2p_target(pn532,keyexc->dataCOAP->dataCOAP3);

						keyexc_print(keyexc); // print received value from Gateway to sensor, via NFC.
               }


//TODO pn532_ss_off(pn532->spi_cs);
//TODO spi_poweroff(SPI_0);
done=1;
printf("\n Key exchange finish.");
     } while (done == 0);

       // printf("\n\n exit with status %02x   \n\n",keyexc->status);

  	  	  	/*printf("\n ID: %i" , keyexc->data802154->id);
  	  	    printf("\n PROTOCOL: %02x" , keyexc->data802154->proto);
    		printf("\n CHANNEL: %02x" , keyexc->data802154->channel);
    		for(int i=0;i<2;i++){
    		printf("\n PAN ID: ");
    		printf("%02x" , keyexc->data802154->panid[i]);
    		}
    		for(int i=0;i<2;i++){
    		printf("\n SHORT ADDRESS: ");
    		printf("%02x" , keyexc->data802154->short_address[i]);
    		}
    		for(int i=0;i<8;i++){
    		printf("\n LONG ADDRESS: ");
    		printf("%02x" , keyexc->data802154->long_address[i]);
    		}
    		for(int i=0;i<16;i++){
    		printf("\n KEY: ");
    		printf("%02x" , keyexc->data802154->key[i]);
    		}*/


return 1;
}




/**
 * @TODO use void * dev instead of pn532
 */
// keyexc_init(&key, &pn532,SENSOR, &id, &channel, panid, short_address, long_address, key);

void keyexc_init(keyexc_t * keyexc, pn532_t * pn532, keyexc_node_t node_type, uint8_t  id, keyexc_802154_t * data802154, keyexc_RPL_t * dataRPL, keyexc_COAP_t * dataCOAP ) {
keyexc->node_type = node_type;
keyexc->id= id;
keyexc->dev = pn532;
//keyexc->data = data;
keyexc->data802154 = data802154;
keyexc->dataRPL = dataRPL;
keyexc->dataCOAP = dataCOAP;
keyexc->status = KEYEXC_IDLE;
keyexc->data802154->flag = 0;
keyexc->dataCOAP->flag = 0;
keyexc->dataRPL->flag = 0;
keyexc->proto=0;
keyexc->rx_count=0;
keyexc->ack=0x00;

/*if(node_type == 0){ // He is the GAteway, initialize internal uart
	uart_t uart = UART_0;
	uint32_t baudrate = (115200U);
if (uart_init(uart, baudrate, _rx_cb, _tx_cb, keyexc) < 0) {
      printf("xbee: Error initializing UART\n");
      return;
  }
}*/

}


void keyexc_set_xbee(kernel_pid_t interfaccia_xbee, keyexc_t * keyexc)
{

//queste funzioni non sono verificate, usale come riferimento, potrebbe mancare qualche argomento.
	/*ng_netapi_set(interfaccia_xbee,NETCONF_OPT_CHANNEL,&(keyexc->data802154->channel),1);
	ng_netapi_set(interfaccia_xbee,NETCONF_OPT_NID,keyexc->data802154->panid,2);
	ng_netapi_set(interfaccia_xbee,NETCONF_OPT_ADDRESS,keyexc->data802154->short_address,2);
	ng_netapi_set(interfaccia_xbee,NETCONF_OPT_ADDRESS_LONG,keyexc->data802154->long_address,8);
	ng_netapi_set(interfaccia_xbee,ng_netconf_enable_t encrypt_status = NETCONF_ENABLE;

	NETCONF_OPT_ENCRYPTION, &encrypt_status,1);
	ng_netapi_set(interfaccia_xbee,NETCONF_OPT_ENCRYPTION_KEY,keyexc->data802154->key,16);
	*/

//Questa potrebbe servire a settare il source adress a 64 bit, ma è da testare.
//ng_netapi_set(interfaccia_xbee, NETCONF_OPT_SRC_LEN, 0, &src_len, sizeof(src_len));

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
