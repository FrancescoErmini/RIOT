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



#define TIMEOUT_S (40)
#define TIMEOUT_US (TIMEOUT_S * 1000 * 1000)
#define TIMEOUT (HWTIMER_TICKS(TIMEOUT_US))

int keyexc_is_equal(uint8_t * known_id, uint8_t * recv_id, int size)
{
	for(int i=0;i<size;i++){
		if (known_id[i] != recv_id[i]){
			return 0;
			}
		}
return 1;
}

int id_check(keyexc_t * keyexc, uint8_t * id_received)
{
	if( keyexc->node_type == SENSOR ){
			if(keyexc_is_equal(keyexc->gateway_id,id_received,4)){
				return 1;
			}

	}
	if( keyexc->node_type == GATEWAY){
		if(keyexc_is_equal(keyexc->node_id,id_received,4)){
				return 1;
		}

	}
	return 0;
}


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





static void callback(void *done_)
{
    volatile int *done = done_;
    *done = 1;
}

int keyexc(keyexc_t * keyexc)
 {	pn532_t * pn532 = keyexc ->dev;
	pn532_initialization(pn532);

	uint8_t resp = 0x00;
	uint8_t info [4] = {0x00, 0x00, 0x00, 0x00 };


	 uint8_t ack = 0x01;
	 uint8_t gateway_id_request = 0x02;
	 uint8_t gateway_id_verified = 0x03;
	 uint8_t gateway_id_failed = 0x04;
	 uint8_t node_id_request = 0x05;
	 uint8_t node_id_verified = 0x06;
	 uint8_t node_id_failed = 0x07;
	 uint8_t key_type_request = 0x08;
	 uint8_t key_type_verified = 0x09;
	 uint8_t key_type_failed = 0x10;
	 uint8_t key_payload_request = 0x11;
	 uint8_t ket_payload_verified = 0x12;
	 uint8_t key_payload_failed = 0x13;


	uint8_t *node_id = keyexc->node_id;
	uint8_t * gateway_id = keyexc->gateway_id;

	uint8_t key_type = keyexc->keyexc_type;

	keyexc->keyexc_status=KEYEXC_IDLE;
	volatile int done = 0;
      hwtimer_set(TIMEOUT, callback, (void *) &done);
  do {



    	   if(keyexc->node_type == 0) {
    	  			  puts("THIS IS GATEWAY");
    	  			  /**ACK CHECK**/
    	  			  p2p_target(pn532,&resp); //gateway stà in attesa di riceve un ack
    	  			  if (ack == resp){    // se riceve ack,
    	  					keyexc->keyexc_status = KEYEXC_ACK_RECEIVED;  //setta lo status in ack received e
    	  					 p2p_initiator(pn532,&ack,1); // manda a sua volta un ack
    	  			  }
    	  			  else {
    	  					keyexc->keyexc_status = KEYEXC_ACK_FAILED; //se riceve un ack sbagliato, fallisce
    	  					return -1;
    	  					  }
    	  			  /**ID CHECK**/
    	  			  //ID GATEWAY CHECK
    	  			  p2p_target(pn532,&resp); // Gateway si mette in attesa di ricevere istruzioni,
    	  			 if (resp == gateway_id_request){  //se il comando di richiesta id è corretto,
    	  				 keyexc->keyexc_status = KEYEXC_GATEWAY_ID_REQUEST_RECEIVED; // aggiorno lo status
    	  				 p2p_initiator(pn532,gateway_id,4); //e manda al nodo il suo id. ovvero gateway_id.
    	  			 }
    	  			 else {
    	  				keyexc->keyexc_status = KEYEXC_GATEWAY_ID_REQUEST_FAILED;
    	  				return -1;
    	  			 }


    	  			  p2p_target(pn532,&resp); // aspetto risposta (ok/no) dal sensore
    	  			  if (resp == gateway_id_verified){ //se il sensore mi ha risposto ok
    	  				  keyexc->keyexc_status = KEYEXC_GATEWAY_ID_VERIFIED; // aggiorna lo status
    	  			  }
    	  			  else { // if (resp == gateway_id_failed)
    	  				  keyexc->keyexc_status= KEYEXC_GATEWAY_ID_FAILED; // altrimenti esco.
    	  				  return -1;
    	  			  }
    	  			  // ID NODE CHECK
	  				  p2p_initiator(pn532,&node_id_request,1); // poi,  chiede al sensore di mandargli il suo id.

    	  			  p2p_target(pn532,info); // aspetto di ricevere l'id del nodo
    	  			  if (id_check(keyexc,keyexc->node_id)) { //se gateway riconosce il nodo ***
    	  				  keyexc->keyexc_status = KEYEXC_NODE_ID_VERIFIED; //aggiorno lo status
    	  				  p2p_initiator(pn532,&node_id_verified,1);  //e lo comunico al nodo
    	  			  }
    	  			  else {
    	  				  keyexc->keyexc_status = KEYEXC_NODE_ID_FAILED;
    	  				  p2p_initiator(pn532,&node_id_failed,1);
    	  				  return -1;
    	  			  }
    	  			  /** KEY TYPE CHECK **/
    	  			  p2p_target(pn532,&resp);
    	  			  if (resp == key_type_request) {
    	  				  keyexc->keyexc_status = KEYEXC_TYPE_REQUEST_RECEIVED; //aggiorno lo status
    	  				  p2p_initiator(pn532,&key_type,1);   // e lo comunico al sensore
    	  			  }
    	  			  else {
    	  				  keyexc->keyexc_status = KEYEXC_TYPE_REQUEST_FAILED;
    	  				 // p2p_initiator(pn532,key_type_fail,1);
    	  				  return -1;
    	  			  }

    	  			  /** KEY PAYLOAD **/
    	  			  p2p_target(pn532,&resp); // aspetto di ricevere la richiesta per la chiave
    	  			  if (resp == key_payload_request) { // se la richiesta è giusta
    	  			 				  keyexc->keyexc_status = KEYEXC_PAYLOAD_REQUEST_RECEIVED; //aggiorno lo status
    	  			 				  p2p_initiator(pn532,keyexc->key_payload,16);   // e mando la chiave al sensore
    	  			 			  }
    	  			 			  else { puts("something went wrong");
    	  			 				 // keyexc->keyexc_status = KEYEXC_REQUEST_FAIL;
    	  			 				  return -1;
    	  			 			  }
    	  		//	p2p_target(pn532,&resp);
    	  			 /* p2p_target(pn532,key_payload_check);
    	  			  if (key_payload_check == key_payload_verified){
    	  				  keyexc->keyexc_status = KEY_PAYLOAD_VERIFIED;
    	  				  return 2;
    	  			  }
    	  			  else {
    	  				  keyexc->keyexc_status = KEY_PAYLOAD_FAIL;
    	  				  return -1;
    	  			  }*/


    	 }
           if(keyexc->node_type == 1) {
              	  		  	  puts("THIS IS SENSOR");
              	  		  	  /** ACK **/
              	  		  	 p2p_initiator(pn532,&ack,1); // sensore manda il primo ack
              	  		  	 p2p_target(pn532,&resp);
              	  		  	 if (ack == resp){ // se  riceve un ack correttamente,
              	  		  		 keyexc->keyexc_status = KEYEXC_ACK_RECEIVED; //aggiorno lo status
              	  		  	 }
              	  		  	 else {
              	  		  		 keyexc->keyexc_status = KEYEXC_ACK_FAILED;
              	  		  		 return -1;
              	  		  	  }
              	  		  	 /** ID CHECK **/
              	  		  	 // ID GATEWAY CHECK
          	  		 		  p2p_initiator(pn532,&gateway_id_request,1); //e chiedo al gatway di mandargli l'ID
              	  	 		  p2p_target(pn532,info);
              	  	 		  if(id_check(keyexc,info)){   //se l'ID ricevuto corrispone a quello conosciuto
              	  	 			  keyexc->keyexc_status = KEYEXC_GATEWAY_ID_VERIFIED; //aggiorna lo status
              	  	 			  p2p_initiator(pn532,&gateway_id_verified,1);   //e notificalo al gateway
              	  	 		  }
              	  	 		  else {
              	  	 		     keyexc->keyexc_status = KEYEXC_GATEWAY_ID_FAILED;
              	  	 			 p2p_initiator(pn532,&gateway_id_failed,1);
              	  	 			 return -1;
              	  	 		  }
              	  	 		  // NODE ID CHECK
              	  	 		  p2p_target(pn532,&resp);  // aspetto che gateway chieda l' ID
              	  	 		  if (resp==node_id_request){
              	  	 			  keyexc->keyexc_status = KEYEXC_NODE_ID_REQUEST_RECEIVED;
              	  	 			  p2p_initiator(pn532,keyexc->node_id,4); //mando l'ID al Gateway
              	  	 		  }
              	  	 		  else {
              	  	 			keyexc->keyexc_status = KEYEXC_NODE_ID_REQUEST_FAILED;
              	  	 			// TODO: send notify with 4 byte address?
              	  	 			return -1;
              	  	 		  }
              	  	 		  p2p_target(pn532,&resp); // se il gateway mi risponde ok
              	  	 		  if (resp == node_id_verified){
              	  	 			  keyexc->keyexc_status = KEYEXC_NODE_ID_VERIFIED; //aggiorno lo status

              	  	 		  }
              	  	 		  else { // if(resp == node__id_failed)
              	  	 			 keyexc->keyexc_status = KEYEXC_NODE_ID_FAILED;
              	  	 			 return -1;
              	  	 		  }
              	  	 		  /** KEY TYPE **/
              	  	 		 p2p_initiator(pn532,&key_type_request,1); // dico al gateway di quale tipo di chiave ho bisogno
              	  	 		 p2p_target(pn532,&resp); // se mi risponde ok,
              	  	 		 if (resp == key_type){
              	  	 			 	keyexc->keyexc_status = KEYEXC_TYPE_VERIFIED; //aggiorno lo status

              	  	 		}
              	  	 		else {
              	  	 			  keyexc->keyexc_status = KEYEXC_TYPE_FAILED;
              	  	 			  return -1;
              	  	 	    }
              	  	 		 /** KEY PAYLOAD **/

              	  	 		 p2p_initiator(pn532,&key_payload_request,1); // il sensore  chiede al gateway la chiave
              	  	 		 p2p_target(pn532, keyexc->key_payload);
              	  	 		 keyexc->keyexc_status = KEYEXC_PAYLOAD_VERIFIED;
              	  	 		 //p2p_initiator(pn532,key_payload_verified,1);
printf("fine sensor");
              	  	 	//p2p_initiator(pn532,&ack,1);

              	  	 			// keyexc->keyexc_status = KEYEXC_PAYLOAD_FAILED;
              	  	 		//	p2p_initiator(pn532,key_payload_failed,1);

               }

printf("WAIT TIMEOUT");
done = 1;

     } while (done == 0);

      printf("\n\nexit with status %02x   \n\n",keyexc->keyexc_status);
    	for(int l=0;l<16;l++)
    	{
    		printf("\npayload_recived[%i]: %02x\n" , l,keyexc->key_payload[l]);
    	}
return 1;
}




/**
 * @TODO use void * dev instead of pn532
 */
void keyexc_init(keyexc_t * keyexc, pn532_t * pn532, keyexc_node_t node_type,keyexc_type_t keyexc_type, uint8_t * node_id, uint8_t * gateway_id, uint8_t * keypayload)
 {
keyexc->node_type = node_type;
keyexc->keyexc_type= keyexc_type;
keyexc->key_payload = keypayload;
keyexc->node_id = node_id;
keyexc->gateway_id = gateway_id;

keyexc->dev = pn532;
/**
 * @TODO timeout variable
 */
/**
 * @TODO how to add new id?
 */
//keyexc->node_id=node_id;

/*check key length */
	if(keyexc_type == KEY_802154){
		keyexc->key_size = 16;
	}
	if (keyexc_type == KEY_RPL){

	}
	if (keyexc_type == KEY_COAP){

	}

	if(node_type == SENSOR){

	}
	if (node_type == GATEWAY){

	}



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
