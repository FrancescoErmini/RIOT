/**
 *	Copyright (c) 2012, Adafruit Industries
 *	All rights reserved.
 */
/*
 * Copyright (C) 2014 Freie Universität Berlin
 * Copyright (C) 2013 INRIA
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
/**************************************************************************/
/**
 * @ingroup     drivers_pn532
 * @{
 * @file        pn532.c
 * @brief       C Library for the PN532 written for Riot OS.
 * 				SPI Driver for NXP's PN532 NFC/13.56MHz RFID Transceiver
 * 				This is a library to connect a PN532 chip on a Sam3x CPU.
 * 				See the README file for details.
 *
 * 				@TODO RFConfiguration
 *
 * @author      University of Florence
 * @author      Simone Pecora <pecora.simone@gmail.com>
 * @}
 */
/*************************************************************************/
#include "pn532.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
//#include <hwtimer_cpu.h>
#include <periph/gpio.h>
#include <periph/spi.h>
#include <periph_conf.h>
#include "debug.h"
#include "errno.h"
#include "xtimer.h"
/* Uncomment these lines to enable debug output for PN532, MIFARE or P2P */
#define ENABLE_DEBUG    (0)
#define MIFAREDEBUG	1
//#define P2PDEBUG	1
//#define PN532DEBUG 1

/* Uncomment these lines to enable reverse bit order function before to send and after reading msg */
#define LSBfirst 1		//Reverse bit order

/* @brief   Delay required by PN532 to wake up or seep, 2ms
 */
//#define SPI_SS_DELAY               (2U * 1000U)
#define SPI_DELAY_1ms			   (1U * 1000U)
#define SPI_DELAY_100us			   (100U)
/* Set delay(ms) */
#define delay(X)	(xtimer_sleep(1000*X))
#define ACKSIZE 6
uint8_t pn532ack[ACKSIZE] = {0x00, 0x00, 0xFF, 0x00, 0xFF, 0x00};
#define NACKSIZE 6
uint8_t pn532nack[NACKSIZE] = {0x00, 0x00, 0xFF, 0xFF,0x00, 0x00};

#define PN532_BUFFSIZE	265
uint8_t writebuffer[PN532_BUFFSIZE];
uint8_t readbuffer[PN532_BUFFSIZE];

uint8_t preamblebuffer[4] = { PN532_SPI_DATAWRITE, PN532_PREAMBLE, PN532_STARTCODE1, PN532_STARTCODE2 };




/**************************************************************************/
/*	@brief  Instantiates a new PN532 SPI communication	*/
/**************************************************************************/

#define SS			PIO_PA14	//pin 23 on the GPIO
#define SSGPIO 		1			//if you want to use GPIO to control SS/CS/NSS
#define GPIO_0_EN	1			//Enable GPIO_0. pin 23 on the GPIO
#define SPI_0_EN	1			//Enable SPI_0
//#define spi_cs		0

//static int spi_dev = 0;			//SPI_0
//static int spi_mode = 0; 		//SPI_CONF_FIRST_RISING ( spi.c )
//static int spi_speed = 0; 	//SPI_SPEED_100KHZ
//static int spi_speed = 1; 	//SPI_SPEED_400KHZ
//static int spi_speed = 2;		//SPI_SPEED_1MHZ
//static int spi_speed = 3;		//SPI_SPEED_5MHZ
//static int spi_speed = 4; 	//SPI_SPEED_10MHZ

/**************************************************************************/
/**
 * @brief	Initialization PN532 SPI communication.
 * @brief	Setting up SPI device, mode and speed.
 *
 */
/**************************************************************************/

int pn532_init_master(pn532_t * dev, spi_t spi_dev, spi_conf_t spi_mode, spi_speed_t spi_speed, gpio_t spi_cs){
/* Check device and spi parameter */
	   if (dev == NULL) {
		   DEBUG("NO DEVICE DESCRIPTOR");
	        return -ENODEV;
	    }
	    if (spi_dev >= SPI_NUMOF) {
	    	DEBUG("SELECTED SPI NOT SUPPORTED");
	        return -ENXIO;
	    }

	/*  configure  spi device descriptor  */
	dev ->spi_dev = spi_dev;
	dev->spi_mode = spi_mode;
	dev->spi_speed = spi_speed;
	dev->spi_cs = spi_cs;

	delay(2);
	DEBUG("SPI_%i not yet initialized as master, cs: GPIO_%i, mode: %i, speed: %i\n", dev->spi_dev, dev->spi_cs, dev->spi_mode, dev->spi_speed);

    int res = spi_init_master( spi_dev, spi_mode, spi_speed );
    if (res < 0) {
        DEBUG("SPI_init_master: error initializing SPI_%i device (code %i)\n", dev->spi_dev, res);
        return -1;
    }
    res = gpio_init(spi_cs, GPIO_DIR_OUT, GPIO_PULLUP); //GPIO_PULLUP = 1
    //res = gpio_init_out(spi_cs, 1); //GPIO_PULLUP = 1

    if (res < 0){
        DEBUG("error initializing GPIO_%i as CS line (code %i)\n", dev->spi_cs, res);
        return -1;
    }
    /* pn532 off */
    pn532_ss_off(dev->spi_cs);
    DEBUG("SPI_%i successfully initialized as master, cs: GPIO_%i, mode: %i, speed: %i\n", dev->spi_dev, dev->spi_cs, dev->spi_mode, dev->spi_speed);
	delay(2);
    return 0;
}

/**************************************************************************/
/**
 *   @brief  	Low-level SPI write. Send 1 byte to PN532
 *
 *   @param  	p       8-bit command to write to the SPI bus
 *   @return	res		1 if the transfer is without errors
 */
/**************************************************************************/
uint8_t pn532_spi_write(pn532_t * dev,uint8_t p) {

	/* Reverse bit order before write on SPI bus. PN532 uses LSB SPI m*/
		p = reverse(p);

		int res = spi_transfer_byte(dev->spi_dev, p, NULL);
		/**
		 * @TODO is this useful?
		 */
		p = reverse(p);

		if (res == 1){
			DEBUG("Transferred byte on the SPI bus with value: %X\n", p);

		}
		else {
			/**
			 * TODO use errno
			 */
		}
		delay(1);

	return res;
}

/**************************************************************************/
/**
 * 	@brief  	Low-level SPI write. Send an array of bytes to PN532
 *
 * 	@param 		buffer		Pointer to the buffer we want to send to PN532
 * 	@param		blen		Buffer length
 *
 * 	@returns	n			Number bytes sent
 */
/**************************************************************************/
uint8_t pn532_spi_transfer_bytes(pn532_t * dev, uint8_t *buffer, uint8_t blen){

	#ifdef LSBfirst
		for(uint8_t i=0; i<blen; i++){
			uint8_t rev = reverse(buffer[i]);
			buffer[i] = rev;
		}
	#endif

	int n = spi_transfer_bytes(dev->spi_dev, (char*)buffer, NULL, blen);

	#ifdef LSBfirst
		for(uint8_t i=0; i<blen; i++){
			uint8_t rev = reverse(buffer[i]);
			buffer[i] = rev;
		}
	#endif
n = (uint8_t) n;
	return n;
}

/**************************************************************************/
/**
 *  @brief  Low-level SPI read wrapper
 *
 *  @returns The 8-bit value that was read from the SPI bus
 */
/**************************************************************************/
uint8_t pn532_spi_read(pn532_t * dev){

	uint8_t in;
	//char inn = (char) in;
	int res;

	res = spi_transfer_byte(dev->spi_dev, 0, (char*)&in);

//	in = (uint8_t) inn;
	/* Reverse bit order before reading. PN532 uses SPI in LSB mode. */
		in = reverse(in);

		if ( res == 1 ){
			DEBUG("Read on SPI bus value %X\n", in);
		}
		else{
			/**
			 * @TODO errno msg
			 */
		}
delay(0.1);

	return in;
}

/**************************************************************************/
/**
 *	@brief  Low-level SPI array order change
 */
/**************************************************************************/

//uint8_t exchange(uint8_t *buff, uint8_t blen){
//
//	uint8_t end = blen - 1;
//	for (int i=0; i < blen / 2; i++) {
//		uint8_t tmp = buff[i];
//		buff[i] = buff[end];
//		buff[end] = tmp;
//		end--;
//	}
//	for ( int i=0; i < blen; i++){
//		printf("%X\n", buff[i]);
//	}
//
//	return 1;
//}

/**************************************************************************/
/*
 *	@brief  Low-level SPI reverse char byte
 *
 *	@returns The 8-bit value reversed
 */
/**************************************************************************/
unsigned char reverse(unsigned char b) {
   b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
   b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
   b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
   return b;
}

/**************************************************************************/
/**
 *	@brief  Low-level SPI CS set the given GPIO to LOW/HIGH
 */
/**************************************************************************/

void pn532_ss_on(gpio_t spi_cs){
	gpio_clear(spi_cs); //Set the given GPIO pin to LOW
	delay(2);
}

void pn532_ss_off(gpio_t spi_cs){
	gpio_set(spi_cs);  //Set the given GPIO pin to HIGH
}

/**************************************************************************/
/**
 *  @brief  Wake up PN532 board
 *  @brief	We also could send a wake up frame but it should just works as it is.
 */
/**************************************************************************/
/**
 *
 * @TODO: delete this function?
 *
 */
void pn532_begin(pn532_t * dev) {

	DEBUG("Wake up PN532 Shield\n");
	pn532_ss_on(dev->spi_cs);		//Set the given GPIO pin to LOW

	//Clean writebuffer[] and readbuffer[]
	for(int i=0; i<PN532_BUFFSIZE; i++){
		writebuffer[i] = 0x00;
		readbuffer[i] = 0x00;
	}

	DEBUG("Writebuffer and readbuffer cleared...\n");
	/*
	//uint8_t cc[] = { 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x04, 0xFC, 0xD4, 0x14, 0x01, 0x01, 0x16, 0x00 };
	//uint8_t cc[] = { 0x55, 0x55, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0xFD, 0xD4, 0x14, 0x01, 0x017, 0x00 };
	uint8_t cc[] = { 0x55,0x55,0x00,0x00,0x00,0x00,0x00,0xff,0x03,0xfd,0xd4,0x14,0x01,0x17,0x00,0x00,0xff,0x03,0xfd,0xd4,0x14,0x01,0x17,0x00 };
	uint8_t clength = sizeof(cc);

	#ifdef PN532DEBUG
		printf("Wakeup sequence before reversing it:\n");
		for(int i=0; i<clength; i++){
			printf("| %X |", cc[i]);
		}
		printf("END\n");
	#endif

	pn532_spi_transfer_bytes( cc, clength);

	#ifdef PN532DEBUG
		printf("Wakeup sequence after reversing it:\n");
		for(int i=0; i<clength; i++){
			printf("| %X |", cc[i]);
		}
		printf("END\n");
	#endif
	*/

	pn532_ss_off(dev->spi_cs);		//Set the given GPIO pin to HIGH
}

/**************************************************************************/
/**
 *  @brief		Write a command to the PN532, automatically inserting the
 *				preamble and required frame details (checksum, len, etc.)
 *
 *  @param		buff		Pointer to the writebuffer
 *  @param  	cmd			Pointer to the command buffer
 *  @param  	cmdlen		Command length in bytes
 *
 *  @returns				Num bytes transferred on the SPI bus
 */
/**************************************************************************/
uint8_t pn532_spi_write_command(pn532_t * dev, uint8_t *buff, uint8_t* cmd, uint8_t cmdlen) {
	pn532_ss_on(dev->spi_cs);

	DEBUG("\n\nSending command to PN532: ");

	//Making dynamic command frame length: |DW| - - - |PDn|
	uint8_t length = sizeof(preamblebuffer) + 3 + cmdlen;
	uint8_t sum = 0;
	uint8_t checksum = 0;

	int end = 0;
	int j = 0;
	for( int i=0; i<length; i++ ){
		switch (i)
		    {
			case 0:
				buff[i] = preamblebuffer[i];		//DataWrite
				break;

			case 1:
				buff[i] = preamblebuffer[i];		//Preamble
				break;

			case 2:
				buff[i] = preamblebuffer[i];		//StartCode1
				break;

			case 3:
				buff[i] = preamblebuffer[i];		//Startcode2
				break;

		    case 4:
		    	buff[i] = ++cmdlen;					//LEN ( cmdlen + TFI )
		    	break;

		    case 5:
		    	buff[i] = ~cmdlen + 1;				//LCS checksum of LEN
		    	break;

		    case 6:
		    	buff[i] = PN532_HOSTTOPN532;		//TFI
		    	sum = PN532_HOSTTOPN532;
		    	break;

		    default:
		    	buff[i] = cmd[j];					//Writing command frame on the writebuffer[]
		    	sum += cmd[j];
		    	j++;
		    }
	}
	checksum = ~sum + 1;							//DCS frame creating
	buff[length] = checksum;						//DCS
	buff[length+1] = PN532_POSTAMBLE;				//POSTAMBLE field
	length += 2;

		DEBUG("writing the frame to PN532...\n");

	//Sending made frame from host to PN532
	end = pn532_spi_transfer_bytes(dev, buff, length);

		DEBUG("Wrote %i bytes.\n", end);

	#ifdef PN532DEBUG
		for(int i=0; i<length; i++){
		  printf("| %X |",buff[i]);
		}
		printf("END.\n\n");
	#endif

	pn532_ss_off(dev->spi_cs);
	return end;

}

/**************************************************************************/
/**
 *  @brief  Read a command from the PN532, automatically inserting the
 *  @brief	preamble and required frame details (checksum, len, etc.)
 *
 *  @param  buffer      Pointer to the readbuffer
 *
 *  @returns            Number of bytes read, -2,-3 on error
 */
/**************************************************************************/
uint16_t pn532_read_pn(pn532_t * dev, uint8_t *buffer ){

	uint8_t timeout = PN532_ACK_WAIT_TIME;
	uint16_t result;
	uint8_t len;
/**
 * @TODO use errno
 */
	do{
		//Until RDY=1
		while (!pn532_is_ready(dev)) {
		        timeout--;
		        if (0 == timeout) {
		            DEBUG("Time out when waiting for ACK\n");
		            result = PN532_TIMEOUT;
		            break;
		        }
		        delay(10);
		}

		pn532_ss_on(dev->spi_cs);
		pn532_spi_write(dev,PN532_SPI_DATAREAD);

		//Starting to read on the SPI bus
		uint8_t res2  = pn532_spi_read(dev);
		uint8_t res1;
		do{
			delay(1);
			res1 = res2;
			res2 = pn532_spi_read(dev);
		}while( !(0x00 == res1 && 0xFF == res2) );
		//Read LEN field
		len = pn532_spi_read(dev);
		uint8_t lcs = pn532_spi_read(dev);
		if(0x00 != (uint8_t)(lcs+len) ){
			result = PN532_INVALID_FRAME;
			DEBUG("PN532_INVALID_FRAME %d %d.\n", lcs, len);
			break;
		}
		//Starting storage to readbuffer from TFI to the last PDn field
		uint8_t sum = 0;
		for(uint8_t i=0; i<len; i++){
			delay(1);
			buffer[i] = pn532_spi_read(dev);
			sum += buffer[i];
		}

		if( PN532_PN532TOHOST != buffer[0] ){
			result = PN532_INVALID_FRAME;
			break;
		}

		//Read DCS field
		uint8_t dcs = pn532_spi_read(dev);
		if( 0x00 != (uint8_t)(sum + dcs) ){
			 DEBUG("checksum is not ok\n");
			 result = PN532_INVALID_FRAME;
			 break;
		}
		//Read Postamble field
		uint8_t pst = pn532_spi_read(dev);
		if( 0x00 != pst ){
			result = PN532_INVALID_FRAME;
			break;
		}
		result = len;
	}while(0);

		DEBUG( "Here it is the read buffer.\n" );
		for( uint8_t i=0; i < len; i++){
			DEBUG( "| [%i]: %X |", i, buffer[i] );
		}
		DEBUG( "\n" );

	pn532_ss_off(dev->spi_cs);

	return result;
}

/**************************************************************************/
/**
 * 	@brief  	Write to PN532 Status Read byte and read PN532 Status if is
 * 	@brief		Ready. RDY = 0x01
 *
 * 	@returns	Status of the PN532
 */
/**************************************************************************/
uint8_t pn532_is_ready(pn532_t * dev){

	pn532_ss_on(dev->spi_cs);
	//Write the Status Read = 0x01
    pn532_spi_write(dev, PN532_SPI_STATREAD);
//    uint8_t status = pn532_spi_read() & 1;
    uint8_t status = pn532_spi_read(dev);
    pn532_ss_off(dev->spi_cs);
    return status;
}

/**************************************************************************/
/**
 *  @brief		Write to PN532 Status Read byte and read PN532 Status if is
 *   			Ready. RDY = 0x01 Then the host send the Status Read field.
 *   			After that the host receive from PN532 the ACK frame.
 *
 *	@returns	Negative on error, 1 on command successfully received
 */
/**************************************************************************/
uint8_t pn532_check_ack (pn532_t * dev) {

	uint8_t timeout = PN532_ACK_WAIT_TIME;
	//Read ACK frame sent from PN532 and check it
	while (!pn532_is_ready(dev)) {				//Stay here until RDY=1
		timeout--;
	    if (0 == timeout) {
	    	printf("Time out when waiting for ACK\n");
	        return PN532_TIMEOUT;
	    }
	    delay(10);
	}
	if (pn532_read_ack(dev)) {
		printf("Invalid ACK!\n");
		return PN532_INVALID_ACK;
	}
	return 1;

}

/**************************************************************************/
/**
 *	@brief  	Read frame from the PN532 and check the ACK received
 *				First of all I send DATAREAD (0x03) field
 *
 *	@returns	0 if the ACK frame is correct
 */
/**************************************************************************/
uint8_t pn532_read_ack(pn532_t * dev){

	pn532_ss_on(dev->spi_cs);
	uint8_t ackbuff[ACKSIZE];
	//Write Data Read = 0x03
	pn532_spi_write(dev,PN532_SPI_DATAREAD);

	for(int i=0; i<ACKSIZE; i++){
		delay(1);
		ackbuff[i] = pn532_spi_read(dev);
	}

	pn532_ss_off(dev->spi_cs);


	#ifdef PN532DEBUG
	DEBUG("START ACK FRAME:\n");
		for(int i=0; i<ACKSIZE; i++){
			printf("| %X |", ackbuff[i]);
		}

	DEBUG("END ACK FRAME\n");
    #endif
	return memcmp( ackbuff, pn532ack, sizeof(pn532ack));
}

/**************************************************************************/
/**
 *	@brief  	Write ACK frame from host to PN532.
 *				This command is optional for some functions.
 *
 *	@returns	1 when the ACK frame is sent
 */
/**************************************************************************/
void pn532_write_ack(pn532_t * dev){
	//Send ACK frame from host to PN532
	DEBUG("Writing the ACK frame to PN532\n");
	//Write from host to PN532 ACK frame
	pn532_ss_on(dev->spi_cs);
	pn532_spi_write(dev, PN532_SPI_DATAWRITE);
	pn532_spi_transfer_bytes(dev, pn532ack, sizeof(pn532ack) );
	delay(1);
	pn532_ss_off(dev->spi_cs);
	return;
}
/**************************************************************************/
/**
 *	@brief  	Prints a hexadecimal value in plain characters
 *
 *	@param		data      Pointer to the uint8_t data
 *	@param  	numBytes  Data length in bytes
 *
 *	@returns	print	Number of bytes printed
 */
/**************************************************************************/
uint8_t pn532_print_hex( uint8_t *data, uint8_t numBytes){
	uint8_t print = 0;

	for (uint8_t i = 0; i < numBytes; i++) {
	       printf("| 0x%2X ", data[i]);
	        print++;
	}
	printf("|\n");
	return print;


}
/**************************************************************************/
/**
 *	@brief  	Checks the firmware version of the PN5xx chip
 *
 *	@params		pointer to Result, IC, Version, Rev and Support PN5352 fields
 *
 *	@returns	1 on success
 *	@returns	-1 on error
 */
/**************************************************************************/
uint32_t pn532_get_firmware_version(pn532_t * dev, uint8_t *res, uint8_t *ic, uint8_t *version, uint8_t *rev, uint8_t *support){

	uint8_t cc[1];
	cc[0] = PN532_COMMAND_GETFIRMWAREVERSION;		//Initialization command vector

	//Send the command from the udoo to the PN532
	pn532_spi_write_command(dev, writebuffer, cc, 1);

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {

	DEBUG("\n\nACK frame received from PN532 wrong!\n\n");

		return PN532_INVALID_ACK;
	}

	//Read response frame. For the getFirmwareVersion total length should be 13 bytes
	pn532_read_pn(dev, readbuffer );
	/* Send response to variables */
	*res = readbuffer[1];
	*ic = readbuffer[2];
	*version = readbuffer[3];
	*rev = readbuffer[4];
	*support = readbuffer[5];

	pn532_write_ack(dev);

	return 1;
}
/**************************************************************************/
/**
 *	@brief		Get General Status allows the host controller to know at
 *     			a given moment the complete situation of the PN532
 *
 */
/**************************************************************************/
void pn532_get_general_status(pn532_t * dev){

	uint8_t cc[1] = {PN532_COMMAND_GETGENERALSTATUS};

		DEBUG("Get the general status of the PN532.\n");

	pn532_spi_write_command(dev, writebuffer, cc, sizeof(cc) );

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
			DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return;
	}

	#ifdef PN532DEBUG
	// Read response packet
	uint8_t len = pn532_read_pn(dev, readbuffer );
		//Read status information frame
		DEBUG("Start General Status frame.\n");

		for(int i=0; i < len; i++){
			printf("| [%i] %X |", i, readbuffer[i]);
		}
		DEBUG("End General Status frame.\n");
	#endif



	//pn532_write_ack();

}
/**************************************************************************/
/*
 *	@brief		This command is used to set internal parameters of the PN532
 *				and then to configure its behavior regarding different cases.
 *
 *	@params		nad		Use of the NAD information in case of initiator configuration
 *	@params		did		DID information in case of initiator configuration
 *	@params		autatr	Automatic generation of the ATR_RES in case of target
 *	@params		autrats	Automatic generation of the RATS in case of ISO/IEC144443-4 PCD mode
 *	@params		piss	Emulation of a ISO/IEC144443-4 PICC is enabled
 *
 *	@returns	1 on success, 0 or negative number on error
 */
/**************************************************************************/
uint8_t pn532_set_parameters(pn532_t * dev, uint8_t nad, uint8_t did, uint8_t autatr, uint8_t autrats, uint8_t picc ) {

	uint8_t flag = 0;
	if( nad != 1 && nad != 0x00 ) {
		return 0;
	} else {
		flag |= nad;
	}
	if( did != 1 && did != 0x00 ) {
		return 0;
	} else {
		did <<= 1;
		flag |= did;
	}
	if( autatr != 1 && autatr != 0x00 ) {
		return 0;
	} else {
		autatr <<= 2;
		flag |= autatr;
	}
	if( autrats != 1 && autrats != 0x00 ) {
		return 0;
	} else {
		autrats <<= 4;
		flag |= autrats;
	}
	if( picc != 1 && picc != 0x00 ) {
		return 0;
	} else {
		picc <<= 5;
		flag |= picc;
	}
		DEBUG("Value Flag: %X", flag);


	uint8_t cc[2];
	cc[0] = PN532_COMMAND_SETPARAMETERS;
	cc[1] = flag;
	pn532_spi_write_command(dev, writebuffer, cc, 2 );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	//Read response
	uint16_t len = pn532_read_pn(dev, readbuffer );
	if( len != 2 ) {
		return PN532_INVALID_FRAME;
			DEBUG("Status incorrect! Status value: %X", readbuffer[2]);
	}
	return 1;
}
/**************************************************************************/
/*
 *	@brief		Configures the SAM (Secure Access Module)
 *				This command is used to select the data flow path by
 *				configuring the internal serial data switch.
 *
 *	@returns	True on success, false on error
 */
/**************************************************************************/
bool pn532_SAM_config(pn532_t * dev) {

	uint8_t cc[4];
	cc[0] = PN532_COMMAND_SAMCONFIGURATION;
	cc[1] = 0x01; // normal mode, SAM is not used!
	cc[2] = 0x14; // timeout 50ms * 20 = 1 second
	cc[3] = 0x00; //Not use IRQ pin!

		DEBUG("SAMConfig.\n");

	pn532_spi_write_command(dev, writebuffer, cc, sizeof(cc) );

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {

				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");

		return PN532_INVALID_ACK;
	}

	// Read response packet
	pn532_read_pn(dev, readbuffer );

	pn532_write_ack(dev);

	if( readbuffer[1] == 0x15 ) {
		return true;
	}
	return false;
}

/**************************************************************************/
/**
 * 						DEP Chaining Part
 */
/**************************************************************************/
/**
 *	@brief		This command is used by a host controller to activate a target
 *   			using either active or passive communication mode.
 *   			Actpass is the communication mode requested by the host controller
 *   			(0x00 = passive; 0x01 = active)
 *   			Br is the baudrate to be used during the activation
 *   			(0x00 = 106 kbps; 0x01 = 212; 0x02 = 424)
 *
 *	@params		buffer	Byte length we want that PN532 as target send to the initiator
 *	@params		blen	Buffer length
 *
 *	@returns	1 on success, 0 or negative number on error
 */
/**************************************************************************/
uint16_t pn532_in_jump_for_dep (pn532_t * dev, uint8_t actpass, uint8_t br, uint8_t next, uint8_t * atrres, uint8_t * alen ) {

	uint8_t cc[4];
	cc[0] = PN532_COMMAND_INJUMPFORDEP;
	cc[1] = actpass;
	cc[2] = br;
	cc[3] = next;
	pn532_spi_write_command(dev, writebuffer, cc, 4 );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	delay(500);	//Read response
	uint16_t len = pn532_read_pn(dev, readbuffer );
	if ( readbuffer[2] != 0x00 ) {
			DEBUG("Status InJumpForDEP incorrect! Value: %X\n", readbuffer[2] );
			return PN532_INVALID_FRAME;
	}
	memcpy( atrres, readbuffer, len );
	*alen = len;
	return 1;
}

/**************************************************************************/
/**
 * @brief		This command is used by a host controller to activate a target
 *				using either active or passive communication mode.
 *				If a target is in the field, it will then be ready for PSL or
 * 				DEP exchanges.
 *
 * @params		buffer	Byte length we want that PN532 as target send to the initiator
 * @params		blen	Buffer length
 *
 */
/**************************************************************************/
uint16_t pn532_in_jump_for_psl (pn532_t * dev, uint8_t actpass, uint8_t br, uint8_t next ) {

	uint8_t cc[4];
	cc[0] = PN532_COMMAND_INJUMPFORPSL;
	cc[1] = actpass;
	cc[2] = br;
	cc[3] = next;
	pn532_spi_write_command(dev, writebuffer, cc, 4 );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	//Delay
	delay(1500);
	//Read response
	//uint16_t len =
			pn532_read_pn(dev, readbuffer );
	if ( readbuffer[2] != 0x00 ) {
			printf("Status InJumpForPSL incorrect! Value: %X\n", readbuffer[2] );
		return PN532_INVALID_FRAME;
	}

	return 1;
}

/**************************************************************************/
/*
 *  @brief		This command is used to support protocol data exchanges
 *				between the PN532 as initiator and a target.
 *
 *	@params		buffer array data to send to the target
 *	@params		blen length array
 *	@params		tg Byte containing the logical number of the relevant target
 *	@params		mi More Information byte
 *
 *  @returns	1 on success, 0 or negative number on error
 */
/**************************************************************************/
uint8_t pn532_in_data_exchange (pn532_t * dev, uint8_t * buffer, uint8_t blen, uint8_t tg, uint8_t * mi, uint8_t * data, uint8_t * dlen ) {
	if ( blen > 240 ) {
		printf("\nError! Array data size is too big! HALT!\n\n");
		return 0;
	}
	uint8_t clength = 2 + blen;
	uint8_t cc[clength];
	cc[0] = PN532_COMMAND_INDATAEXCHANGE;
	uint8_t moreinfo = *mi;
	moreinfo <<= 6;
	cc[1] = tg|moreinfo;
	if ( blen >= 1 ) {
		for ( uint8_t i = 0; i<blen; i++ ) {
			cc[ 2 + i ] = buffer[i];
		}
	}
	pn532_spi_write_command(dev, writebuffer, cc, clength );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return PN532_INVALID_ACK;
	}
	/* Delay */
	delay(400);
	uint8_t rlen = pn532_read_pn(dev, readbuffer  );
	uint8_t status = readbuffer[2] & 0x3F;
	*mi = ( readbuffer[2] & 0x40 );
	*mi >>= 6;
	*dlen = rlen;
	if ( status == 0x00 ) {
		memcpy( data, readbuffer, rlen );
		return 1;
	}
	return 0;
}
/**************************************************************************/
/*
 *	@brief		The host controller uses this command to configure the PN532 as target.
 *
 *	@params		mode is a byte indicating which mode the PN532 should respect
 *
 *  @returns	1 on success,	0 or negative number on error
 */
/**************************************************************************/
uint8_t pn532_tg_init_as_target(pn532_t * dev, uint8_t mode ) {



	uint8_t dep = mode & 2;
	dep >>= 1;
	uint8_t picc = mode & 4;
	picc >>= 2;
	#ifdef P2PDEBUG
	uint8_t passive = mode & 1;
		if ( passive == 1 ) {
			printf("Passive Mode initialization.\n");
		}
		if ( dep == 1) {
			printf("DEP target Mode initialization.\n");
		}
		if ( picc == 1 ) {
			printf("ISO/IEC144443-4 PICC Mode initialization.\n");
		}
	#endif

	uint8_t cc[44] = {
	        PN532_COMMAND_TGINITASTARGET,
	        mode,				//0 for passive mode, 2 for DEP only, 4 for PICC only
			/* Mifare Params[] */
	        0x08, 0x00,         //SENS_RES
	        0x12, 0x34, 0x56,   //NFCID1t
	        0x40,               //SEL_RES 0x40 for DEP, 0x20 for ISO/IEC144443-4 PICC, 0x60 for both
			/* FeliCa Params[] */
	        0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, // POL_RES
	        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	        0xFF, 0xFF,
			/* NFCID3t: Change this to desired value */
	        0x01, 0xFE, 0x0F, 0xBB, 0xBA, 0xA6, 0xC9, 0x89, 0x00, 0x00,
			/* LEN Gt codes the number of general bytes (max. 47 bytes). This field is
			mandatory. When set to 0, there are no general bytes following. Last byte is LEN Tk */
	        0x06, 0x46,  0x66, 0x6D, 0x01, 0x01, 0x10, 0x00
	};
	uint8_t clength = sizeof(cc);
	pn532_spi_write_command(dev, writebuffer, cc, clength );

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	/* Delay */
	delay(60);
    uint16_t len = pn532_read_pn(dev, readbuffer );
	#ifdef PN532DEBUG
		pn532_print_hex( readbuffer, len );
	#endif
	uint8_t baudrate = ( readbuffer[2] & 0x70 ) ;
	baudrate >>= 4;
	picc = ( readbuffer[2] & 0x08 );
	picc >>= 3;
	dep = ( readbuffer[2] & 0x04 );
	dep >>= 2;

	#ifdef P2PDEBUG
	uint8_t type = ( readbuffer[2] & 0x03 );
		switch ( baudrate ) {
			case 0:
				printf( "Baudrate 106 kbps activated. Value: %d\n", baudrate );
				break;
			case 1:
				printf( "Baudrate 212 kbps activated. Value: %d\n", baudrate );
				break;
			case 2:
				printf( "Baudrate 424 kbps activated. Value: %d\n", baudrate );
				break;
		}
		if ( picc == 1) {
			printf("ISO/IEC144443-4 PICC mode activated\n");
		}
		if ( dep == 1) {
				printf("DEP mode activated\n");
		}
		if ( type == 0x01 ) {
			printf("Active mode activated!\n");
		} else if ( type == 0x00 ) {
			printf("Mifare type activated\n");
		} else {
			printf("FeliCa type activated!\n");
		}
	#endif

    if ( len > 0 ) {
        return 1;
    } else if ( PN532_TIMEOUT == len ) {
        return 0;
    } else {
        return -2;
    }

}
/**************************************************************************/
/**
 *	@brief		This command is used in combination with the TgInitAsTarget
 *				command to give the General Bytes.
 *				The PN532 uses them to build the ATR_RES sent to the initiator.
 *
 *	@params		gt is an array containing the general bytes to be used in the ATR_RES (max 47 bytes).
 *	@params		glen is the gt array length
 *
 *	@returns	1 on success, 0 or negative number on error
 */
/**************************************************************************/
uint8_t pn532_tg_set_general_bytes(pn532_t * dev, uint8_t * gt, uint8_t glen ) {

	if( glen > 47 ) {
		printf("Gt[] array size too big! Max 47 bytes! HALT");
		return 0;
	}
	uint8_t cc [ 1 + glen ];
	cc[0] = PN532_COMMAND_TGSETGENERALBYTES;
	if( glen >= 1 && glen != 0 ) {
		for ( int i = 0; i < glen; i++ ) {
			cc [ 1 + i ] = gt[i];
		}
	}
	pn532_spi_write_command(dev, writebuffer, cc, glen );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	delay(150);
	//Read response
	pn532_read_pn(dev, readbuffer );
	if ( readbuffer[2] != 0x00 ) {
			printf("Status TgSetGeneralBytes incorrect! Value: %X\n", readbuffer[2] );
			return PN532_INVALID_FRAME;
	}

	return 1;
}
/**************************************************************************/
/*
 *  @brief		This command is used in case of the PN532 configured as target
 *  			for Data Exchange Protocol (DEP) or for ISO/IEC14443-4 protocol
 *  			when PN532 is activated in ISO/IEC14443-4 PICC emulated.
 *  			Mi pointer corresponds to More Information byte.
 *
 *
 *	@param		mi		More Information bit
 *	@param		data	Pointer to the data array we read
 *	@param		dlen	Pointer to the data length we get
 *
 *  @returns	Byte length read on success, negative number on error
 */
/**************************************************************************/
uint16_t pn532_tg_get_data (pn532_t * dev, uint8_t * mi, uint8_t * data, uint8_t * dlen ) {

	delay(500);
	uint8_t cc[1];
	cc[0] = PN532_COMMAND_TGGETDATA;
	//send command
	pn532_spi_write_command(dev, writebuffer, cc, 1);

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	/* Delay */
	delay(1000);
	//Read response
	uint16_t length = pn532_read_pn( dev, readbuffer );
	//Check the response frame
	if( readbuffer[1] != 0x87 ) {
			DEBUG("Wrong Response frame!\n");
		return PN532_INVALID_FRAME;
	}

	uint8_t status = ( readbuffer[2] & 0x3F );
	if ( status != 0x00 ) {
				DEBUG("Status is not correct! Status value: %X\n", status);
			return PN532_INVALID_FRAME;
	}
	*mi = ( readbuffer[2] & 0x40 );
	*mi >>= 6;
	uint8_t nad = ( readbuffer[2] & 0x80 );
	nad >>= 7;
		DEBUG("Here it is the status value: %d, MI bit: %d, NAD bit: %d\n", status, *mi, nad );
	*dlen = length;
	memcpy( data, readbuffer, length );

	return length;
}

/**************************************************************************/
/*
 *  @brief		This command is used in case of the PN532 configured as target
 *  			for Data Exchange Protocol (DEP) or for ISO/IEC14443-4 protocol
 * 			 	when PN532 is activated in ISO/IEC14443-4 PICC emulated (see §4, p:21).
 *  		 	The overall amount of data to be sent can be transmitted in one
 *  		 	frame (262 bytes maximum).
 *
 *	@params		buffer	Byte length we want that PN532 as target send to the initiator
 *	@params		blen	Buffer length
 *
 *  @returns	Byte length on success, negative number on error
 */
/**************************************************************************/
uint8_t pn532_tg_set_data ( pn532_t * dev, uint8_t * buffer, uint8_t blen ) {

	uint8_t clength = 1 + blen;
	uint8_t cc[clength];
	cc[0] = PN532_COMMAND_TGSETDATA;
	if ( blen >= 1 ) {
		for ( uint8_t i = 0; i < blen; i++ ) {
			cc [ 1 + i] = buffer[i];
		}
	}
	uint8_t length = pn532_spi_write_command(dev, writebuffer, cc, clength );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return -1;
	}
	delay(100);
	pn532_read_pn(dev, readbuffer );
	uint8_t status = ( readbuffer[2] & 0x3F );
	if ( status != 0x00 ) {
				DEBUG("Status is not correct! Status value: %X\n", status);
			return PN532_INVALID_FRAME;
	}

	return length;
}

/**************************************************************************/
/*
 *  @brief		This command is used in case of the PN532 configured as target for Data
 *				Exchange Protocol (DEP) if the overall amount of data to be sent cannot be
 *				transmitted in one frame (more than 262 bytes).
 *
 *	@params		buffer	Byte length we want that PN532 as target send to the initiator
 *	@params		blen	Buffer length
 *
 *  @returns	Byte length on success, negative number on error
 */
/**************************************************************************/
bool pn532_tg_set_meta_data (pn532_t * dev, uint8_t * buffer, uint8_t blen ) {

	uint8_t clength = 1 + blen;
	uint8_t cc[clength];
	cc[0] = PN532_COMMAND_TGSETMETADATA;
	for ( uint8_t i = 0; i < blen; i++ ) {
		cc [ 1 + i] = buffer[i];
	}
	pn532_spi_write_command(dev, writebuffer, cc, clength );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	delay(200);
	pn532_read_pn(dev, readbuffer );
	if ( readbuffer[2] != 0x00 ) {
		printf("Status value wrong! %X\n", readbuffer[2]);
		return false;
	}

	return true;
}

/**************************************************************************/
/*
 *	@brief		This command is used when the PN532 is configured as target.
 *				This command is used to get a packet of data from an initiator
 *				and to send it back to the host controller.
 *
 *	@params		incommand is an array of raw data (up to 262 bytes) received by
 *				the PN532 Target from the Initiator.
 *	@params		length is the array's length
 *
 */
/**************************************************************************/
uint8_t pn532_tg_get_initiator_command (pn532_t * dev, uint8_t * incommand, uint8_t * length ) {

	uint8_t cc[1];
	cc[0] = PN532_COMMAND_TGGETINITIATORCOMMAND;
	pn532_spi_write_command(dev, writebuffer, cc, 1 );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	delay(50);
    uint16_t len = pn532_read_pn( dev,readbuffer );
	if ( readbuffer[2] != 0x00 ) {
			printf("Status TgSetGeneralBytes incorrect! Value: %X\n", readbuffer[2] );
			return PN532_INVALID_FRAME;
	}
	*length = len;
	memcpy( incommand, readbuffer, len );

	return 1;
}

/**************************************************************************/
/*
 *	@brief		This command is used to send a response packet of data to an initiator.
 *
 *	@params		tgresponse	pointer of the array of raw data to be sent by the PN532 to the Initiator
 *	@params		rlen
 *
 *  @returns	1 on success,	0 or negative number on error
 */
/**************************************************************************/
uint8_t pn532_tg_response_to_initiator (pn532_t * dev, uint8_t * tgresponse, uint8_t rlen ) {

	if ( rlen > 255 ) {
		printf("Array length too big! HALT!");
		return 0;
	}
	uint8_t csize = 1 + rlen;
	uint8_t cc[ csize ];
	cc[0] = PN532_COMMAND_TGRESPONSETOINITIATOR;
	if ( rlen >= 1 ) {
		for( int i = 0; i < rlen; i++ ) {
			cc[ 1 + i ] = tgresponse[i];
		}
	}
	pn532_spi_write_command(dev, writebuffer, cc, csize );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return 0;
	}
	delay(100);
	pn532_read_pn(dev, readbuffer );
	if ( readbuffer[2] != 0x00 ) {
			printf("Status TgSetGeneralBytes incorrect! Value: %X\n", readbuffer[2] );
		return PN532_INVALID_FRAME;
	}
	return 1;
}
/**************************************************************************/
/**
 * @brief		This command is used by the host controller to know what
 * 				the current state of the PN532 is.
 *
 */
/**************************************************************************/
void pn532_tg_get_target_status(pn532_t * dev) {
	uint8_t cc[1];
	cc[0] = PN532_COMMAND_TGGETTARGETSTATUS;
	pn532_spi_write_command(dev, writebuffer, cc, 1 );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return;
	}
    pn532_read_pn(dev, readbuffer );
    return;
}
/**************************************************************************/
/**
 *			 				Mifare Classic Part
 */
/**************************************************************************/
/**
 *	@brief		Waits for an ISO14443A target to enter the field
 *
 *	@param  	cardBaudRate  Baud rate of the card
 *	@param  	uid           Pointer to the array that will be populated
 *	                          with the card's UID (up to 7 bytes)
 *	@param		uidLength     Pointer to the variable that will hold the
 *	                          length of the card's UID.
 *
 *  @returns true on succes, false on error
 */
/**************************************************************************/
bool pn532_read_passive_target_id(pn532_t * dev, uint8_t cardbaudrate, uint8_t *uid, uint8_t * uidlength){

	uint8_t cc[3];
	cc[0] = PN532_COMMAND_INLISTPASSIVETARGET;
	cc[1] = 1;
	cc[2] = cardbaudrate;

	///uint8_t res =
			pn532_spi_write_command(dev, writebuffer, cc, 3 );

		DEBUG("Tranferred %d bytes.\n", res);

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return PN532_INVALID_ACK;
	}
	//Read response frame.
	pn532_read_pn(dev, readbuffer );
	// check some basic stuff

	/* ISO14443A card response should be in the following format:

      byte            Description
      -------------   ------------------------------------------
      b0              Tags Found
      b1              Tag Number (only one used in this example)
      b2..3           SENS_RES
      b4              SEL_RES
      b5              NFCID Length
      b6..NFCIDLen    NFCID
    */

	#ifdef MIFAREDEBUG
		printf("Found ");
		printf("%d", readbuffer[2]);
		printf(" tags.\n");
	#endif

	if( readbuffer[3] != 1 ){
		return false;
	}

	uint16_t sens_res = readbuffer[4];
	sens_res <<= 8;
	sens_res |= readbuffer[5];

	#ifdef MIFAREDEBUG
		printf("ATQA: 0x");
		printf("%X%X\n", readbuffer[4], readbuffer[5] );
		printf("SAK: 0x");
		printf("%X\n", readbuffer[6] );
	#endif

	/* Card appears to be Mifare Classic */
	*uidlength = readbuffer[7];

	#ifdef MIFAREDEBUG
		printf("UID:");
	#endif

	for(uint8_t i=0; i < readbuffer[7]; i++ ){
		uid[i] = readbuffer[ 8 + i ];
		#ifdef MIFAREDEBUG
			  printf(" 0x");
			  printf("%X ", uid[i] );
		#endif
	}
	#ifdef MIFAREDEBUG
		printf("\n");
	#endif

	return true;
}
/**************************************************************************/
/**
 *	@brief		Tries to authenticate a block of memory on a MIFARE card using the
 *				INDATAEXCHANGE command.  See section 7.3.8 of the PN532 User Manual
 *				for more information on sending MIFARE and other commands.
 *
 *	@param		uid           Pointer to a byte array containing the card UID
 *	@param		uidLen        The length (in bytes) of the card's UID (Should
 *                  	      be 4 for MIFARE Classic)
 *  @param		blockNumber   The block number to authenticate.  (0..63 for
 *              	          1KB cards, and 0..255 for 4KB cards).
 *  @param		keyNumber     Which key type to use during authentication
 *                  	      (0 = MIFARE_CMD_AUTH_A, 1 = MIFARE_CMD_AUTH_B)
 *  @param		keyData       Pointer to a byte array containing the 6 bytes
 *              	          key value
 *
 *	@returns	1 on success, 0 on error
 */
/**************************************************************************/
uint8_t pn532_mifareclassic_auth_block(pn532_t * dev, uint8_t *uid, uint8_t uidLen, uint32_t blockNumber, uint8_t keyNumber, uint8_t *keyData){

	uint8_t i;
	uint8_t keysize = 6;

	#ifdef MIFAREDEBUG
		printf("Trying to authenticate card ");
		for( uint8_t i = 0; i < uidLen; i++ ){
			printf("0x%X ", uid[i]);
		}
		printf("\n");
		printf("Using authentication KEY ");
		if( keyNumber == 0 ){
			printf("A");
		}
		else {
			printf("B");
		}
		printf( ": " );
		for( uint8_t i=0; i < keysize; i++){
			printf( "0x%X ", keyData[i] );
		}
		printf("\n");

	#endif

	// Prepare the authentication command //
	uint8_t clength = 4 + keysize + uidLen;
	uint8_t cc[clength];
	//Data Exchange header
	cc[0] = PN532_COMMAND_INDATAEXCHANGE;
	//Max Card numbers
	cc[1] = 1;
	cc[2] = (keyNumber) ? MIFARE_CMD_AUTH_B : MIFARE_CMD_AUTH_A;	//If keyNumber==1 cc[2]=0x61 otherwise cc[2]=0x60
	/* Block Number (1K = 0..63, 4K = 0..255 */
	cc[3] = blockNumber;
	//Write the authentication key inside the array command
	for( uint8_t i=0; i < keysize; i++){
		cc[ 4 + i] = keyData[i];
	}
	// 4/7 byte card ID
	for (i = 0; i < uidLen; i++){
		cc[ 10 + i ] = uid[i];
	}


	pn532_spi_write_command(dev, writebuffer, cc, clength);
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {

				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");

		return PN532_INVALID_ACK;
	}
	//After sending the command authentication, we should wait for a default value of 51.2 ms
	delay(100);
	//Read response frame.
	uint8_t len = pn532_read_pn(dev, readbuffer );

	// check if the response is valid and we are authenticated???
	// for an auth success it should be bytes 5-7: 0xD5 0x41 0x00
	// Mifare auth error is technically byte 7: 0x14 but anything other and 0x00 is not good
	if ( readbuffer[2] != 0x00){
		#ifdef MIFAREDEBUG
	    	printf("Authentication failed: ");
	    	pn532_print_hex( readbuffer, len );
	    #endif
	    //TODO write a matrix keyData and a for cycle to change the key and try iteratively to authenticate
	    return 0;
	}

	return 1;
}

/**************************************************************************/
/**
 *	@brief		Tries to read an entire 16-bytes data block at the specified block
 *				address.
 *
 *	@param		blockNumber		The block number to authenticate.  (0..63 for
 *								1KB cards, and 0..255 for 4KB cards).
 *	@param		data			Pointer to the byte array that will hold the
 *								retrieved data (if any)
 *
 *	@returns	1 on success, 0 on error
 */
/**************************************************************************/
uint8_t pn532_mifareclassic_read_data_block(pn532_t * dev, uint8_t blockNumber, uint8_t * data){

	#ifdef MIFAREDEBUG
		printf("Trying to read 16 bytes from block ");
		printf("%d", blockNumber);
	#endif

	/* Prepare the write command */
	uint8_t cc[4];
	cc[0] = PN532_COMMAND_INDATAEXCHANGE;
	cc[1] = 1;                      /* Card number */
	cc[2] = MIFARE_CMD_READ;        /* Mifare Read command = 0x30 */
	cc[3] = blockNumber;            /* Block Number (0..63 for 1K, 0..255 for 4K) */

	pn532_spi_write_command(dev, writebuffer, cc, 4);

	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return PN532_INVALID_ACK;
	}
	//Read response frame.
	uint8_t len = pn532_read_pn(dev, readbuffer );

	/* If byte 3 isn't 0x00 we probably have an error */
	if (readbuffer[2] != 0x00){
		DEBUG("Unexpected response.\n");
	    pn532_print_hex( readbuffer, len );
	    return 0;
	}

	/* Copy the 16 data bytes to the output buffer        */
	/* Block content starts at byte 7 of a valid response */
	memcpy (data, readbuffer + 7, 16);

	return 1;
}

/**************************************************************************/
/**
 *	@brief		Tries to write an entire 16-byte data block at the specified block
 *				address.
 *
 *	@param		blockNumber		The block number to authenticate.  (0..63 for
 *								1KB cards, and 0..255 for 4KB cards).
 *  @param		data			The byte array that contains the data to write.
 *
 *  @returns 	1 on success, 0 on error
 */
/**************************************************************************/
uint8_t pn532_mifareclassic_write_data_block(pn532_t * dev, uint8_t blockNumber, uint8_t * data ){
	#ifdef MIFAREDEBUG
		printf( "Trying to write 16 bytes to block %d\n", blockNumber );
	#endif

	//Prepare the array command
	uint8_t datasize = sizeof(data);
	if( datasize > 16 ) {
		printf("\nArray data to write in is too high! HALT!\n\n");
		return 0;
	}
	uint8_t clength = 4 + sizeof(data);
	uint8_t cc[clength];
	cc[0] = PN532_COMMAND_INDATAEXCHANGE;
	cc[1] = 1;
	//Mifare Write command = 0xA0
	cc[2] = MIFARE_CMD_WRITE;
	cc[3] = blockNumber;
	for( uint8_t i = 0; i < clength; i++){
		cc[ 4 + i ] = data[i];
	}
	pn532_spi_write_command(dev, writebuffer, cc, clength );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return PN532_INVALID_ACK;
	}
	delay(10);
	pn532_read_pn( dev, readbuffer );

	return 1;
}

/**************************************************************************/
/**
 *	@brief		Formats a Mifare Classic card to store NDEF Records
 *
 *	@returns	1 on success, 0 for an error
 */
/**************************************************************************/
uint8_t pn532_mifareclassic_format_ndef(pn532_t * dev){

    uint8_t sectorbuffer1[16] = {0x14, 0x01, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer2[16] = {0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1, 0x03, 0xE1};
    uint8_t sectorbuffer3[16] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0x78, 0x77, 0x88, 0xC1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

    // Note 0xA0 0xA1 0xA2 0xA3 0xA4 0xA5 must be used for key A
    // for the MAD sector in NDEF records (sector 0)
    // Write block 1 and 2 to the card
    if ( !(pn532_mifareclassic_write_data_block(dev, 1, sectorbuffer1 )) ) {
        return 0;
    }
    if ( !( pn532_mifareclassic_write_data_block(dev, 2, sectorbuffer2 )) ) {
        return 0;
    }
    // Write key A and access rights card
    if (! ( pn532_mifareclassic_write_data_block(dev, 3, sectorbuffer3 )) ) {
        return 0;
    }
	return 1;
}

/**************************************************************************/
/**
 *	@brief		Format a clean Mifare Classic 1K card as an NFC Forum tag
 *				(to store NDEF messages that can be read by any NFC enabled
 *				Android phone, etc.).
 *				We can encode many different kinds of pointers to the card,
 *				from a URL, to an Email address, to a phone number, and many more
 *				check the library header .h file to see the large # of supported
 *				prefixes! It depends from ndefprefix we choose.
 *				See the header for the corrispondent pointer.
 *
 *				Before to use this function you must initialize the board, get the
 *				firmware, SAMConfig and authenticate.
 *
 *	@param		url				byte url
 *	@param		ndefprefix		byte Prefixes for NDEF Records (to identify record type)
 *	@param		blockNumber		block number where we want to authenticate and write in it
 *
 *	@returns		1 on success, 0 on error
 */
/**************************************************************************/

uint8_t pn532_format_to_ndef(pn532_t * dev, uint8_t * url, uint8_t ndefprefix, uint8_t blockNumber) {

	printf("\nnPLEASE NOTE: Formatting your card for NDEF records will change the\n");
	printf("authentication keys.  To reformat your NDEF tag as a clean Mifare\n");
	printf("Classic tag, use the pn532_mifareclassic_ndef_to_classic function!\n\n");
	printf("Place your Mifare Classic card on the reader to format with NDEF\n");

	// Start with block 4 (the first block of sector 1) since sector 0
	// contains the manufacturer data and it's probably better just
	// to leave it alone unless you know what you're doing
	if( blockNumber==1 || blockNumber==0 ){
		printf("Block number inserted wrong. Could be dangerous. Block: %d", blockNumber);
		return 0;
	}
	while(1){
		uint8_t success;
		uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
		uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
		success = pn532_read_passive_target_id(dev,PN532_MIFARE_ISO14443A, uid, &uidLength);
		//be careful to choose the block number. Starting from 2 is the best solution if you don't know the use of it

		if (success) {
		   			// Display some basic information about the card
		       		printf("Found an ISO14443A card");
		       		printf("  UID Length: ");
		       		printf("%d",uidLength);
		       		printf(" bytes\n");
		       		printf("UID Value: ");
		       		pn532_print_hex( uid, uidLength );
		       		printf("\n");

		       		if (uidLength == 4){
		       			// We probably have a Mifare Classic card ...
		        		printf("Seems to be a Mifare Classic card (4 byte UID)\n");

		        		// Now we need to try to authenticate it for read/write access
		        		// Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
		        		printf("Trying to authenticate block 4 with default KEYA value\n");
		        		uint8_t keya[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	//	        		uint8_t keya[6] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7};
	//	        		uint8_t keya[6] = {0xA0, 0xA1, 0xA2, 0xA3, 0xA4, 0xA5};
	//	        		uint8_t keya[6] = {0xB0, 0xB1, 0xB2, 0xB3, 0xB4, 0xB5};
	//	        		uint8_t keya[6] = {0x4D, 0x3A, 0x99, 0xC3, 0x51, 0xDD};
	//	        		uint8_t keya[6] = {0x1A, 0x98, 0x2C, 0x7E, 0x45, 0x9A};
	//	        		uint8_t keya[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
	//	        		uint8_t keya[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	//	        		uint8_t keya[6] = {0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56};


		        		success = pn532_mifareclassic_auth_block(dev, uid, uidLength, 0, 0, keya);
		        	    if (!success)
		        	    {
		        	      printf("Unable to authenticate block 0 to enable card formatting!\n");
		        	      return 0;
		        	    }
		        	    success = pn532_mifareclassic_format_ndef(dev);
		        	    if (!success)
		        	    {
		        	      printf("Unable to format the card for NDEF\n");
		        	      return 0;
		        	    }
		        	    printf("Card has been formatted for NDEF data using MAD1\n");
		        	    // Try to authenticate block 4 (first block of sector 1) using our key
		        	    success = pn532_mifareclassic_auth_block (dev, uid, uidLength, blockNumber, 0, keya);
		        	    if (!success)
		        	    {
		        	      printf("Authentication failed.");
		        	      return 0;
		        	    }

		        	    printf("Try to write to sector 1 as an NDEF Message");
		        	    if ( strlen((const char*)url) > 38 ) {
		        	    	// The length is also checked in the WriteNDEFURI function, but lets
		        	    	// warn users here just in case they change the value and it's bigger
		        	    	// than it should be
		        	    	printf("URI is too long ... must be less than 38 characters long");
		        	    	return 0;
		        	    }
		        	    // URI is within size limits ... write it to the card and report success/failure
		        	    success = pn532_mifareclassic_write_ndef_uri(dev, 1, ndefprefix, (const char *)url );
		        	    if (success) {
		        	    	printf("NDEF URI Record written to sector 1\n");
		        	    }
		        	    else {
		        	    	printf("NDEF Record creation failed!\n");
		        	    }

		       		}
		}
	}

	return 1;
}

/**************************************************************************/
/**
 *	@brief		Writes an NDEF URI Record to the specified sector (1..15)
 *				Note that this function assumes that the Mifare Classic card is
 *				already formatted to work as an "NFC Forum Tag" and uses a MAD1
 *				file system.  You can use the NXP TagWriter app on Android to
 *				properly format cards for this.
 *
 *	@param  	sectorNumber	The sector that the URI record should be written
 *								to (can be 1..15 for a 1K card)
 *	@param		uriIdentifier	The uri identifier code (0 = none, 0x01 = "http://www.", etc.)
 *	@param		url				The uri text to write (max 38 characters).
 *
 *	@returns	1 on success, 0 on error
 */
/**************************************************************************/
uint8_t pn532_mifareclassic_write_ndef_uri (pn532_t * dev, uint8_t sectorNumber, uint8_t uriIdentifier, const char * url) {

	// Figure out how long the string is
	uint8_t len = strlen(url);

	// Make sure we're within a 1K limit for the sector number
	if ((sectorNumber < 1) || (sectorNumber > 15)) {
		return 0;
	}
	// Make sure the URI payload is between 1 and 38 chars
	if ((len < 1) || (len > 38)) {
		return 0;
	}

    // Note 0xD3 0xF7 0xD3 0xF7 0xD3 0xF7 must be used for key A
    // in NDEF records

	// Setup the sector buffer (w/pre-formatted TLV wrapper and NDEF message)
	uint8_t sectorbuffer1[16] = {0x00, 0x00, 0x03, len+5, 0xD1, 0x01, len+1, 0x55, uriIdentifier, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t sectorbuffer2[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t sectorbuffer3[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	uint8_t sectorbuffer4[16] = {0xD3, 0xF7, 0xD3, 0xF7, 0xD3, 0xF7, 0x7F, 0x07, 0x88, 0x40, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	if (len <= 6) {
		// Unlikely we'll get a url this short, but why not ...
	    memcpy (sectorbuffer1+9, url, len);
	    sectorbuffer1[len+9] = 0xFE;
	}
	else if (len == 7) {
	    // 0xFE needs to be wrapped around to next block
	    memcpy (sectorbuffer1+9, url, len);
	    sectorbuffer2[0] = 0xFE;
	}
	else if ((len > 7) || (len <= 22)) {
	    // Url fits in two blocks
	    memcpy (sectorbuffer1+9, url, 7);
	    memcpy (sectorbuffer2, url+7, len-7);
	    sectorbuffer2[len-7] = 0xFE;
	}
	else if (len == 23) {
	    // 0xFE needs to be wrapped around to final block
	    memcpy (sectorbuffer1+9, url, 7);
	    memcpy (sectorbuffer2, url+7, len-7);
	    sectorbuffer3[0] = 0xFE;
	}
	else {
	    // Url fits in three blocks
	    memcpy (sectorbuffer1+9, url, 7);
	    memcpy (sectorbuffer2, url+7, 16);
	    memcpy (sectorbuffer3, url+23, len-24);
	    sectorbuffer3[len-22] = 0xFE;
	}

	// Now write all three blocks back to the card
	if ( 0x01 != ( pn532_mifareclassic_write_data_block (dev, sectorNumber*4, sectorbuffer1))) {
	      return 0;
	}
	if ( 0x01 != ( pn532_mifareclassic_write_data_block (dev, (sectorNumber*4)+1, sectorbuffer2))) {
	      return 0;
	}
	if ( 0x01 != ( pn532_mifareclassic_write_data_block (dev, (sectorNumber*4)+2, sectorbuffer3))) {
	      return 0;
	}
	if ( 0x01 != ( pn532_mifareclassic_write_data_block (dev, (sectorNumber*4)+3, sectorbuffer4))) {
	      return 0;
	}

	return 1;
}

/**************************************************************************/
/**
 *	@brief		Tries to read an entire 4-byte page at the specified address.
 *
 *	@param		page		The page number (0..63 in most cases)
 *	@param		buffer      Pointer to the byte array that will hold the
 *							retrieved data (if any).
 *
 *	@returns	1 on success, 0 on error
 */
/**************************************************************************/
uint8_t pn532_mifareultralight_read_page (pn532_t * dev, uint8_t page, uint8_t *buffer){

	if( page >= 64 ){
		#ifdef MIFAREDEBUG
			printf("Page value out of range!\n");
		#endif
		return 0;
	}

	#ifdef MIFAREDEBUG
	  printf("Reading page ");
	  printf("%d\n", page);
	#endif
	/* Prepare the command */
	uint8_t cc[4];
	cc[0] = PN532_COMMAND_INDATAEXCHANGE;
	cc[1] = 1;                   /* Card number */
	cc[2] = MIFARE_CMD_READ;     /* Mifare Read command = 0x30 */
	cc[3] = page;                /* Page Number (0..63 in most cases) */

	pn532_spi_write_command(dev,writebuffer, cc, 4);
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {
				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");
		return PN532_INVALID_ACK;
	}
	//Read response frame.
	uint8_t len = pn532_read_pn( dev,readbuffer  );

	#ifdef MIFAREDEBUG
		printf("Received: ");
		pn532_print_hex( readbuffer, len );
	#endif

    /* If byte 3 isn't 0x00 we probably have an error */
    if ( readbuffer[2] == 0x00 ) {
        /* Copy the 4 data bytes to the output buffer         */
        /* Block content starts at byte 9 of a valid response */
        /* Note that the command actually reads 16 bytes or 4  */
        /* pages at a time ... we simply discard the last 12  */
        /* bytes                                              */
        memcpy ( buffer, readbuffer + 1, 4 );
    } else {
    	#ifdef MIFAREDEBUG
    		printf("Unexpected response reading block: ");
    		pn532_print_hex(readbuffer, len );
    	#endif
    	return 0;
    }

	return 1;
}

/**************************************************************************/
/*!
    Tries to write an entire 4-bytes data buffer at the specified page
    address.

    @param  page     	The page number to write into.  (0..63).
    @param  buffer   	The byte array that contains the data to write.
    @param	buffersize	The size of the buffer array

    @returns 1 if everything executed properly, 0 for an error
*/
/**************************************************************************/
uint8_t pn532_mifareultralight_write_page (pn532_t * dev, uint8_t page, uint8_t *buffer, uint8_t buffersize) {
	/* Prepare the first command */
	uint8_t cc[ 4 + buffersize ];
	cc[0] = PN532_COMMAND_INDATAEXCHANGE;
	cc[1] = 1;                   /* Card number */
	cc[2] = MIFARE_CMD_WRITE_ULTRALIGHT; /* Mifare UL Write cmd = 0xA2 */
	cc[3] = page;                /* Page Number (0..63 in most cases) */
	for( uint8_t i = 0; i < buffersize; i++ ){
		cc[ 4 + i ] = buffer[i];
	}
	uint8_t clength = 4 + buffersize;
	pn532_spi_write_command(dev, writebuffer, cc, clength );
	//Read ACK frame sent from PN532 and check it
	if( 1 != pn532_check_ack(dev) ) {

				DEBUG("\n\nACK frame received from PN532 wrong!\n\n");

		return PN532_INVALID_ACK;
	}

	pn532_read_pn(dev, readbuffer );

	return 1;
}
