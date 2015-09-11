

/**
 * @{
 *
 * @file       Test for Keyexc
 * @brief      Key exchange demo test
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author 		Francesco Ermini <francescoermini8@gmail.com>
 * @}
 */
#include <stdio.h>
#include "net/ng_netbase.h"
#include "net/ng_pktdump.h"
/*
 * @brief Avoid the use of shell increase stability in flash process.
 * Therefore uart0 module has been add in Makefile.
 */
//#include "shell.h"
//#include "shell_commands.h"

#include "periph/uart.h"
#include "periph/gpio.h"

#include "pn532.h"
#include "keyexc.h"
#include "hwtimer.h"
#include "vtimer.h"
/**
 * To flash inside the Sensor puts Sensor 1, Gateway 0.
 * To flash inside the Gateway puts Sensor 0, Gateway 1.
 */
#ifndef _SENSOR
#define _SENSOR (1)
#endif
#ifndef _GATEWAY
#define _GATEWAY (0)
#endif
/**
 * @brief PN532, Device descriptor for NFC
 * @brief Keyexc, virtual descriptor.
 * Define parameters and data exchanged by the API.
 */

static keyexc_t key;
static pn532_t pn532;


int main(void)
{

	   // shell_t shell;
	   printf("\nKEYEXC DEMO\n");
	   /**
	    * @brief start Xbee code part.
	    */
	   ng_netreg_entry_t dump;
	     /* initialize and register pktdump */
	     dump.pid = ng_pktdump_getpid();
	     if (dump.pid <= KERNEL_PID_UNDEF) {
	         puts("Error starting pktdump thread");
	         return -1;
	     }
	     dump.demux_ctx = NG_NETREG_DEMUX_CTX_ALL;
	     ng_netreg_register(NG_NETTYPE_UNDEF, &dump);



	     /**
	      * The Xbee init function has been moved to:
	      * /sys/auto_init/netif/auto_init_xbee.c.
	      *
	      * To dialog with Xbee we need to retrieve his kernel_pid.
	      * - Interfaccia_xbee is the kernel pid of Xbee.
	      * - Then you must use ng_netapi_sent, ng_netapi_set, ng_netadpi_get..
	      * NB: This works if only one network interface is running.
	      * NB: addr is the 64 bit address of Xbee. I used:
	      * Sensor source: uint8_t addr[8]={0x00, 0x13, 0xa2, 0x00, 0x40, 0x87, 0x0e, 0x5f};
	      * Gateway source: uint8_t addr[8]={0x00, 0x13, 0xa2, 0x00, 0x40, 0x87, 0x0d, 0x4e};
	      */
	     kernel_pid_t interfacce_network[NG_NETIF_NUMOF];
	     kernel_pid_t interfaccia_xbee = 0; // Needed to shut down the compiler waring.
	     size_t numero_interfacce = ng_netif_get(interfacce_network);

	         for (size_t i = 0; i < numero_interfacce && i < NG_NETIF_NUMOF; i++) {
	            interfaccia_xbee = interfacce_network[i];
	         }

	         char data[] = " senza cifratura";
	            size_t addr_len = 8;
	            uint8_t addr[8]={0x00, 0x13, 0xa2, 0x00, 0x40, 0xc4, 0x36, 0x57};
	            ng_pktsnip_t *pkt = ng_pktbuf_add(NULL, data, sizeof(data), NG_NETTYPE_UNDEF);
	            ng_pktsnip_t *hdr = ng_netif_hdr_build(NULL, 0, addr, addr_len);
	            LL_PREPEND(pkt, hdr);

	     puts(" Sending some test packets...");
	            for(int i=0;i<10;i++){
	            ng_netapi_send(interfaccia_xbee, pkt);
	            hwtimer_wait(1000*5000);
	            }


       puts("\n\nSTART KEY EXCHANGE:\n");
/**
 * Define data protocol containers.
 */
       static keyexc_802154_t data802154;
       static keyexc_RPL_t dataRPL;
       static keyexc_COAP_t dataCOAP;

 /*
  * keyexc init initialize different parameters in Gateway and Sensor.
  * The only value given here is the sensor id. All others values are obtained through UART and NFC.
  *
  *Keyexc manage the UART communication between  the gateway and the Linux databse
  *Keyexc  manage the NFC communication between  the gateway and the sensor.
  */

       uint8_t id;
   		#if _GATEWAY
   		  id = 0x00;
   		keyexc_init(&key, &pn532, GATEWAY, id, &data802154, &dataRPL, &dataCOAP);
   		#endif
   		#if _SENSOR
   		   id = 0x02;
   		keyexc_init(&key, &pn532, SENSOR, id, &data802154, &dataRPL, &dataCOAP);
   		#endif

        keyexc(&key);

   #if _SENSOR2

        uint16_t src_len = 8;
        keyexc_set(interfaccia_xbee);
      	



   #endif

      //  shell_init(&shell, NULL, SHELL_BUFSIZE, getchar, putchar);
      //  shell_run(&shell);

}
