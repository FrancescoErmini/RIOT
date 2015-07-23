

/**
 * @ingroup     tests

 * @{
 *
 * @file
 * @brief       Test application for Xbee S1 network device driver
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @ modified my Francesco Ermini for thesis porpose
 * @}
 */
#include <stdio.h>
#include "net/ng_netbase.h"
#include "net/ng_pktdump.h"
//#include "shell.h"
//#include "shell_commands.h"

#include "periph/uart.h"
#include "periph/gpio.h"

#include "pn532.h"
#include "keyexc.h"
#include "hwtimer.h"
#include "vtimer.h"
#ifndef _SENSOR
#define _SENSOR (1)
#endif
#ifndef _GATEWAY
#define _GATEWAY (0)
#endif
#define KEYEXC 1

static keyexc_t key;
static pn532_t pn532;

/**
 * @brief   Buffer size used by the shell
 */
#define SHELL_BUFSIZE           (64U)

int main(void)
{
	  // shell_t shell;
	 printf("\nKEYEXC DEMO\n");

	   ng_netreg_entry_t dump;
	     /* initialize and register pktdump */
	     dump.pid = ng_pktdump_getpid();
	     if (dump.pid <= KERNEL_PID_UNDEF) {
	         puts("Error starting pktdump thread");
	         return -1;
	     }
	     dump.demux_ctx = NG_NETREG_DEMUX_CTX_ALL;
	     ng_netreg_register(NG_NETTYPE_UNDEF, &dump);




	     kernel_pid_t interfacce_network[NG_NETIF_NUMOF];
	     kernel_pid_t interfaccia_xbee = 0;
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
	            uint16_t src_len = 8;

	            ng_netapi_set(interfaccia_xbee, NETCONF_OPT_SRC_LEN, 0, &src_len, sizeof(src_len));




       puts("\n\nSTART KEY EXCHANGE:\n");

       static keyexc_802154_t data802154;
       static keyexc_RPL_t dataRPL;
       static keyexc_COAP_t dataCOAP;
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

             //  printf("\n\nSET RECEIVED KEY AND PARAMETRS TO THE 802.15.4 MODULE");
             // keyexc_set_xbee(&dev, &key);


   #endif

      //  shell_init(&shell, NULL, SHELL_BUFSIZE, getchar, putchar);
      //  shell_run(&shell);

}
