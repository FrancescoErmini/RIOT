

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
#include "board.h"
#include "kernel.h"
#include "xbee.h"
#include "msg.h"
#include "net/ng_netbase.h"
#include "net/ng_nomac.h"
#include "net/ng_pktdump.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "pn532.h"
#include "keyexc.h"
#include "thread.h"
#include "posix_io.h"
#include "shell.h"
#include "shell_commands.h"
#include "board_uart0.h"
#include "hwtimer.h"

#ifndef _SENSOR
#define _SENSOR (0)
#endif
#ifndef _GATEWAY
#define _GATEWAY (0)
#endif
//#define KEYEXC 1
#define OPT_ENCRYPTION 1


/* make sure an UART to device is defined in the Makefile */
#ifndef XBEE_UART
#error "XBEE_UART not defined"
#endif

/**
 * @brief   Xbee parameters
 */
#define XBEE_BAUDRATE           (9600U)
#define XBEE_SLEEP 				(GPIO_18)
#define XBEE_RST				(GPIO_19)


/**
 * @brief   Initialize  device descriptor.
 */
static xbee_t dev;
static keyexc_t key;
static pn532_t pn532;


/**
 * @brief   Buffer size used by the shell
 */
#define SHELL_BUFSIZE           (64U)

/**
 * @brief   Stack for the nomac thread
 */
static char nomac_stack[KERNEL_CONF_STACKSIZE_DEFAULT];



static uint8_t key_buf[] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	                                  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,};

/*
void keyexc_set_xbee(xbee_t * dev, uint8_t * channel, uint8_t * panid, uint8_t * short_address, uint8_t * long_address)
{

	dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_CHANNEL,channel,1);
	dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_NID,panid,2);
	dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ADDRESS,short_address,2);
	dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ADDRESS_LONG,long_address,8);
	ng_netconf_enable_t encrypt_status = NETCONF_ENABLE;
	dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ENCRYPTION, &encrypt_status,1);
	dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ENCRYPTION_KEY,key_buf,sizeof(key_buf));

}
void keyexc_get_xbee(xbee_t * dev, uint8_t * channel, uint8_t * panid, uint8_t * short_address, uint8_t * long_address)
{
dev->driver->get((ng_netdev_t *)dev,NETCONF_OPT_CHANNEL,channel,1);
dev->driver->get((ng_netdev_t *)dev,NETCONF_OPT_NID,panid,2);
dev->driver->get((ng_netdev_t *)dev,NETCONF_OPT_ADDRESS,short_address,2);
dev->driver->get((ng_netdev_t *)dev,NETCONF_OPT_ADDRESS_LONG,long_address,8);

printf("\nCHANNEL: %s",channel);
printf("\nPAN ID: %s",panid);
printf("\nSHORT ADDRESS: %s", short_address);
printf("\nLONG ADDRESS: %s", long_address);

}
void keyexc_set_key(xbee_t * dev)
{  printf("\n disabilito l'interrupt");
//	gpio_irq_disable(GPIO_15);
//provaprova =1;


//xbee_init(dev, XBEE_UART, XBEE_BAUDRATE, GPIO_NUMOF, GPIO_NUMOF);
printf("SCRIVO I COMANDI EE E KY NELL XBEE");
ng_netconf_enable_t encrypt_status = NETCONF_ENABLE;
dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ENCRYPTION, &encrypt_status,1);
dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ENCRYPTION_KEY,key_buf,sizeof(key_buf));

}
void keyexc_off_key(xbee_t * dev)
{
	ng_netconf_enable_t encrypt_status = NETCONF_DISABLE;
	      dev->driver->set((ng_netdev_t *)dev,NETCONF_OPT_ENCRYPTION, &encrypt_status,1);
}
void keyexc_cb(void *arg){

        printf("\nSTARTING P2P KEY EXCHANGE.... ");
        keyexc(&key);
        printf("\n NOW SET THE KEY INTO THE XBEE..");
    	printf("SCRIVO I COMANDI EE E KY NELL XBEE");

    	keyexc_set_key(&dev);

        printf("\nFinish.");
    }
*/

/**
 * @brief   Maybe you are a golfer?!
 */

/**
 * @brief   Buffer size used by the shell
 */

/**
 * @brief   Maybe you are a golfer?!
 */




int main(void)
{


printf("\n\n Count down...");
	for(int i=0;i<1000;i++)
	{ printf("wait %i",i);
hwtimer_wait(1000000);
	}printf("OK!");



	kernel_pid_t iface;
	    int res;
	    shell_t shell;
	    ng_netreg_entry_t dump;

	    puts("Xbee S1 device driver test");
	    printf("Initializing the Xbee S1 device UART_%i... \n", XBEE_UART);

	    /* initialize network module(s) */
	    ng_netif_init();

	    /* initialize and register pktdump */
	    dump.pid = ng_pktdump_getpid();
	    if (dump.pid <= KERNEL_PID_UNDEF) {
	        puts("Error starting pktdump thread");
	        return -1;
	    }
	    dump.demux_ctx = NG_NETREG_DEMUX_CTX_ALL;
	    ng_netreg_register(NG_NETTYPE_UNDEF, &dump);


	    /* setup Xbee device */
	    puts("wait xbee to wake up");
	    hwtimer_wait(1000000);
	    res = xbee_init(&dev, XBEE_UART, XBEE_BAUDRATE, GPIO_NUMOF, GPIO_NUMOF);
	    if (res < 0) {
	        puts("Error initializing xbee device driver");
	        return -1;
	    }
	    printf("\nXbee init stoppe.");
	    /* start MAC layer */
	    iface = ng_nomac_init(nomac_stack, sizeof(nomac_stack), PRIORITY_MAIN - 3,
	                          "xbee_l2", (ng_netdev_t *)&dev);
	    if (iface <= KERNEL_PID_UNDEF) {
	        puts("Error initializing MAC layer");
	        return -1;
	    }

	    uint8_t addr[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	     ng_pktsnip_t *pkt;
	     ng_netif_hdr_t *nethdr;
	     char data[]="hello";
	    size_t addr_len = 8;

	    /* put packet together */
	     pkt = ng_pktbuf_add(NULL,(char *)data, strlen(data), NG_NETTYPE_UNDEF);
	     pkt = ng_pktbuf_add(pkt, NULL, sizeof(ng_netif_hdr_t) + addr_len,
	                         NG_NETTYPE_NETIF);
	     nethdr = (ng_netif_hdr_t *)pkt->data;
	     ng_netif_hdr_init(nethdr, 0, addr_len);

	     ng_netif_hdr_set_dst_addr(nethdr, addr, addr_len);
	     /* and send it */
	    for ( int i= 0; i<300; i++){
	    	printf("\nmess[%i]",i);
	     ng_netapi_send(iface, pkt);
	     hwtimer_wait(1000*5000);
	    }

#if _SENSOR


    	    static keyexc_802154_t keyexc_802154;
    		uint8_t id = 0x02;
    		//TODO: casting sulla struttura dati, rpl coap
	        keyexc_init(&key, &pn532,SENSOR, id, &keyexc_802154);
	        keyexc(&key);
	        keyexc_set_key(&dev);



#endif

#if _GATEWAY

	       // static keyexc_802154_t keyexc_802154;
	        	uint8_t tmp_buf[30];
	          		uint8_t id = 0x00;
	          		//TODO: casting sulla struttura dati, rpl coap
	      	       keyexc_init(&key, &pn532,GATEWAY, id,tmp_buf);
	      	       keyexc(&key);
	      	      // keyexc_set_key(&dev);

#endif


#if _SENSOR

    for(int i=0;i<30;i++)
    {
    	printf("\npkt_encrypt[%i]",i);
     ng_netapi_send(iface, pkt);
     hwtimer_wait(1000*2000);
    }
    return 0;
#endif
}
