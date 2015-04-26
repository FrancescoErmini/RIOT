

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
#include "shell.h"
#include "shell_commands.h"
#include "xbee.h"
#include "net/ng_netbase.h"
#include "net/ng_nomac.h"
#include "net/ng_pktdump.h"
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include "pn532.h"
#include "keyexc.h"
#include "hwtimer.h"
#ifndef _SENSOR
#define _SENSOR (1)
#endif
#ifndef _GATEWAY
#define _GATEWAY (0)
#endif
#define KEYEXC 1
#define OPT_ENCRYPTION 1


/* make sure an UART to device is defined in the Makefile */
#ifndef XBEE_UART
#error "XBEE_UART not defined"
#endif

/**
 * @brief   This is the default baudrate the Xbee modules are programmed to
 *          when you buy them
 */
#define XBEE_BAUDRATE           (9600U)

/**
 * @brief   Buffer size used by the shell
 */
#define SHELL_BUFSIZE           (64U)

/**
 * @brief   The Xbee device descriptor
 */
static xbee_t dev;

#ifdef KEYEXC
static keyexc_t key;
static pn532_t pn532;
#endif

/**
 * @brief   Stack for the nomac thread
 */
static char nomac_stack[KERNEL_CONF_STACKSIZE_DEFAULT];

/**
 * @brief   Read chars from STDIO
 */
int shell_read(void)
{
    return (int)getchar();
}

/**
 * @brief   Write chars to STDIO
 */
void shell_put(int c)
{
    putchar((char)c);
}
#ifdef KEYEXC
void keyexc_set_key(void)
{
ng_netconf_enable_t encrypt = NETCONF_ENABLE;      /* warning: use NETCONF_DISABLE before
                                                    * set OPT_ENCRYPTION to undefined */
static  uint8_t key_buf[XBEE_ENCRYPTION_KEY_LEN]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
             0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,}; // change those value to match your key
dev.driver->set((ng_netdev_t *)&dev,NETCONF_OPT_ENCRYPTION, &encrypt,1);
dev.driver->set((ng_netdev_t *)&dev,NETCONF_OPT_ENCRYPTION_KEY,key_buf,sizeof(key_buf));
}
void keyexc_cb(void *arg){
        printf("THE NODE IS ASKING A NEW KEY ");
        keyexc(&key);
        printf("\n NOW SET THE KEY INTO THE XBEE");
        keyexc_set_key();
    }

#endif
/**
 * @brief   Maybe you are a golfer?!
 */
int main(void)
{
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
    res = xbee_init(&dev, XBEE_UART, XBEE_BAUDRATE, GPIO_NUMOF, GPIO_NUMOF);
    if (res < 0) {
        puts("Error initializing xbee device driver");
        return -1;
    }
    /* start MAC layer */
    iface = ng_nomac_init(nomac_stack, sizeof(nomac_stack), PRIORITY_MAIN - 3,
                          "xbee_l2", (ng_netdev_t *)&dev);
    if (iface <= KERNEL_PID_UNDEF) {
        puts("Error initializing MAC layer");
        return -1;
    }
    /* optionally en/disable and set Xbee encryption key */






#ifdef KEYEXC

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

#endif

    #ifdef OPT_ENCRYPTION____TMP
    ng_netconf_enable_t encrypt = NETCONF_ENABLE;      /* warning: use NETCONF_DISABLE before
                                                        * set OPT_ENCRYPTION to undefined */
    static  uint8_t key_buf[XBEE_ENCRYPTION_KEY_LEN]={0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
                 0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,}; // change those value to match your key
    dev.driver->set((ng_netdev_t *)&dev,NETCONF_OPT_ENCRYPTION, &encrypt,1);
    dev.driver->set((ng_netdev_t *)&dev,NETCONF_OPT_ENCRYPTION_KEY,key_buf,sizeof(key_buf));
    #endif

    /* start the shell */
    puts("Initialization OK, starting shell now");
    printf("\n\n Start Sending.....\n\n");
    //addr_dest=2222 -> 7E 00 0A 01 01 22 22 00 68 65 6C 6C 6F A5
    //
     uint8_t addr[8]={0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

     ng_pktsnip_t *pkt;
     ng_netif_hdr_t *nethdr;
     char data[]="hello";
    size_t addr_len = 2;

    /* put packet together */
     pkt = ng_pktbuf_add(NULL,(char *)data, strlen(data), NG_NETTYPE_UNDEF);
     pkt = ng_pktbuf_add(pkt, NULL, sizeof(ng_netif_hdr_t) + addr_len,
                         NG_NETTYPE_NETIF);
     nethdr = (ng_netif_hdr_t *)pkt->data;
     ng_netif_hdr_init(nethdr, 0, addr_len);

     ng_netif_hdr_set_dst_addr(nethdr, addr, addr_len);
     /* and send it */
    for ( int i= 0; i<300; i++){
     ng_netapi_send(iface, pkt);
     hwtimer_wait(1000*5000);
    }
    return 0;
}
