/*
 *
 *
GPIO_0 +------------#-7-4-1--c0
GPIO_1 +------------0-8-5-2--c1
GPIO_2 +------------*-9-6-3--c2
GPIO_3 +------------D-C-B-A--c3
--------------------|-|-|-|
GPIO_8 <------------|-|-|-r0
GPIO_5 <------------|-|-r1
GPIO_6 <------------|-r2
GPIO_7 <------------r3

NB: use GPIO_4 instead of GPIO_8 and vice versa. I got problem with GPIO_4 by I don't know why.

*/
 

#ifndef XBEE_H_
#define XBEE_H_

#include <stdint.h>

 #include <stdio.h>
#include <stdint.h>
#include "periph_conf.h"
#include "periph/gpio.h"
#include "mutex.h"
#ifdef __cplusplus
 extern "C" {
#endif



typedef struct {
gpio_t gpio_cols[4];
gpio_t gpio_rows[4];
int iter,iter_max;
uint8_t *data;
mutex_t mutex;
} keypad_t;


void keypad4x4(keypad_t *keypad, gpio_t gpio_row[4], gpio_t gpio_col[4],int iter_max,void *data);

#ifdef __cplusplus
}
#endif

#endif /* XBEE_H_ */
/** @} */
