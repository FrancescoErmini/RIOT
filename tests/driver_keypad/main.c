
#include <stdio.h>
#include <stdint.h>
#include "periph/gpio.h"
#include "periph_conf.h"
#include "keypad4x4.h"
#include "mutex.h"
static keypad_t keypad;

int main (void)
{

 puts("Matrix keypad 4x4 test with GPIO and Interrupt");
 puts("type the secret key now...");
gpio_t cols[4] = {0, 1, 2, 3};
gpio_t rows[4] = {8, 5, 6, 7};
int iter_max = 5;
uint8_t data[iter_max];

keypad4x4(&keypad,rows,cols,iter_max,data);
/**
 * @TODO implement sync control to notified the data is ready to read end can't read before
 * @TODO use timeout
 * @TODO write exit cases on return value function
 */
}

