#include <stdio.h>
#include <stdint.h>
#include "periph_conf.h"
#include "periph/gpio.h"
#include "keypad4x4.h"
#include "hwtimer.h"
#define delay(X) (hwtimer_wait(1000*X))
#define ACTIVE_FLANK        GPIO_RISING
#define NOPULL              GPIO_NOPULL
#define PULL				GPIO_NOPULL
#define PULLUP				GPIO_PULLUP
unsigned int _row[4];
unsigned int _col[4];



void read_button(keypad_t *keypad);
void read_rows(keypad_t *keypad);
void read_cols(keypad_t *keypad);

void _switch_row(keypad_t *keypad);
void _switch_col(keypad_t *keypad);
uint8_t _keypad_parser(void);
void _reset_parser(void);

//scan which row an col have  been set and retun the number push on the keypad
uint8_t _keypad_parser (void){

if(_row[0]==1 && _col[0]==1){return 0x1;}
if(_row[0]==1 && _col[1]==1){return 0x2;}
if(_row[0]==1 && _col[2]==1){return 0x3;}
if(_row[0]==1 && _col[3]==1){return 0xA;}

if(_row[1]==1 && _col[0]==1){return 0x4;}
if(_row[1]==1 && _col[1]==1){return 0x5;}
if(_row[1]==1 && _col[2]==1){return 0x6;}
if(_row[1]==1 && _col[3]==1){return 0xB;}

if(_row[2]==1 && _col[0]==1){return 0x7;}
if(_row[2]==1 && _col[1]==1){return 0x8;}
if(_row[2]==1 && _col[2]==1){return 0x9;}
if(_row[2]==1 && _col[3]==1){return 0xC;}

if(_row[3]==1 && _col[0]==1){return 0xD;}//17 as #
if(_row[3]==1 && _col[1]==1){return 0;}
if(_row[3]==1 && _col[2]==1){return 0xE;}//18 as *
if(_row[3]==1 && _col[3]==1){return 0xF;}

return -1;
}
// reset variable row and col before pushing a new button
void _reset_parser(void){
	for(int i=0;i<4;i++)
	{
		_row[i]=0;
		_col[i]=0;
	}
}
// callback functions  for colums
void c0_cb(void *arg){
   keypad_t *keypad = (keypad_t *)arg;
	_col[0]=1;
	_switch_row(keypad);
}
void c1_cb(void *arg){
	keypad_t *keypad = (keypad_t *)arg;
	_col[1]=1;
	_switch_row(keypad);
}
void c2_cb(void *arg){
	 keypad_t *keypad = (keypad_t *)arg;
	_col[2]=1;
	_switch_row(keypad);
}
void c3_cb(void *arg){
	 keypad_t *keypad = (keypad_t *)arg;
	_col[3]=1;
	_switch_row(keypad);
}
//callback function for rows

void r0_cb(void *arg){
	 keypad_t *keypad = (keypad_t *)arg;
	_row[0]=1;
	_switch_col(keypad);
}

static void r1_cb(void *arg){
	 keypad_t *keypad = (keypad_t *)arg;
	_row[1]=1;
	_switch_col(keypad);
}

void r2_cb(void *arg){
	 keypad_t *keypad = (keypad_t *)arg;
	_row[2]=1;
	_switch_col(keypad);
}

void r3_cb(void *arg){
	 keypad_t *keypad = (keypad_t *)arg;
	_row[3]=1;
	_switch_col(keypad);
}


void _switch_col(keypad_t *keypad){
// hack needed for correct switch configuration

	/*NVIC_DisableIRQ(GPIO_8_IRQ);
	NVIC_DisableIRQ(GPIO_5_IRQ);
	NVIC_DisableIRQ(GPIO_6_IRQ);
	NVIC_DisableIRQ(GPIO_7_IRQ);*/

	gpio_irq_disable(keypad->gpio_rows[0]);
	gpio_irq_disable(keypad->gpio_rows[1]);
	gpio_irq_disable(keypad->gpio_rows[2]);
	gpio_irq_disable(keypad->gpio_rows[3]);

	read_cols(keypad);

}
void _switch_row(keypad_t *keypad)
{
// hack needed for correct switch configuration 
/*NVIC_DisableIRQ(GPIO_0_IRQ);
    NVIC_DisableIRQ(GPIO_1_IRQ);
	NVIC_DisableIRQ(GPIO_2_IRQ);
	NVIC_DisableIRQ(GPIO_3_IRQ);*/
gpio_irq_disable(keypad->gpio_cols[0]);
gpio_irq_disable(keypad->gpio_cols[1]);
gpio_irq_disable(keypad->gpio_cols[2]);
gpio_irq_disable(keypad->gpio_cols[3]);

read_button(keypad);



}
void read_button(keypad_t *keypad){
//puts("dentro read_button");

	int push_button = _keypad_parser(); //read the number
	keypad->data[keypad->iter++] = push_button;
	printf("\n\n\n PIGIATO IL BOTTONE: %i",push_button);
	 _reset_parser();
	 delay(1000);
	 if(keypad->iter != keypad->iter_max){ //max number you want to read, then stop.
		  read_rows(keypad);
		 }
		 else {
			 puts("done.");
			 mutex_unlock(&(keypad->mutex));
				//gpio_init_out(GPIO_4,PULLUP); gpio_set(GPIO_4); // puts a led on when finished

		 }

}
void read_rows(keypad_t *keypad)
{
/*	gpio_init_out(GPIO_0,PULL); gpio_set(GPIO_0);
		gpio_init_out(GPIO_1,PULL); gpio_set(GPIO_1);
		gpio_init_out(GPIO_2,PULL); gpio_set(GPIO_2);
		gpio_init_out(GPIO_3,PULL); gpio_set(GPIO_3);

		gpio_init_int(GPIO_8, PULL,ACTIVE_FLANK , r0_cb, keypad);
		gpio_init_int(GPIO_5, PULL,ACTIVE_FLANK , r1_cb, keypad);
		gpio_init_int(GPIO_6, PULL,ACTIVE_FLANK , r2_cb, keypad);
		gpio_init_int(GPIO_7, PULL,ACTIVE_FLANK , r3_cb, gkeypad);

*/
			gpio_init_out(keypad->gpio_cols[0],NOPULL); gpio_set(keypad->gpio_cols[0]);
			gpio_init_out(keypad->gpio_cols[1],NOPULL); gpio_set(keypad->gpio_cols[1]);
			gpio_init_out(keypad->gpio_cols[2],NOPULL); gpio_set(keypad->gpio_cols[2]);
			gpio_init_out(keypad->gpio_cols[3],NOPULL); gpio_set(keypad->gpio_cols[3]);

			gpio_init_int(keypad->gpio_rows[0], NOPULL,ACTIVE_FLANK , r0_cb, keypad);
			gpio_init_int(keypad->gpio_rows[1], NOPULL,ACTIVE_FLANK , r1_cb, keypad);
			gpio_init_int(keypad->gpio_rows[2], NOPULL,ACTIVE_FLANK , r2_cb, keypad);
			gpio_init_int(keypad->gpio_rows[3], NOPULL,ACTIVE_FLANK , r3_cb, keypad);

}
void read_cols(keypad_t *keypad)
{

		gpio_init_int(keypad->gpio_cols[0], NOPULL,ACTIVE_FLANK , c0_cb, keypad);
		gpio_init_int(keypad->gpio_cols[1], NOPULL,ACTIVE_FLANK , c1_cb, keypad);
		gpio_init_int(keypad->gpio_cols[2], NOPULL,ACTIVE_FLANK , c2_cb, keypad);
		gpio_init_int(keypad->gpio_cols[3], NOPULL,ACTIVE_FLANK , c3_cb, keypad);

		 gpio_init_out(keypad->gpio_rows[0],NOPULL); gpio_set(keypad->gpio_rows[0]);
		 gpio_init_out(keypad->gpio_rows[1],NOPULL); gpio_set(keypad->gpio_rows[1]);
		 gpio_init_out(keypad->gpio_rows[2],NOPULL); gpio_set(keypad->gpio_rows[2]);
		 gpio_init_out(keypad->gpio_rows[3],NOPULL); gpio_set(keypad->gpio_rows[3]);
	/*gpio_init_int(GPIO_0, PULL,ACTIVE_FLANK , c0_cb, keypad);
				 gpio_init_int(GPIO_1, PULL,ACTIVE_FLANK , c1_cb, keypad);
				 gpio_init_int(GPIO_2, PULL,ACTIVE_FLANK , c2_cb, keypad);
				 gpio_init_int(GPIO_3, PULL,ACTIVE_FLANK , c3_cb, keypad);

					 gpio_init_out(GPIO_5,PULL);
					 gpio_init_out(GPIO_6,PULL);
					 gpio_init_out(GPIO_8,PULL);
					 gpio_init_out(GPIO_7,PULL);
					 gpio_set(GPIO_5);
							 		 gpio_set(GPIO_6);
							 		 gpio_set(GPIO_7);
							 		gpio_set(GPIO_8);
*/
}
void keypad4x4(keypad_t *keypad,gpio_t rows[4], gpio_t cols[4],int iter_max,void * data)
{

for (int i=0;i<4;i++){
	keypad->gpio_rows[i] = rows[i];
	keypad->gpio_cols[i] = cols[i];
}
	keypad->iter = 0;
	keypad->iter_max = iter_max;
	keypad->data = data;


	_reset_parser();// init parser vector
	//start reading rows
	read_rows(keypad);
}
