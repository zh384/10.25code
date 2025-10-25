#include "stdio.h"
#include "gpio.h"
#include "valve.h"

void CLK_set(){
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);
}

void CLK_reset(){
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_RESET);
}

void SI_reset(){
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);
}

void SI_set(){
     HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);
}

void radiotube(uint8_t cmd){
  uint8_t data;
	if(cmd>0xff){
	   data=0xff;
	}
	else{
	   data=cmd;
	}
	CLK_reset();
	SI_set();
	SI_reset();
	CLK_set();
	for(int i=0;i<4;i++)
	{
	  CLK_reset();
		if((data&0xff)!=(data>>1)<<1){
		   SI_set();
		}
		else{
		   SI_reset();
		}
		CLK_set();
		data=data>>1;
	
	}
	CLK_reset();
  SI_set();
  SI_reset();	
  CLK_set();
}


