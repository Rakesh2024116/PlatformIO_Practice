#include "gpio_driver.h"

void init_GPIO(unsigned short port, unsigned short pin, unsigned short dir, unsigned short opt)
{
	volatile unsigned long* CR;
	unsigned short tPin = pin;
	unsigned short offset = 0x00;
	
	if(pin > 7)
	{
		tPin = 8;
		offset = 0x01;
	}
	
	if (port ==1)
	{
		RCC_APB2ENR |= 0x0010;	// Enabling Port A
		CR = (volatile unsigned long*) (&GPIO_A + offset);
	}
	else if (port ==2)
	{
		RCC_APB2ENR |= 0x0100; // Enabling Port B
		CR = (volatile unsigned long*) (&GPIO_B + offset);
	}
	else if (port ==3)
	{
		RCC_APB2ENR |= 0x1000; // Enabling Port B
		CR = (volatile unsigned long*) (&GPIO_C + offset);
	}
	
	*CR &= ~(0xf<<(tPin)*4); // Reset the target pin
	*CR |= ((dir <<(tPin*4)) | (opt<<(tPin*4+2))); //Port Mode and the option for the pin
}

int Read_GP(unsigned short port, unsigned short pin)
{
	volatile unsigned long* IDR;
	unsigned long offset = 0x02;
	int state;
	
	if (port == 1)
	{
		IDR = (volatile unsigned long*) (&GPIO_A + offset);
	}
	else if(port == 2)
	{
		IDR = (volatile unsigned long*) (&GPIO_B + offset);
	}
	else if(port == 3)
	{
		IDR = (volatile unsigned long*) (&GPIO_C + offset);
	}
	
	state = ((*IDR & (1 << pin)) >> pin);
	return state;
}

void Write_GP(unsigned short port, unsigned short pin, unsigned short state)
{
	volatile unsigned long* ODR;
	unsigned long offset = 0x03;
	// int state;
	
	if (port == 1)
	{
		ODR = (volatile unsigned long*) (&GPIO_A + offset);
	}
	else if(port == 2)
	{
		ODR = (volatile unsigned long*) (&GPIO_B + offset);
	}
	else if(port == 3)
	{
		ODR = (volatile unsigned long*) (&GPIO_C + offset);
	}
	
	state ? (*ODR |= (state << pin)) : (*ODR &= ~(1<< pin));
}

void toggle_GP(unsigned short port, unsigned short pin)
{
	if (Read_GP(port, pin))
	{
		Write_GP(port, pin, 0);
	}
	else
	{
		Write_GP(port, pin, 1);
	}
}

