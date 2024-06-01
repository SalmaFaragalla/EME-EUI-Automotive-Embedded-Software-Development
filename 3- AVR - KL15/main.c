/*
 * DIO.c
 *
 * Created: 17-Aug-23 2:29:20 PM
 * Author : Ahmed hisham
 */ 

#include "klima.h"

int main(void)
{
	Klima_init();
    while (1)
    {
		Klima_runnable();
    }

}