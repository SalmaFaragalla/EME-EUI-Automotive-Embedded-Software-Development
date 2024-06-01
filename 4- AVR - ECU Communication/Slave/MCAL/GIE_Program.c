/*
 * GIE_Program.c
 *
 *  Created on: ??þ/??þ/????
 *      Author: moham
 */

#include "STD_TYPES.h"

#include "GIE_Private.h"
#include "BIT_MATH.h"

void GIE_Enable(void)
{
	SET_BIT(SREG , I_BIT);
}
void GIE_Disable(void)
{
	CLR_BIT(SREG , I_BIT);
}
