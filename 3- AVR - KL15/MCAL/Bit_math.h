/*
 * Bit_math.h
 *
 * Created: 17-Aug-23 2:31:54 PM
 *  Author: Salma
 */ 


#ifndef BIT_MATH_H_
#define BIT_MATH_H_

#define SET_BIT(REG,BIT)	(REG |= (1<<BIT))
#define ClEAR_BIT(REG,BIT)	(REG &= (~(1<<BIT)))
#define TOGGLE_BIT(REG,BIT) (REG ^= (1<<BIT))
#define READ_BIT(REG,BIT)	((REG>>BIT) & (0x01))

#endif /* BIT_MATH_H_ */