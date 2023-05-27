/*
 * rv_common.h
 *
 *  Created on: May 26, 2023
 *      Author: TK
 */

#ifndef INC_RV_COMMON_H_
#define INC_RV_COMMON_H_

#include <stdint.h>

#define SET_BITS(REG, BIT)                    ((REG) |= (BIT))
#define CLEAR_BITS(REG, BIT)                  ((REG) &= ~(BIT))
#define READ_BITS(REG, BIT)                   ((REG) & (BIT))
#define WRITE_BITS(REG, CLEARMASK, SETMASK)   ((REG) = (((REG) & (~(CLEARMASK))) | (SETMASK)))

#endif /* INC_RV_COMMON_H_ */
