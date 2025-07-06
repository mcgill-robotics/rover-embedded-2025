/*
 * brushed_system.h
 *
 *  Created on: Jul 5, 2025
 *      Author: vince
 */

#ifndef INC_BRUSHED_SYSTEM_H_
#define INC_BRUSHED_SYSTEM_H_

enum MotorState {
	OFF,
	POSITION_NORMAL,
	POSITION_HIT_LIMIT,
	POSITION_FAULT,
	HOMING_LOWER,
	HOMING_UPPER,
	ERROR
};

#endif /* INC_BRUSHED_SYSTEM_H_ */
