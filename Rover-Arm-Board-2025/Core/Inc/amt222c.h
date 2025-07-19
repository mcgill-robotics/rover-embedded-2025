/*
 * amt222c.h
 *
 *  Created on: Jul 18, 2025
 *      Author: vince
 */

#ifndef INC_AMT222C_H_
#define INC_AMT222C_H_

#include "stm32f4xx_hal.h"

#define AMT222C_SPI_TIMEOUT 10
#define AMT222C_MAX_TICKS 4096  // 12-bit resolution for position
#define AMT222C_TURN_SCALER 4096

typedef enum {
	AMT222C_OK = 0, AMT222C_ERR_SPI = -1, AMT222C_ERR_CHECKSUM = -2
} AMT222C_Status;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	float gear_ratio;
	float position_continuous;
	uint16_t last_raw;
	int16_t last_turn;
} AMT222C_Handle;

void AMT222C_Init(AMT222C_Handle *encoder);
AMT222C_Status AMT222C_ReadRaw(AMT222C_Handle *encoder, uint16_t *raw);
AMT222C_Status AMT222C_ReadTurn(AMT222C_Handle *encoder, uint16_t *raw,
		int16_t *turn);
AMT222C_Status AMT222C_UpdatePosition(AMT222C_Handle *encoder);
float AMT222C_GetPosition(AMT222C_Handle *encoder);

#endif /* INC_AMT222C_H_ */
