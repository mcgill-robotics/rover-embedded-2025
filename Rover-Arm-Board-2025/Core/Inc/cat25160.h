#ifndef CAT25160_H
#define CAT25160_H

#include "stm32f4xx_hal.h"

#define CAT25160_CMD_WREN   0x06
#define CAT25160_CMD_WRDI   0x04
#define CAT25160_CMD_RDSR   0x05
#define CAT25160_CMD_WRSR   0x01
#define CAT25160_CMD_READ   0x03
#define CAT25160_CMD_WRITE  0x02

#define CAT25160_PAGE_SIZE  32

// Modify with your setup
#define CAT25160_CS_LOW()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET)
#define CAT25160_CS_HIGH()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET)

union CAT25160_StatusRegister{
	struct {
	uint8_t wpen : 1;
	uint8_t reserved: 3;
	uint8_t bp1: 1;
	uint8_t bp0: 1;
	uint8_t wel: 1;
	uint8_t nrdy: 1;
	} bits;
	uint8_t value;
};

void CAT25160_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef CAT25160_Read(uint16_t addr, uint8_t *data, uint16_t len);
HAL_StatusTypeDef CAT25160_WritePage(uint16_t addr, const uint8_t *data, uint16_t len);
uint8_t CAT25160_ReadStatus(void);
void CAT25160_WriteEnable(void);

#endif
