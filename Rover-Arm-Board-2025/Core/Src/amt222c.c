// amt222c.c
#include "amt222c.h"

static void delay_us(uint32_t us) {
	uint32_t start = DWT->CYCCNT;
	uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
	while ((DWT->CYCCNT - start) < ticks)
		;
}

static uint8_t spi_transfer_byte(AMT222C_Handle *encoder, uint8_t tx,
		GPIO_PinState release) {
	uint8_t rx = 0;
	HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_RESET);
	delay_us(3);
	HAL_StatusTypeDef err = HAL_SPI_TransmitReceive(encoder->hspi, &tx, &rx, 1,
			AMT222C_SPI_TIMEOUT);
	delay_us(3);
	if (release == GPIO_PIN_SET)
		HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_SET);
	return rx;
}

static uint8_t AMT222C_CalculateOddCheck(uint16_t value) {
	return !((value >> 15 ^ value >> 13 ^ value >> 11 ^ value >> 9 ^ value >> 7
			^ value >> 5 ^ value >> 3 ^ value >> 1) & 0x01);
}

static uint8_t AMT222C_CalculateEvenCheck(uint16_t value) {
	return !((value >> 14 ^ value >> 12 ^ value >> 10 ^ value >> 8 ^ value >> 6
			^ value >> 4 ^ value >> 2 ^ value >> 0) & 0x01);
}

void AMT222C_Init(AMT222C_Handle *encoder) {
	HAL_Delay(200);  // Wait for encoder startup
	encoder->position_continuous = 0.0f;
	encoder->last_raw = 0;
	encoder->last_turn = 0;

	// Enable DWT if not already done
	if (!(DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

AMT222C_Status AMT222C_ReadTurn(AMT222C_Handle *encoder, uint16_t *raw,
		int16_t *turn) {
	uint8_t rx0, rx1, rx2, rx3;

	HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_RESET);
	delay_us(3);
	rx0 = spi_transfer_byte(encoder, 0x00, GPIO_PIN_RESET);
	delay_us(3);
	rx1 = spi_transfer_byte(encoder, 0xA0, GPIO_PIN_RESET);
	delay_us(3);
	rx2 = spi_transfer_byte(encoder, 0x00, GPIO_PIN_RESET);
	delay_us(3);
	rx3 = spi_transfer_byte(encoder, 0x00, GPIO_PIN_SET);

	uint16_t pos = ((uint16_t) rx0 << 8) | rx1;
	uint8_t odd = (pos >> 15) & 0x01;
	uint8_t even = (pos >> 14) & 0x01;

	if (odd != AMT222C_CalculateOddCheck(pos)
			|| even != AMT222C_CalculateEvenCheck(pos)) {
		return AMT222C_ERR_CHECKSUM;
	}

	*raw = pos & 0x0FFF;

	uint16_t raw_turn = ((uint16_t) rx2 << 8) | rx3;
	*turn = (int16_t) (raw_turn & 0x3FFF);
	if (*turn & 0x2000) {
		*turn |= 0xC000;
	}

	return AMT222C_OK;
}

AMT222C_Status AMT222C_ReadRaw(AMT222C_Handle *encoder, uint16_t *raw) {
	int16_t dummy;
	return AMT222C_ReadTurn(encoder, raw, &dummy);
}

AMT222C_Status AMT222C_UpdatePosition(AMT222C_Handle *encoder) {
	uint16_t raw;
	int16_t turn;
	if (AMT222C_ReadTurn(encoder, &raw, &turn) != AMT222C_OK)
		return AMT222C_ERR_SPI;

	int32_t absolute_ticks = ((int32_t) turn * AMT222C_TURN_SCALER) + raw;
	int32_t last_ticks = ((int32_t) encoder->last_turn * AMT222C_TURN_SCALER)
			+ encoder->last_raw;
	int32_t delta = absolute_ticks - last_ticks;

	encoder->position_continuous += (float) delta / AMT222C_MAX_TICKS
			* encoder->gear_ratio;
	encoder->last_raw = raw;
	encoder->last_turn = turn;
	return AMT222C_OK;
}

float AMT222C_GetPosition(AMT222C_Handle *encoder) {
	return encoder->position_continuous;
}
