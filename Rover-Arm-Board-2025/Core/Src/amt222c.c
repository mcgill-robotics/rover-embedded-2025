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
	HAL_SPI_TransmitReceive(encoder->hspi, &tx, &rx, 1,
	AMT222C_SPI_TIMEOUT);
	delay_us(3);
	if (release == GPIO_PIN_SET)
		HAL_GPIO_WritePin(encoder->cs_port, encoder->cs_pin, GPIO_PIN_SET);
	return rx;
}

static uint8_t AMT222C_CalculateCheck(uint16_t value) {
	// Odd: K1 = !(H5^H3^H1^L7^L5^L3^L1)
	uint8_t bits[16];
	for (int i = 0; i < 16; i++)
		bits[i] = (0x01) & (value >> (i));
	return (bits[15]
			== !(bits[13] ^ bits[11] ^ bits[9] ^ bits[7] ^ bits[5] ^ bits[3]
					^ bits[1])) // Odd check
			&& (bits[14]
					== !(bits[12] ^ bits[10] ^ bits[8] ^ bits[6] ^ bits[4]
							^ bits[2] ^ bits[0])); // Even check
}

void AMT222C_Init(AMT222C_Handle *encoder) {
	HAL_Delay(200);  // Wait for encoder startup
	encoder->position_continuous = 0.0f;

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
	uint16_t raw_turn = ((uint16_t) rx2 << 8) | rx3;

//	if (AMT222C_CalculateCheck(pos) && AMT222C_CalculateCheck(raw_turn)) {
//		return AMT222C_ERR_CHECKSUM;
//	}

	*raw = pos & 0x0FFF;

	*turn = (int16_t) (raw_turn & 0x3FFF);
	if (*turn & 0x2000) {
		*turn |= 0xC000;
	}

	return AMT222C_OK;
}

AMT222C_Status AMT222C_UpdatePosition(AMT222C_Handle *encoder) {
	uint16_t raw;
	int16_t turn;
	if (AMT222C_ReadTurn(encoder, &raw, &turn) != AMT222C_OK)
		return AMT222C_ERR_SPI;

	encoder->ticks = (turn << 12) | raw;
	encoder->position_continuous = (float) encoder->ticks / AMT222C_MAX_TICKS
			/ encoder->gear_ratio * 360.0f;

	return AMT222C_OK;
}

float AMT222C_GetPosition(AMT222C_Handle *encoder) {
	return encoder->position_continuous;
}
