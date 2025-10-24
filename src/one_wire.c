/* Standard libraries */
#include <string.h>

/* ESP-IDF */
#include "driver/uart.h"
#include "driver/gpio.h"
#include "soc/uart_periph.h"
#include "esp_rom_gpio.h"

/* ESP Logging */
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#include "esp_log.h"

/* Public header file */
#include "one_wire.h"

/* Private definitions - UART bytes representing 1-Wire protocol symbols */
#define OW_BYTE_RESET (0xF0U)
#define OW_BYTE_ONE (0xFFU)
#define OW_BYTE_ZERO (0x00U)

/* 1-Wire commands */
#define OW_CMD_SEARCH_ROM (0xF0U)
#define OW_CMD_MATCH_ROM (0x55U)
#define OW_CMD_SKIP_ROM (0xCCU)

/* ROM code size in bits */
#define OW_ROM_CODE_SIZE_BITS (64U)

/* UART TX/RX timeout ms */
#define OW_UART_TIMEOUT_MS (100U)

/* Private variables - log tag */
static const char *TAG = "ONE_WIRE";

/* Public functions */
void ONE_WIRE_Init(uart_port_t eUART, gpio_num_t eGPIONum)
{
	/* Debug */
	ESP_LOGI(TAG, "Initializing 1-Wire on UART %d, GPIO %d", eUART, eGPIONum);

	/* Configure UART module */
	ESP_ERROR_CHECK(uart_driver_install(eUART, UART_HW_FIFO_LEN(eUART) + 1, 0, 0, NULL, ESP_INTR_FLAG_IRAM));
	uart_config_t stUARTConfig =
	{
		.baud_rate = 9600,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_1,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	};
	ESP_ERROR_CHECK(uart_param_config(eUART, &stUARTConfig));

	/* Set RX timeout to 1 bit time (to detect end of slot) */
	ESP_ERROR_CHECK(uart_set_rx_timeout(eUART, 1));

	/* Configure GPIO pin */
	const gpio_config_t stPinCfg =
	{
		.pin_bit_mask = (1ul << eGPIONum),
		.mode = GPIO_MODE_INPUT_OUTPUT_OD, /* Input and Output w/ open-drain! */
		.pull_up_en = GPIO_PULLUP_ENABLE, /* Open-drain requires a pull-up. */
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&stPinCfg));

	/* Connect both RX and TX to the same pin */
	esp_rom_gpio_connect_out_signal(eGPIONum, UART_PERIPH_SIGNAL(eUART, SOC_UART_TX_PIN_IDX), false, false);
	esp_rom_gpio_connect_in_signal(eGPIONum, UART_PERIPH_SIGNAL(eUART, SOC_UART_RX_PIN_IDX), false);
}

void ONE_WIRE_Deinit(uart_port_t eUART, gpio_num_t eGPIONum)
{
	/* Debug */
	ESP_LOGI(TAG, "Deinitializing 1-Wire on UART %d, GPIO %d", eUART, eGPIONum);

	/* Delete UART driver */
	ESP_ERROR_CHECK(uart_driver_delete(eUART));

	/* Return pin to GPIO input */
	const gpio_config_t stPinCfg =
	{
		.pin_bit_mask = (1ul << eGPIONum),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE
	};
	ESP_ERROR_CHECK(gpio_config(&stPinCfg));
}

bool ONE_WIRE_Reset(uart_port_t eUART)
{
	bool bDevicePresent = false;

	/* Debug */
	ESP_LOGD(TAG, "Sending 1-Wire reset on UART %d", eUART);

	/* Update UART baud rate to 9600 for reset pulse */
	uart_set_baudrate(eUART, 9600);

	/* Flush input buffer */
	uart_flush_input(eUART);

	/* Transmit reset command */
	uint8_t byDataOut = OW_BYTE_RESET;
	uart_write_bytes(eUART, (const char *)&byDataOut, sizeof(byDataOut));

	/* Wait until transmission is complete */
	uart_wait_tx_done(eUART, pdMS_TO_TICKS(OW_UART_TIMEOUT_MS));

	/* Read response */
	uint8_t byDataIn;
	if (uart_read_bytes(eUART, &byDataIn, sizeof(byDataIn), pdMS_TO_TICKS(OW_UART_TIMEOUT_MS)) > 0)
	{
		/* Presence pulse is indicated by readback other than reset byte (aka a device has responded) */
		bDevicePresent = (OW_BYTE_RESET != byDataIn);
	}

	/* Return UART baud rate to 115200 for data */
	uart_set_baudrate(eUART, 115200);

	/* Debug */
	ESP_LOGD(TAG, "1-Wire reset on UART %d %s", eUART, bDevicePresent ? "detected a device" : "did not detect a device");

	/* Return presence indication */
	return bDevicePresent;
}

void ONE_WIRE_WriteBit(uart_port_t eUART, uint8_t uiBit)
{
	/* Debug */
	ESP_LOGD(TAG, "Writing bit %u on UART %d", uiBit ? 1 : 0, eUART);

	/* Prepare byte to send */
	uint8_t byDataOutIn = uiBit ? OW_BYTE_ONE : OW_BYTE_ZERO;

	/* Send bit (represented by a byte) */
	uart_write_bytes(eUART, (const char *)&byDataOutIn, sizeof(byDataOutIn));

	/* Wait until transmission is complete */
	uart_wait_tx_done(eUART, pdMS_TO_TICKS(OW_UART_TIMEOUT_MS));

	/* Read back written bytes */
	uart_read_bytes(eUART, &byDataOutIn, sizeof(byDataOutIn), pdMS_TO_TICKS(OW_UART_TIMEOUT_MS));
}

uint8_t ONE_WIRE_ReadBit(uart_port_t eUART)
{
	/* Prepare byte to send and assume one received */
	uint8_t byDataOut = OW_BYTE_ONE;
	uint8_t byDataIn = 1;

	/* Debug */
	ESP_LOGD(TAG, "Reading bit on UART %d", eUART);

	/* Ensure input buffer flushed */
	uart_flush_input(eUART);

	/* Send read time slot */
	uart_write_bytes(eUART, (const char *)&byDataOut, sizeof(byDataOut));

	/* Wait until transmission is complete */
	uart_wait_tx_done(eUART, pdMS_TO_TICKS(OW_UART_TIMEOUT_MS));

	/* Read response */
	if (uart_read_bytes(eUART, &byDataIn, sizeof(byDataIn), pdMS_TO_TICKS(OW_UART_TIMEOUT_MS)) > 0)
	{
		/* If received byte is OW_BYTE_ONE, the bit is 1; otherwise it's 0 */
		byDataIn = (byDataIn == OW_BYTE_ONE) ? 1 : 0;

		/* Debug */
		ESP_LOGD(TAG, "Received bit: %u on UART %d", byDataIn ? 1 : 0, eUART);
	}
	else
	{
		/* Debug */
		ESP_LOGE(TAG, "No data received when reading bit on UART %d", eUART);
	}

	return byDataIn;
}

void ONE_WIRE_WriteByte(uart_port_t eUART, uint8_t uiByte)
{
	/* Debug */
	ESP_LOGD(TAG, "Writing byte 0x%02X on UART %d", uiByte, eUART);

	/* Write each bit in turn, LSB first */
	for (uint8_t i = 0U; i < 8U; i++)
	{
		ONE_WIRE_WriteBit(eUART, (uiByte >> i) & 0x01U);
	}
}

uint8_t ONE_WIRE_ReadByte(uart_port_t eUART)
{
	uint8_t uiByte = 0;

	/* Debug */
	ESP_LOGD(TAG, "Reading byte on UART %d", eUART);

	/* Read each bit in turn, LSB first */
	for (uint8_t i = 0U; i < 8U; i++)
	{
		if (0U != ONE_WIRE_ReadBit(eUART))
		{
			uiByte |= (1U << i);
		}
	}

	return uiByte;
}

/* Dallas/Maxim 1-Wire CRC8 polynomial: x^8 + x^5 + x^4 + 1 (0x31, reversed is 0x8C) */
uint8_t ONE_WIRE_CRC8(const uint8_t *pbyData, size_t uiLength)
{
	uint8_t uiCRC = 0;

	for (size_t i = 0; i < uiLength; i++)
	{
		uiCRC ^= pbyData[i];

		for (uint8_t j = 0; j < 8; j++)
		{
			if (uiCRC & 0x01)
			{
				uiCRC = (uiCRC >> 1) ^ 0x8C;
			}
			else
			{
				uiCRC >>= 1;
			}
		}
	}

	return uiCRC;
}

bool ONE_WIRE_Search(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, uint8_t *puiLastDiscrepancy)
{
	bool bSearchSuccess = true;
	uint8_t uiLastZero = 0U;

	/* Debug */
	ESP_LOGD(TAG, "Searching for 1-Wire devices on UART %d", eUART);

	/* Check for valid pointers */
	if (pstROMCode == NULL || puiLastDiscrepancy == NULL)
	{
		ESP_LOGE(TAG, "Invalid arguments");
		bSearchSuccess = false;
	}

	if (bSearchSuccess)
	{
		/* Reset bus */
		if (!ONE_WIRE_Reset(eUART))
		{
			/* No devices present */
			*puiLastDiscrepancy = 0U;
			bSearchSuccess = false;
		}
	}

	if (bSearchSuccess)
	{
		/* Issue Search ROM command */
		ONE_WIRE_WriteByte(eUART, OW_CMD_SEARCH_ROM);

		/* Clear ROM code buffer */
		memset(pstROMCode, 0x00, sizeof(*pstROMCode));

		/* Iterate until all bits have been read */
		uint8_t uiROMBitMask = 0x01U;
		uint8_t uiROMByteIndex = 0U;
		for (uint8_t uiIDBitNumber = 1U; uiIDBitNumber <= OW_ROM_CODE_SIZE_BITS; uiIDBitNumber++)
		{
			/* Read bit and its complement */
			uint8_t uiIDBit = ONE_WIRE_ReadBit(eUART);
			uint8_t uiIDBitComplement = ONE_WIRE_ReadBit(eUART);

			uint8_t uiSearchDir;
			if (1U == uiIDBit && 1U == uiIDBitComplement)
			{
				/* No devices participating in search */
				bSearchSuccess = false;
				break;
			}
			else if ((0U == uiIDBit) && (0U == uiIDBitComplement))
			{
				/* Discrepancy, choose direction */
				if (uiIDBitNumber < *puiLastDiscrepancy)
				{
					uiSearchDir = (0U != (pstROMCode->abyData[uiROMByteIndex] & uiROMBitMask));
				}
				else
				{
					uiSearchDir = (uiIDBitNumber == *puiLastDiscrepancy);
				}
				if (0U == uiSearchDir) uiLastZero = uiIDBitNumber;
			}
			else
			{
				/* No discrepancy, bit must be ID bit */
				uiSearchDir = (1U == uiIDBit) ? 1U : 0U;
			}

			/* Write chosen direction */
			ONE_WIRE_WriteBit(eUART, uiSearchDir);

			/* Store bit in ROM code buffer */
			if (1U == uiSearchDir)
			{
				pstROMCode->abyData[uiROMByteIndex] |= uiROMBitMask;
			}
			else
			{
				pstROMCode->abyData[uiROMByteIndex] &= ~uiROMBitMask;
			}

			/* Advance to next bit */
			uiROMBitMask <<= 1U;
			if (0U == uiROMBitMask)
			{
				uiROMBitMask = 0x01U;
				uiROMByteIndex++;
			}
		}
	}

	if (bSearchSuccess)
	{
		/* Update last discrepancy for next call */
		*puiLastDiscrepancy = uiLastZero;

		/* Check CRC */
		uint8_t uiExpectedCRC = pstROMCode->abyData[ONE_WIRE_ROM_CODE_SIZE - 1U];
		uint8_t uiCalculatedCRC = ONE_WIRE_CRC8(pstROMCode->abyData, ONE_WIRE_ROM_CODE_SIZE - 1U);
		bSearchSuccess = (uiCalculatedCRC == uiExpectedCRC);
	}

	return bSearchSuccess;
}

void ONE_WIRE_MatchROM(uart_port_t eUART, const stONE_WIRE_ROM_Code_t *pstROMCode)
{
	bool bSuccess = true;

	/* Debug */
	ESP_LOGD(TAG, "Matching ROM code on UART %d", eUART);

	if (NULL == pstROMCode)
	{
		ESP_LOGE(TAG, "Invalid argument");
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Issue Match ROM command */
		ONE_WIRE_WriteByte(eUART, OW_CMD_MATCH_ROM);

		/* Write ROM code */
		for (size_t i = 0; i < ONE_WIRE_ROM_CODE_SIZE; i++)
		{
			ONE_WIRE_WriteByte(eUART, pstROMCode->abyData[i]);
		}
	}
}

void ONE_WIRE_SkipROM(uart_port_t eUART)
{
	/* Debug */
	ESP_LOGD(TAG, "Issuing Skip ROM command on UART %d", eUART);

	/* Issue Skip ROM command */
	ONE_WIRE_WriteByte(eUART, OW_CMD_SKIP_ROM);
}

bool ONE_WIRE_ROMCodeToStr(const stONE_WIRE_ROM_Code_t *pstROMCode, char *pcBuffer, size_t uiBufferSize)
{
	bool bSuccess = true;

	/* Debug */
	ESP_LOGD(TAG, "Converting ROM code to string");

	/* Validate arguments */
	if (	(NULL == pstROMCode)
		 || (NULL == pcBuffer)
		 || (uiBufferSize < ONE_WIRE_ROM_CODE_STR_BUF_SIZE)
	   )
	{
		ESP_LOGE(TAG, "Invalid arguments");
		bSuccess = false;
	}

	if (bSuccess)
	{
		/* Convert each byte to hex string */
		size_t uiPos = 0U;
		for (size_t i = 0U; i < ONE_WIRE_ROM_CODE_SIZE; i++)
		{
			int iWritten = snprintf(&pcBuffer[uiPos], uiBufferSize - uiPos, "%02X", pstROMCode->abyData[i]);
			if (iWritten < 0)
			{
				ESP_LOGE(TAG, "Failed to write byte %zu to string", i);
				bSuccess = false;
				break;
			}
			uiPos += (size_t)iWritten;
		}

		/* Null-terminate string */
		if (bSuccess)
		{
			pcBuffer[uiPos] = '\0';
		}
	}

	return bSuccess;
}

uint8_t ONE_WIRE_GetFamilyCode(const stONE_WIRE_ROM_Code_t *pstROMCode)
{
	uint8_t uiFamilyCode = UINT8_MAX;

	/* Debug */
	ESP_LOGD(TAG, "Getting family code from ROM code");

	if (NULL == pstROMCode)
	{
		ESP_LOGE(TAG, "Invalid argument");
	}
	else
	{
		uiFamilyCode = pstROMCode->abyData[0];
	}

	return uiFamilyCode;
}
