#ifndef ONE_WIRE_H
#define ONE_WIRE_H

/* Standard libraries */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ESP-IDF */
#include "hal/uart_types.h"
#include "soc/gpio_num.h"

/* Public definitions - Size of a 1-Wire ROM code in bytes */
#define ONE_WIRE_ROM_CODE_SIZE (8U)

/* Size of 1-Wire ROM code when rendered as string */
#define ONE_WIRE_ROM_CODE_STR_BUF_SIZE ((2U * ONE_WIRE_ROM_CODE_SIZE) + 1U)

/* Public types ROM code */
typedef struct
{
	uint8_t abyData[ONE_WIRE_ROM_CODE_SIZE];

} stONE_WIRE_ROM_Code_t;

/* Public functions */

/**
 * @brief Initialize 1-Wire bus on specified UART and GPIO pin
 *
 * @param eUART UART instance to use for 1-Wire communication
 * @param eGPIONum GPIO pin number to use for 1-Wire data line
 */
void ONE_WIRE_Init(uart_port_t eUART, gpio_num_t eGPIONum);

/**
 * @brief Deinitialize 1-Wire bus on specified UART and GPIO pin
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param eGPIONum GPIO pin number used for 1-Wire data line
 */
void ONE_WIRE_Deinit(uart_port_t eUART, gpio_num_t eGPIONum);

/**
 * @brief Send 1-Wire reset signal and check for device presence
 *
 * @param eUART UART instance used for 1-Wire communication
 *
 * @return TRUE if one or more devices are present on the bus, FALSE otherwise
 */
bool ONE_WIRE_Reset(uart_port_t eUART);

/**
 * @brief Write a single bit to the 1-Wire bus
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param uiBit Bit value to write (0 or 1)
 */
void ONE_WIRE_WriteBit(uart_port_t eUART, uint8_t uiBit);

/**
 * @brief Read a single bit from the 1-Wire bus
 *
 * @param eUART UART instance used for 1-Wire communication
 *
 * @return Bit value read (0 or 1)
 */
uint8_t ONE_WIRE_ReadBit(uart_port_t eUART);

/**
 * @brief Write a single byte to the 1-Wire bus
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param uiByte Byte value to write
 */
void ONE_WIRE_WriteByte(uart_port_t eUART, uint8_t uiByte);

/**
 * @brief Read a single byte from the 1-Wire bus
 *
 * @param eUART UART instance used for 1-Wire communication
 *
 * @return Byte value read
 */
uint8_t ONE_WIRE_ReadByte(uart_port_t eUART);

/**
 * @brief Compute 8-bit CRC using Dallas/Maxim 1-Wire polynomial
 *
 * @param pbyData Pointer to data buffer
 * @param uiLength Length of data buffer in bytes
 *
 * @return Computed CRC8 value
 */
uint8_t ONE_WIRE_CRC8(const uint8_t *pbyData, size_t uiLength);

/**
 * @brief Search for 1-Wire devices on the bus
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param pstROMCode Pointer to buffer to store found ROM code
 * @param puiLastDiscrepancy Pointer to last discrepancy position, should be initialised to zero on first call (updated by function)
 *
 * @return TRUE if a device was found, FALSE otherwise
 */
bool ONE_WIRE_Search(uart_port_t eUART, stONE_WIRE_ROM_Code_t *pstROMCode, uint8_t *puiLastDiscrepancy);

/**
 * @brief Issue Match ROM command to select a specific device
 *
 * @param eUART UART instance used for 1-Wire communication
 * @param pstROMCode Pointer to ROM code of the device to select
 */
void ONE_WIRE_MatchROM(uart_port_t eUART, const stONE_WIRE_ROM_Code_t *pstROMCode);

/**
 * @brief Issue Skip ROM command to address all devices on the bus
 *
 * @param eUART UART instance used for 1-Wire communication
 */
void ONE_WIRE_SkipROM(uart_port_t eUART);

/**
 * @brief Convert ROM code to string representation
 *
 * @param pstROMCode Pointer to ROM code structure
 * @param pcBuffer Pointer to buffer to store string (must be at least ONE_WIRE_ROM_CODE_STR_BUF_SIZE bytes)
 * @param uiBufferSize Size of provided buffer in bytes
 *
 * @return TRUE if conversion was successful, FALSE otherwise
 */
bool ONE_WIRE_ROMCodeToStr(const stONE_WIRE_ROM_Code_t *pstROMCode, char *pcBuffer, size_t uiBufferSize);

/**
 * @brief Get family code from ROM code
 *
 * @param pstROMCode Pointer to ROM code structure
 *
 * @return Family code byte (or 0xFF on error)
 */
uint8_t ONE_WIRE_GetFamilyCode(const stONE_WIRE_ROM_Code_t *pstROMCode);

#endif
