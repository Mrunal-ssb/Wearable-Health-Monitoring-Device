#include "ait1001.h"

/**
 * @brief  Initializes the AIT1001 sensor handle.
 * @param  dev: Pointer to the AIT1001 handle structure.
 * @param  hi2c: Pointer to the I2C peripheral handle.
 * @retval HAL status.
 */
HAL_StatusTypeDef AIT1001_Init(AIT1001_Handle_t *dev, I2C_HandleTypeDef *hi2c)
{
    if (dev == NULL || hi2c == NULL) {
        return HAL_ERROR;
    }

    dev->hi2c = hi2c;
    dev->object_temp = 0.0f;
    dev->ambient_temp = 0.0f;

    // Check if the device is ready
    return HAL_I2C_IsDeviceReady(dev->hi2c, AIT1001_I2C_ADDR, 2, 100);
}

/**
 * @brief  Reads both the object and ambient temperature from the sensor.
 * @note   This function reads 6 bytes starting from the NTC register:
 * [Tntc_H, Tntc_L, Tntc_CRC, Tobj_H, Tobj_L, Tobj_CRC]
 * @param  dev: Pointer to the AIT1001 handle structure.
 * @retval HAL status.
 */
HAL_StatusTypeDef AIT1001_ReadTemperatures(AIT1001_Handle_t *dev)
{
    uint8_t reg_addr = AIT1001_REG_TNTC;
    uint8_t read_buffer[6];
    HAL_StatusTypeDef status;

    // Step 1: Write the starting register address (0x00 for NTC temp)
    status = HAL_I2C_Master_Transmit(dev->hi2c, AIT1001_I2C_ADDR, &reg_addr, 1, 100);
    if (status != HAL_OK) {
        return status;
    }

    // A short delay might be needed between write and read operations for some sensors.
    HAL_Delay(5);

    // Step 2: Read 6 bytes of data (NTC temp, NTC CRC, OBJ temp, OBJ CRC)
    status = HAL_I2C_Master_Receive(dev->hi2c, AIT1001_I2C_ADDR, read_buffer, 6, 100);
    if (status != HAL_OK) {
        return status;
    }

    // --- Process Ambient (NTC) Temperature ---
    uint8_t ntc_data[2] = {read_buffer[0], read_buffer[1]};
    uint8_t ntc_crc = read_buffer[2];

    // Verify CRC for ambient temperature data
    if (AIT1001_CalculateCRC(ntc_data, 2) == ntc_crc) {
        int16_t raw_ambient = (int16_t)((ntc_data[0] << 8) | ntc_data[1]);
        dev->ambient_temp = raw_ambient / 10.0f;
    } else {
        // CRC error, you might want to handle this (e.g., return an error, use old value)
        // For simplicity, we'll just not update the value.
    }

    // --- Process Object Temperature ---
    uint8_t obj_data[2] = {read_buffer[3], read_buffer[4]};
    uint8_t obj_crc = read_buffer[5];

    // Verify CRC for object temperature data
    if (AIT1001_CalculateCRC(obj_data, 2) == obj_crc) {
        int16_t raw_object = (int16_t)((obj_data[0] << 8) | obj_data[1]);
        dev->object_temp = raw_object / 10.0f;
    } else {
        // CRC error
    }

    return HAL_OK;
}

/**
 * @brief  Calculates CRC-8 for data integrity check.
 * @note   This function is directly from the AIT1001 datasheet.
 * Polynomial: 0x31 (x^8 + x^5 + x^4 + 1)
 * @param  pdat: Pointer to the data buffer.
 * @param  len: Length of the data in bytes.
 * @retval Calculated 8-bit CRC value.
 */
uint8_t AIT1001_CalculateCRC(uint8_t *pdat, uint8_t len)
{
    uint8_t bits, byte, crc = 0xFF; // Initial value is 0xFF

    for(byte = 0; byte < len; byte++)
    {
        crc ^= (*pdat);
        for(bits = 0; bits < 8; bits++)
        {
            if(crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc = (crc << 1);
            }
        }
        pdat++;
    }
    return crc;
}

