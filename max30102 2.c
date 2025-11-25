/*
 * max30102.c
 *
 *
 * Basic driver for the MAX30102 Pulse Oximeter and Heart-Rate Sensor.
 * Target: STM32F405xx using HAL library.
 */

#include "max30102.h"

uint32_t ir_buffer[BUFFER_SIZE];
uint32_t red_buffer[BUFFER_SIZE];
volatile int32_t buffer_index = 0;

void add_sample_to_buffer(uint32_t red_val, uint32_t ir_val) {
    red_buffer[buffer_index] = red_val;
    ir_buffer[buffer_index] = ir_val;
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
}

// Helper function to write a byte to a specific register
static HAL_StatusTypeDef max30102_write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, MAX30102_I2C_ADDR_WR, reg, 1, &value, 1, HAL_MAX_DELAY);
}

// Helper function to read a byte from a specific register
static HAL_StatusTypeDef max30102_read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value) {
    return HAL_I2C_Mem_Read(hi2c, MAX30102_I2C_ADDR_WR, reg, 1, value, 1, HAL_MAX_DELAY);
}

/**
 * @brief Performs a soft reset of the sensor.
 * @param hi2c Pointer to a I2C_HandleTypeDef structure.
 * @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef max30102_reset(I2C_HandleTypeDef *hi2c) {
    // Set the RESET bit in the mode configuration register
    if (max30102_write_reg(hi2c, REG_MODE_CONFIG, 0x40) != HAL_OK) {
        return HAL_ERROR;
    }

    // Wait for the reset bit to clear
    uint8_t temp_reg = 0;
    do {
        HAL_Delay(10); // Wait for reset to complete
        if (max30102_read_reg(hi2c, REG_MODE_CONFIG, &temp_reg) != HAL_OK) {
            return HAL_ERROR;
        }
    } while ((temp_reg & 0x40) != 0);

    return HAL_OK;
}


/**
 * @brief Initializes the MAX30102 sensor.
 * @param hi2c Pointer to a I2C_HandleTypeDef structure that contains
 * the configuration information for the specified I2C.
 * @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef max30102_init(I2C_HandleTypeDef *hi2c) {
    uint8_t part_id = 0;

    // First, check if we can communicate with the sensor by reading the Part ID
    if (max30102_read_reg(hi2c, REG_PART_ID, &part_id) != HAL_OK) {
        return HAL_ERROR; // Communication failed
    }
    if (part_id != 0x15) {
        return HAL_ERROR; // Incorrect device
    }

    // Reset the sensor to a known state
    if (max30102_reset(hi2c) != HAL_OK) {
        return HAL_ERROR;
    }

    // --- Configuration ---

    // Interrupt Enable 1: Enable A_FULL_EN (FIFO Almost Full)
    // An interrupt will be generated when FIFO has 1 unread sample.
    if (max30102_write_reg(hi2c, REG_INTR_ENABLE_1, 0x80) != HAL_OK) return HAL_ERROR;

    // FIFO Configuration:
    // SMP_AVE = 4 (average 4 samples)
    // FIFO_ROLLOVER_EN = 0 (rollover disabled)
    // FIFO_A_FULL = 0xF (interrupt triggers when 15 samples are in FIFO, leaving 1 space)
    // We set it to 0x0 to get an interrupt for every sample.
    if (max30102_write_reg(hi2c, REG_FIFO_CONFIG, 0x4F) != HAL_OK) return HAL_ERROR;


    // Mode Configuration: Set to SpO2 mode
    // MODE = 0x03 for SpO2 mode (Red and IR LEDs active)
    if (max30102_write_reg(hi2c, REG_MODE_CONFIG, 0x03) != HAL_OK) return HAL_ERROR;

    // SpO2 Configuration:
    // SPO2_ADC_RGE = 0x01 (4096 nA)
    // SPO2_SR = 0x01 (100 samples per second)
    // LED_PW = 0x03 (411 us pulse width, 18-bit ADC resolution)
    if (max30102_write_reg(hi2c, REG_SPO2_CONFIG, 0x27) != HAL_OK) return HAL_ERROR;

    // LED Pulse Amplitude (Current) Configuration:
    // These values are examples and should be tuned for your specific application.
    // A good starting point is around 7.6mA
    if (max30102_write_reg(hi2c, REG_LED1_PA, 0x24) != HAL_OK) return HAL_ERROR; // IR LED
    if (max30102_write_reg(hi2c, REG_LED2_PA, 0x24) != HAL_OK) return HAL_ERROR; // Red LED

    // Clear FIFO pointers
    if (max30102_write_reg(hi2c, REG_FIFO_WR_PTR, 0x00) != HAL_OK) return HAL_ERROR;
    if (max30102_write_reg(hi2c, REG_FIFO_RD_PTR, 0x00) != HAL_OK) return HAL_ERROR;
    if (max30102_write_reg(hi2c, REG_OVF_COUNTER, 0x00) != HAL_OK) return HAL_ERROR;

    return HAL_OK;
}

/**
 * @brief Reads a single sample (IR and Red values) from the FIFO.
 * @param hi2c Pointer to a I2C_HandleTypeDef structure.
 * @param red_val Pointer to a uint32_t to store the Red LED value.
 * @param ir_val Pointer to a uint32_t to store the IR LED value.
 * @retval HAL_StatusTypeDef: HAL_OK if successful, HAL_ERROR otherwise.
 */
HAL_StatusTypeDef max30102_read_fifo(I2C_HandleTypeDef *hi2c, uint32_t *red_val, uint32_t *ir_val) {
    uint8_t buffer[6];

    // Read the 6 bytes of data for one sample (3 bytes for Red, 3 for IR)
    if (HAL_I2C_Mem_Read(hi2c, MAX30102_I2C_ADDR_WR, REG_FIFO_DATA, 1, buffer, 6, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine the bytes into 32-bit integers.
    // The data is 18-bit and right-justified.
    *red_val = ((uint32_t)buffer[0] << 16) | ((uint32_t)buffer[1] << 8) | buffer[2];
    *ir_val  = ((uint32_t)buffer[3] << 16) | ((uint32_t)buffer[4] << 8) | buffer[5];

    // Mask out the unused upper bits to ensure data is purely 18-bit.
    *red_val &= 0x03FFFF;
    *ir_val  &= 0x03FFFF;

    return HAL_OK;
}
