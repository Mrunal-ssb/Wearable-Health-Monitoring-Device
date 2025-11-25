/*
 * hc05.h
 *
 * Created on: Jul 25, 2025
 * Author: Azeez
 */

#include "4X4_keypad.h"
#include "FreeRTOS.h"

// Default key layout (can be overridden per keypad)
static const char DEFAULT_KEYMAP[4][4] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

void Keypad_GPIO_Init(Keypad_TypeDef* keypad) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    for (int i = 0; i < 4; i++) {
    	// Activate COLUMN PORT CLOCKS
        switch ((uint32_t)(keypad->col_ports[i])) {
            case (uint32_t)GPIOA: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
            case (uint32_t)GPIOB: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
            case (uint32_t)GPIOC: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
            case (uint32_t)GPIOD: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        }

        // Activate COLUMN PORT CLOCKS
        switch ((uint32_t)(keypad->row_ports[i])) {
            case (uint32_t)GPIOA: __HAL_RCC_GPIOA_CLK_ENABLE(); break;
            case (uint32_t)GPIOB: __HAL_RCC_GPIOB_CLK_ENABLE(); break;
            case (uint32_t)GPIOC: __HAL_RCC_GPIOC_CLK_ENABLE(); break;
            case (uint32_t)GPIOD: __HAL_RCC_GPIOD_CLK_ENABLE(); break;
        }

        // Column Pin Configurations
        GPIO_InitStruct.Pin = keypad->col_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad->col_ports[i], &GPIO_InitStruct);

        // Row Pin Configurations
        GPIO_InitStruct.Pin = keypad->row_pins[i];
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(keypad->row_ports[i], &GPIO_InitStruct);

        // OPTIONAL: ALL ROWS TO HIGH
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }
}

// Initialize KeyPad
void Keypad_Init(Keypad_TypeDef *keypad) {

	Keypad_GPIO_Init(keypad);

    // (Optional) Copy default keymap if not already set
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (keypad->keymap[i][j] == '\0') {  // If uninitialized
                keypad->keymap[i][j] = DEFAULT_KEYMAP[i][j];
            }
        }
    }
}

int r, c, x;


// Scan a single keypad
char Keypad_Scan(Keypad_TypeDef *keypad) {
    for (int row = 0; row < 4; row++) {
        // Activate current row (set LOW)
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_RESET);

        // Small delay for signal stabilization (optional but recommended)
        HAL_Delay(1);  // 1ms debounce delay
//        vTaskDelay(pdMS_TO_TICKS(1));  // Non-blocking delay

        // Check each column
        for (int col = 0; col < 4; col++) {
        	r = row; c = col;

        	x = HAL_GPIO_ReadPin(keypad->col_ports[col], keypad->col_pins[col]);

        	if (!HAL_GPIO_ReadPin(keypad->col_ports[col], keypad->col_pins[col])) {
                // Deactivate row before returning
                HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET);
                return keypad->keymap[row][col];  // Return the correct key
            }
        }

        // Deactivate row (set LOW)
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET);
    }
    return '\0';  // No key pressed
}

