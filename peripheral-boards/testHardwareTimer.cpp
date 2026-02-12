#include "mbed.h"

// Define the pin and timer (check datasheet for AF mapping)
// PA_5 often maps to TIM2_CH1 or similar on G4 series
PinName pinName = PA_5;
TIM_HandleTypeDef htim2;

void initHardwareCounter() {
    // 1. Enable Clocks
    HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // 2. Configure GPIO PA_5 for Alternate Function
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // Or GPIO_MODE_AF_INPUT
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2; // AF1 is usually TIM2 for PA5
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // 3. Initialize Timer as Counter
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFFFFFF; // Max period for 32-bit timer
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim2);

    // 4. Set Timer to External Clock Mode 1 (counting on pin transitions)
    // Or Configure as Input Capture if measuring pulse width
    HAL_TIM_Base_Start(&htim2);
}

int main() {
    initHardwareCounter();
    while (1) {
        // Read counter value
        uint32_t count = __HAL_TIM_GET_COUNTER(&htim2);
        printf("Counter: %lu\r\n", count);
        thread_sleep_for(1000);
    }
}
