#include "stm32f4xx.h"

// Jack Nystrom (3/30/2026):
// Override the SetSysClock_PLL_HSE (from mbed-os/targets/TARGET_STM/TARGET_STM32F4/TARGET_STM32F446xE/system_clock.c)
// to use the correct PLL values for our 24MHz crystal. The default stm32f4xx's
// system_clock.c assumes an 8MHz crystal so we must override that here.
uint8_t SetSysClock_PLL_HSE(uint8_t bypass)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
#if DEVICE_USBDEVICE
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
#endif

    /* The voltage scaling allows optimizing the power consumption when the device is
       clocked below the maximum system frequency, to update the voltage scaling value
       regarding system frequency refer to product datasheet. */
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /* Get the Clocks configuration according to the internal RCC registers */
    HAL_RCC_GetOscConfig(&RCC_OscInitStruct);

    /* PLL could be already configured by bootlader */
    if (RCC_OscInitStruct.PLL.PLLState != RCC_PLL_ON) {

        // Enable HSE oscillator and activate PLL with HSE as source
        RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSE;
        if (bypass == 0) {
            RCC_OscInitStruct.HSEState          = RCC_HSE_ON; // External 8 MHz xtal on OSC_IN/OSC_OUT
        } else {
            RCC_OscInitStruct.HSEState          = RCC_HSE_BYPASS; // External 8 MHz clock on OSC_IN
        }

        RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
        RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
        RCC_OscInitStruct.PLL.PLLM = 12;            // VCO input clock = 2 MHz (24 MHz / 12)
        RCC_OscInitStruct.PLL.PLLN = 180;           // VCO output clock = 360 MHz (2 MHz * 180)
        RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLCLK = 180 MHz (360 MHz / 2)
        RCC_OscInitStruct.PLL.PLLQ = 2;             //
        RCC_OscInitStruct.PLL.PLLR = 2;             //
        if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
            return 0; // FAIL
        }
    }

    // Activate the OverDrive to reach the 180 MHz Frequency
    if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
        return 0; // FAIL
    }

#if DEVICE_USBDEVICE
    // Select PLLSAI output as USB clock source
    PeriphClkInitStruct.PLLSAI.PLLSAIM = 8;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLSAIP;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
#endif /* DEVICE_USBDEVICE */

    // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // 180 MHz
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  //  45 MHz
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  //  90 MHz
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        return 0; // FAIL
    }

    // Output clock on MCO1 pin(PA8) for debugging purpose
#if DEBUG_MCO == 1
    if (bypass == 0) {
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_2);
    } else {
        HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSE, RCC_MCODIV_1);
    }
#endif

    return 1; // OK
}
