/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description: This example demonstrates how to use the Voltage DAC (12-bit)
*              component and CTDAC as a sawtooth wave generator in a PSoC 6 MCU.
*
* Related Document: CE220923.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 MCU BLE Pioneer Kit or
*                      CY8CKIT-062 PSoC 6 MCU Pioneer Kit
*
******************************************************************************
* Copyright (2017), Cypress Semiconductor Corporation.
******************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress) and is
* protected by and subject to worldwide patent protection (United States and
* foreign), United States copyright laws and international treaty provisions.
* Cypress hereby grants to licensee a personal, non-exclusive, non-transferable
* license to copy, use, modify, create derivative works of, and compile the
* Cypress Source Code and derivative works for the sole purpose of creating
* custom software in support of licensee product to be used only in conjunction
* with a Cypress integrated circuit as specified in the applicable agreement.
* Any reproduction, modification, translation, compilation, or representation of
* this software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH
* REGARD TO THIS MATERIAL, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
* Cypress reserves the right to make changes without further notice to the
* materials described herein. Cypress does not assume any liability arising out
* of the application or use of any product or circuit described herein. Cypress
* does not authorize its products for use as critical components in life-support
* systems where a malfunction or failure may reasonably be expected to result in
* significant injury to the user. The inclusion of Cypress' product in a life-
* support systems application implies that the manufacturer assumes all risk of
* such use and in doing so indemnifies Cypress against all charges. Use may be
* limited by and subject to the applicable Cypress software license agreement.
*****************************************************************************/
#include "project.h"
#include "ctdac/cy_ctdac.h"

/* This project assumes a configuration tool will automatically configure system */
/* resources, including setting the PeriClk to 50 Mhz. */
/* To demonstrate how PDL drivers are used to manually configure */
/* the CTDAC, set the PDL_CONFIGURATION #define to 1, */
/* otherwise leave set to 0. */
#define PDL_CONFIGURATION   (0u)

uint32_t currentCode = 0u;

void userIsr(void);

#if PDL_CONFIGURATION
    /* A custom config structure is used instead of one of the pre-defined fast Init config
    *  in order to set the DAC clock frequency to 50 kHz. */

    /* Set the deglitch time to the suggested 700 ns. */
    #define DEGLITCH_TARGET_TIME_NS    (700uL)
    #define FACTOR_NANO_TO_MICRO       (1000uL)
    #define DEGLITCH_TARGET_CYCLES     (((CYDEV_CLK_PERICLK__MHZ * DEGLITCH_TARGET_TIME_NS) / FACTOR_NANO_TO_MICRO) - 1uL)

    /* With a 50 MHz PeriClk, divide by 1000 to get a 50 kHz clock for the DAC. */
    #define CLKCFG_TYPE                CY_SYSCLK_DIV_16_BIT
    #define CLKCFG_NUM                 (0uL)
    #define CLKCFG_DIV                 (999uL) /* This value is one less than the desired divider value. */

    /* The interrupt source number and priority for the DAC interrupt. */
    #define INTC_NUMBER                pass_interrupt_dacs_IRQn
    #define INTC_CORTEXM4_PRIORITY     7

    /* Define the config structure for initializing the CTDAC. */
    const cy_stc_ctdac_config_t Cy_CTDAC_Config =
    {
        .refSource          = CY_CTDAC_REFSOURCE_VDDA,         /* Vdda reference. */
        .formatMode         = CY_CTDAC_FORMAT_UNSIGNED,        /* Unsigned format mode. */
        .updateMode         = CY_CTDAC_UPDATE_BUFFERED_WRITE,  /* Buffered write mode. */
        .deglitchMode       = CY_CTDAC_DEGLITCHMODE_UNBUFFERED,/* Enable deglitching on the unbuffered output path. */
        .outputMode         = CY_CTDAC_OUTPUT_VALUE,           /* DAC output is enabled and drives the programmed value. */
        .outputBuffer       = CY_CTDAC_OUTPUT_UNBUFFERED,      /* DAC output is routed to a direct pin (unbuffered). */
        .deepSleep          = CY_CTDAC_DEEPSLEEP_DISABLE,      /* Disable operation in Deep Sleep. */
        .deglitchCycles     = DEGLITCH_TARGET_CYCLES,          /* Set the deglitch time to 700 ns.  */
        .value              = 0,                               /* Set the initial code value to 0. */
        .nextValue          = 0,                               /* Set the buffered code value to 0. */
        .enableInterrupt    = true,                            /* Enable interrupts. */
        .configClock        = true,                            /* Configure the CTDAC clock. */
        .dividerType        = CLKCFG_TYPE,                     /* The CTDAC clock divider type. */
        .dividerNum         = CLKCFG_NUM,                      /* The CTDAC clock divider number. */
        .dividerIntValue    = CLKCFG_DIV,                      /* The CTDAC clock integer divider value. */
        .dividerFracValue   = CY_CTDAC_DEINIT,                 /* Fractional divider not used. */
    };

    /* Define the config for the CTDAC interrupt. */
    const cy_stc_sysint_t SysInt_CTDAC_cfg = {
        .intrSrc            = INTC_NUMBER,
        .intrPriority       = INTC_CORTEXM4_PRIORITY
    };

#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Initializes Interrupt Component
*   2. Starts the CTDAC hardware
*   3. On each interrupt, updates the CTDAC output value.
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
int main(void)
{
    /* Enable global interrupts. */
    __enable_irq();

#if PDL_CONFIGURATION
    /* Configure the interrupt and provide the ISR address. */
    (void)Cy_SysInt_Init(&SysInt_CTDAC_cfg, userIsr);

    /* Enable the interrupt */
    NVIC_EnableIRQ(SysInt_CTDAC_cfg.intrSrc);

    /* Initialize and enable the CTDAC. */
    Cy_CTDAC_Init(CTDAC0, &Cy_CTDAC_Config);
    Cy_CTDAC_Enable(CTDAC0);
#else
    /* Configure the interrupt and provide the ISR address. */
    (void)Cy_SysInt_Init(&SysInt_1_cfg, userIsr);

    /* Enable the interrupt. */
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);

    /* Start the component. */
    VDAC_1_Start();
#endif

    for(;;)
    {
    }
}

/*******************************************************************************
* Function Name: userIsr
********************************************************************************
*
*  Summary:
*  Interrupt service routine for the VDAC update.
*
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
#if PDL_CONFIGURATION
    /* This function gets called when an interrupt occurs in any CTDAC on the PSoC 6 device. */
    void userIsr(void)
    {
        uint8_t intrStatus;

        /* Check that an interrupt occurred in the desired CTDAC. */
        intrStatus = Cy_CTDAC_GetInterruptStatus(CTDAC0);
        if (intrStatus)
        {
            /* Clear the interrupt. */
            VDAC_1_ClearInterrupt();
            
            /* Set the next value that the DAC will output. */
            Cy_CTDAC_SetValueBuffered(CTDAC0, currentCode);

            /* Increment the DAC codes from 0 to the maximum code value to generate a sawtooth waveform. */
            if (currentCode >= CY_CTDAC_UNSIGNED_MAX_CODE_VALUE)
            {
                currentCode = 0u;
            }
            else
            {
                currentCode++;
            }
        }
    }
#else
    /* This function gets called when an interrupt occurs in any CTDAC on the PSoC 6 MCU device. */
    void userIsr(void)
    {
        uint8_t intrStatus;

        /* Check that an interrupt occurred in the VDAC component instance. */
        intrStatus = VDAC_1_GetInterruptStatus();
        if (intrStatus)
        {
            /* Clear the interrupt. */
            VDAC_1_ClearInterrupt();
            
            /* Set the next value that the DAC will output. */
            VDAC_1_SetValueBuffered(currentCode);

            /* Increment the DAC code from 0 to the maximum code value to generate a sawtooth waveform. */
            if (currentCode >= CY_CTDAC_UNSIGNED_MAX_CODE_VALUE)
            {
                currentCode = 0u;
            }
            else
            {
                currentCode++;
            }
        }
    }
#endif

/* [] END OF FILE */
