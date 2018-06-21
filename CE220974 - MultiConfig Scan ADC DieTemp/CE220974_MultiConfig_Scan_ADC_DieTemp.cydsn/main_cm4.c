/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description:  This example demonstrates a Scan_ADC with the DieTemp sensor 
*               using low-level PDL function calls in the PSoC 6 MCU.
*
* Related Document: CE220974.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit or 
*                      CY8CKIT-062 PSoC 6 Pioneer Kit
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
#include <project.h>
#include <stdio.h>
#include "sar/cy_sar.h"
#include "sysanalog/cy_sysanalog.h"
#include "mcwdt/cy_mcwdt.h"
#include "sysclk/cy_sysclk.h"

/*******************************************************************************
* Constants for configuring the SAR ADC clock and interrupt priority.
*******************************************************************************/
/* The ADC clock is an integer divider of the PeriClk.
*
* The maximum supported clock frequency for the SAR is 18 MHz.
* With a PeriClk of 50 MHz, the minimum target clock divider is 3, 
* SAR Clock = 16.67 MHz 
*/
#define SAR_TARGET_CLK_DIVIDER      (3u)
/* Set lowest priority for the SAR interrupt. */
#define SAR_IRQ_PRIORITY            (7u)

/*******************************************************************************
* Constants for configuring Config0 of the SAR ADC
*******************************************************************************/
/* Enable channels 0 and 1. */
#define CONFIG0_CHAN_EN             (3u)

/* Channel 0 is a differential channel.
* Aperture time is set by Sample Time 0.
* Positive terminal is connected to Pin 0 of the SARMUX dedicated port.
* Negative terminal is connected to Pin 1 of the SARMUX dedicated port (input differential pair).
* Averaging is disabled. */
#define CONFIG0_CHAN0_CONFIG        (CY_SAR_CHAN_DIFFERENTIAL_PAIRED \
                                    | CY_SAR_CHAN_SAMPLE_TIME_0 \
                                    | CY_SAR_POS_PORT_ADDR_SARMUX \
                                    | CY_SAR_CHAN_POS_PIN_ADDR_0 \
                                    | CY_SAR_CHAN_AVG_DISABLE)
                            
/* Channel 1 is a differential channel.
* Aperture time is set by Sample Time 1.
* Positive terminal is connected to Pin 2 of the SARMUX dedicated port.
* Negative terminal is connected to Pin 3 of the SARMUX dedicated port (input differential pair).
* Averaging is disabled. */
#define CONFIG0_CHAN1_CONFIG        (CY_SAR_CHAN_DIFFERENTIAL_PAIRED \
                                    | CY_SAR_CHAN_SAMPLE_TIME_1 \
                                    | CY_SAR_POS_PORT_ADDR_SARMUX \
                                    | CY_SAR_CHAN_POS_PIN_ADDR_2 \
                                    | CY_SAR_CHAN_AVG_DISABLE)

/* Set differential channels to use signed format. */
#define CONFIG0_SAMPLE_CTRL         (CY_SAR_DIFFERENTIAL_SIGNED)

/* Channels 2 through 15 are unconfigured. */
#define CONFIG0_CHAN_CONFIG         {(uint32_t)CONFIG0_CHAN0_CONFIG \
                                    , (uint32_t)CONFIG0_CHAN1_CONFIG \
                                    , 0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL}
                                
/* Enable the End of Scan interrupt only. */
#define CONFIG0_INTR_MASK           (CY_SAR_INTR_EOS_MASK)
                                
/* Use the internal 1.2 V bandgap reference for the SAR reference source.
* Enable the bypass capacitor connection. */
#define CONFIG0_VREF_MV_VALUE       (1200uL)
#define CONFIG0_CTRL                (CY_SAR_VREF_SEL_BGR \
                                    | CY_SAR_BYPASS_CAP_ENABLE)
                            
/* Set the aperture times to target a 50 ksps scan rate.
* Recall that the SAR clock is 16.67 MHz.
* Sample Time 0 is set to 302 clock cycles.
* Sample Time 1 is set to 4 clock cycles. */
#define CONFIG0_SAMPLE_TIME01       ((302 << CY_SAR_SAMPLE_TIME0_SHIFT) \
                                    | (4 << CY_SAR_SAMPLE_TIME1_SHIFT))

/* Set the unused sample times 2 and 3 to be 4 clock cycles.
* Note that these two sample times are not used by any channels and only shown for reference */
#define CONFIG0_SAMPLE_TIME23       ((4 << CY_SAR_SAMPLE_TIME2_SHIFT) \
                                    | (4 << CY_SAR_SAMPLE_TIME3_SHIFT))

/* Set the initial state of switches.
* Close the switch between Pin 0 of the SARMUX to the positive terminal of the SAR (used by channel 0).
* Close the switch between Pin 1 of the SARMUX to the negative terminal of the SAR (used by channel 0).
* Close the switch between Pin 2 of the SARMUX to the positive terminal of the SAR (used by channel 1).
* Close the switch between Pin 3 of the SARMUX to the negative terminal of the SAR (used by channel 1). */
#define CONFIG0_MUX_SWITCH0         (CY_SAR_MUX_FW_P0_VPLUS \
                                    | CY_SAR_MUX_FW_P1_VMINUS \
                                    | CY_SAR_MUX_FW_P2_VPLUS \
                                    | CY_SAR_MUX_FW_P3_VMINUS)

/* Enable sequencer control of the SARMUX Pins 0 to 3 used by channel 0 and channel 1. */
#define CONFIG0_MUX_SWITCH_SQ_CTRL  (CY_SAR_MUX_SQ_CTRL_P0 \
                                    | CY_SAR_MUX_SQ_CTRL_P1 \
                                    | CY_SAR_MUX_SQ_CTRL_P2 \
                                    | CY_SAR_MUX_SQ_CTRL_P3)

/* Define the initialization structure for Config1. */
const cy_stc_sar_config_t config0 =
{
    .ctrl               = (uint32_t)CONFIG0_CTRL,
    .sampleCtrl         = (uint32_t)CONFIG0_SAMPLE_CTRL,
    .sampleTime01       = CONFIG0_SAMPLE_TIME01,
    .sampleTime23       = CONFIG0_SAMPLE_TIME23,
    .rangeThres         = CY_SAR_DEINIT,
    .rangeCond          = CY_SAR_RANGE_COND_BELOW,
    .chanEn             = CONFIG0_CHAN_EN,
    .chanConfig         = CONFIG0_CHAN_CONFIG,
    .intrMask           = CONFIG0_INTR_MASK,         
    .satIntrMask        = CY_SAR_DEINIT,                /* Disable the saturation interrupt. */
    .rangeIntrMask      = CY_SAR_DEINIT,                /* Disable the range interrupt. */
    .muxSwitch          = CONFIG0_MUX_SWITCH0,
    .muxSwitchSqCtrl    = CONFIG0_MUX_SWITCH_SQ_CTRL,
    .configRouting      = true,
    .vrefMvValue        = CONFIG0_VREF_MV_VALUE,
};

/*******************************************************************************
* Constants for configuring Config1 of the Scan_ADC
*******************************************************************************/
/* Enable only channel 0. */
#define CONFIG1_CHAN_EN             (1u)

/* Channel 0 is single ended.
* Aperture time is set by Sample Time 0.
* The DieTemp sensor is connected on the SARMUX_VIRT port at PIN_ADDR 0.
* The sensor is enabled once connection to the SAR ADC is made.
* Averaging is enabled. */
#define CONFIG1_CHAN0_CONFIG        (CY_SAR_CHAN_SINGLE_ENDED \
                                    | CY_SAR_CHAN_SAMPLE_TIME_0 \
                                    | CY_SAR_POS_PORT_ADDR_SARMUX_VIRT \
                                    | CY_SAR_CHAN_POS_PIN_ADDR_0 \
                                    | CY_SAR_CHAN_AVG_ENABLE)
                            
/* Single ended channels are signed.
* Averaging mode is set to sequential fixed with 32 samples of averaging. */
#define CONFIG1_SAMPLE_CTRL         (CY_SAR_SINGLE_ENDED_SIGNED \
                                    | CY_SAR_AVG_CNT_32 \
                                    | CY_SAR_AVG_MODE_SEQUENTIAL_FIXED)

/* Channels 1 through 15 are unconfigured. */
#define CONFIG1_CHAN_CONFIG         {(uint32_t)CONFIG1_CHAN0_CONFIG \
                                    , 0uL, 0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL,0uL}
                                
/* Enable the End of Scan interrupt only. */
#define CONFIG1_INTR_MASK           (CY_SAR_INTR_EOS_MASK)
                                
/* Use the internal 1.2 V bandgap reference for the SAR reference source.
* Enable the bypass capacitor connection. 
* Connect the negative terminal for single ended channels to VSSA. */
#define CONFIG1_VREF_MV_VALUE       (1200uL)
#define CONFIG1_CTRL                (CY_SAR_VREF_SEL_BGR \
                                    | CY_SAR_BYPASS_CAP_ENABLE \
                                    | CY_SAR_NEG_SEL_VSSA_KELVIN)

/* Config1 will operate in single shot mode.
* Set the sample time to meet the DieTemp settling time requirement of 1 us.
* With a 16.67 MHz SAR clock, 17 cycles (or a value of 18 in the register)
* gives an aperture time of 1.02 us. */
#define CONFIG1_SAMPLE_TIME01       ((18 << CY_SAR_SAMPLE_TIME0_SHIFT) \
                                    | (4 << CY_SAR_SAMPLE_TIME1_SHIFT))

/* Set the sample times for 2 and 3 to be 4 clock cycles.
* Note that these two sample times are not used by any channels and only shown for reference. */
#define CONFIG1_SAMPLE_TIME23       ((4 << CY_SAR_SAMPLE_TIME2_SHIFT) \
                                    | (4 << CY_SAR_SAMPLE_TIME3_SHIFT))

/* Set the initial state of switches.
* Close the switch between the DieTemp sensor and the positive terminal of the SAR (TEMP_VPLUS).
* Close the switch between VSSA and the negative terminal of the SAR (VSSA_VMINUS). */
#define CONFIG1_MUX_SWITCH0         (CY_SAR_MUX_FW_VSSA_VMINUS \
                                    | CY_SAR_MUX_FW_TEMP_VPLUS)

/* Enable sequencer control for the VSSA and TEMP switches.
* While unnecessary in this design because there is only one channel in Config1, 
* the code is provided for reference for designs with multiple channels. */
#define CONFIG1_MUX_SWITCH_SQ_CTRL  (CY_SAR_MUX_SQ_CTRL_VSSA \
                                    | CY_SAR_MUX_SQ_CTRL_TEMP)

/* Define the initialization structure for Config1. */
const cy_stc_sar_config_t config1 =
{
    .ctrl               = (uint32_t)CONFIG1_CTRL,
    .sampleCtrl         = (uint32_t)CONFIG1_SAMPLE_CTRL,
    .sampleTime01       = CONFIG1_SAMPLE_TIME01,
    .sampleTime23       = CONFIG1_SAMPLE_TIME23,
    .rangeThres         = CY_SAR_DEINIT,
    .rangeCond          = CY_SAR_RANGE_COND_BELOW,                
    .chanEn             = CONFIG1_CHAN_EN,
    .chanConfig         = CONFIG1_CHAN_CONFIG,
    .intrMask           = CONFIG1_INTR_MASK,         
    .satIntrMask        = CY_SAR_DEINIT,                /* Disable the saturation interrupt. */
    .rangeIntrMask      = CY_SAR_DEINIT,                /* Disable the range interrupt. */
    .muxSwitch          = CONFIG1_MUX_SWITCH0,
    .muxSwitchSqCtrl    = CONFIG1_MUX_SWITCH_SQ_CTRL,
    .configRouting      = true,
    .vrefMvValue        = CONFIG1_VREF_MV_VALUE,
};

/* Pointer to the active configuration structure, initialized to Config0. *
*  Firmware uses this pointer to switch configuration of the SAR */
const cy_stc_sar_config_t *activeConfig = &config0;

/* Configuration structure for the SAR interrupt. */
const cy_stc_sysint_t SAR_IRQ_cfg = {
    .intrSrc        	= pass_interrupt_sar_IRQn,
    .intrPriority   	= SAR_IRQ_PRIORITY
};

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void SAR_Interrupt(void);
void MCWDT_Interrupt(void);
int32_t DieTemp_CountsTo_Celsius(int16_t adcCounts);


/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Initializes UART and SAR ADC
*   2. Starts UART, SAR ADC, and Watchdog Timer
*   3. Transmit 2 ADC channel values and DieTemp each second over UART
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

    /* Enable analog reference block needed by the SAR. */
    Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);
    Cy_SysAnalog_Enable();
    
    /* Configure the clock for the UART to generate 115,200 bps. */
	Cy_SysClk_PeriphAssignDivider(PCLK_SCB5_CLOCK, CY_SYSCLK_DIV_8_BIT, 0u);
	Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 0u, 35u);
	Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 0u);
    
    /* Configure the clock for the SAR for a 16.67 MHz clock frequency. */
	Cy_SysClk_PeriphAssignDivider(PCLK_PASS_CLOCK_SAR, CY_SYSCLK_DIV_8_BIT, 1u);
	Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 1u, SAR_TARGET_CLK_DIVIDER - 1u);
	Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 1u);
    
    /* Configure and enable the SAR interrupt. */
    (void)Cy_SysInt_Init(&SAR_IRQ_cfg, SAR_Interrupt);
    NVIC_EnableIRQ(SAR_IRQ_cfg.intrSrc);

    /* Start the UART component. */
    UART_Start();

    /* Initialize and enable the SAR to Config0. */
    Cy_SAR_Init(SAR, activeConfig);
    Cy_SAR_Enable(SAR);
    
    /* Initiate continuous conversions. */
    Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);

    /* Configure and enable the watchdog timer interrupt. */
    Cy_SysInt_Init(&SysInt_1_cfg, MCWDT_Interrupt);
    NVIC_EnableIRQ(srss_interrupt_mcwdt_0_IRQn);
        
    /* Start the watchdog timer component. */
    MCWDT_Start();
    
    for(;;)
    { 
    }
}


/*******************************************************************************
* Function Name: SAR_Interrupt
********************************************************************************
*
*  Summary:
*  When an End of Scan (EOS) interrupt occurs, the ADC results for each channel
*  of the active config will be retrieved and stored into an array.
*  The results will be transmitted over UART at 1 second intervals. 
*    
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void SAR_Interrupt(void)
{
    uint32_t intrStatus = 0u;
    /* An array to store the channel results, two channels for Config0 and 1 channel for Config1 */
    static int16_t chanResults[3];
    /* String to transmit over UART. */
    char uartString[200];
    
    /* Read interrupt status register. */
    intrStatus = Cy_SAR_GetInterruptStatus(SAR);
    
    /* Check for the EOS interrupt. */
    if ((intrStatus & CY_SAR_INTR_EOS_MASK) == CY_SAR_INTR_EOS_MASK)
    {
        /* Clear handled interrupt. */
        Cy_SAR_ClearInterrupt(SAR, intrStatus);
        
        if (activeConfig == &config0)
        {
            /* Get the ADC results for both channels in Config0. */
            chanResults[0] = Cy_SAR_CountsTo_mVolts(SAR, 0, Cy_SAR_GetResult16(SAR, 0));
            chanResults[1] = Cy_SAR_CountsTo_mVolts(SAR, 1, Cy_SAR_GetResult16(SAR, 1));
        }
        else if (activeConfig == &config1)
        {
            /* Get the ADC result for the DieTemp sensor in Config1. 
            * Convert the ADC results to degrees Celsius. */
            chanResults[2] = (int16_t) DieTemp_CountsTo_Celsius(Cy_SAR_GetResult16(SAR, 0));
            
            /* Switch to Config0 and initiate continuous conversions. */
            activeConfig = &config0;
            Cy_SAR_StopConvert(SAR);
            Cy_SAR_DeInit(SAR, true);
            Cy_SAR_Init(SAR, activeConfig);
            Cy_SAR_Enable(SAR);
            Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_CONTINUOUS);
            
            /* Transmit results over UART. */
            sprintf(uartString, "Config0 Channel 0 = %hd mV\r\nConfig0 Channel 1 = %hd mV\r\nConfig1 DieTemp   = %hd degC\r\n\r\n",
                chanResults[0], chanResults[1], chanResults[2]);
            Cy_SCB_UART_PutString(UART_HW, uartString);
        }
        
    }
}

/*******************************************************************************
* Function Name: MCWDT_Interrupt
********************************************************************************
*
*  Summary:
*  When the watchdog timer interrupt occurs, a single scan of the DieTemp 
*  sensor is performed. The SAR will hardware average over 32 samples in this 
*  single scan. The red LED on the pioneer kit will be toggled as a visual cue.
*    
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void MCWDT_Interrupt(void)
{
    uint32_t mcwdtIsrMask;
    
    mcwdtIsrMask = Cy_MCWDT_GetInterruptStatus(MCWDT_HW);
    
    /* Confirm Counter0 is the interrupt source as this IRQ is shared. Not required 
	*  if only one source enabled but shown here as an example*/
    if(0u != (CY_MCWDT_CTR0 & mcwdtIsrMask))
    {
        Cy_MCWDT_ClearInterrupt(MCWDT_HW, CY_MCWDT_CTR0);
        
        /* Toggle the red LED on the pioneer kit as a visual cue. */
        Cy_GPIO_Inv(LED_Red_P0_3_PORT, LED_Red_P0_3_NUM);
        
        /* Switch to Config1 to scan the DieTemp sensor. */
        activeConfig = &config1;
        Cy_SAR_StopConvert(SAR);
        Cy_SAR_DeInit(SAR, true);
        Cy_SAR_Init(SAR, activeConfig);
        Cy_SAR_Enable(SAR);
        
        /* Initiate a single conversion. */
        Cy_SAR_StartConvert(SAR, CY_SAR_START_CONVERT_SINGLE_SHOT);
    }
}


/*******************************************************************************
* Function Name: DieTemp_CountsTo_Celsius
********************************************************************************
*
*  Summary:
*  Function to convert ADC counts to degrees Celsius. For details on operation 
*  please see the Die Temperature (DieTemp) Component datasheet.
*    
*  Parameters:
*  int16_t adcCounts - ADC counts for DieTemp scan from ADC.
*
*  Return:
*  int32_t - Temperature in whole degrees Celsius.
*
**********************************************************************************/
/*******************************************************************************
* Constants used to convert ADC counts to degrees Celsius
* for the DieTemp sensor.
*******************************************************************************/
#define DieTemp_SAR_TEMP_OFFSET_SHIFT      (10u)
#define DieTemp_SAR_TEMP_OFFSET_MULT       (0x400)
#define DieTemp_SAR_TEMP_OFFSET_DIVIDER    (0x10000)
#define DieTemp_SAR_TEMP_SHIFT             (16u)
#define DieTemp_SAR_TEMP_DIVIDER           (0x10000)
#define DieTemp_SCALE_ADJUSTMENT_DIVIDER   (16u)
#define DieTemp_HALF_OF_ONE                ((int32)1u << (DieTemp_SAR_TEMP_SHIFT - 1u))

/* (effectively 0.5 << 4u) 0.5 in Q28.4 format */
#define DieTemp_SCALE_ADJUSTMENT           (8)
/* 15 in Q16.16 format */
#define DieTemp_DUAL_SLOPE_CORRECTION      (0xF0000)
/* 100 in Q16.16 format */
#define DieTemp_HIGH_TEMPERATURE           (0x640000)
/* 40 in Q16.16 format */
#define DieTemp_LOW_TEMPERATURE            (0x280000)

int32_t DieTemp_CountsTo_Celsius(int16_t adcCounts)
{
    int32_t tempCelsius;
    int32_t tInitial;
    int32_t tAdjust;
    int32_t offsetReg;
    int32_t multReg;

    offsetReg = (int16_t)SFLASH->SAR_TEMP_OFFSET;
    multReg   = (int16_t)SFLASH->SAR_TEMP_MULTIPLIER;
    
    /* Calculate tInitial in Q16.16 */
    tInitial = (adcCounts * multReg) + (offsetReg * DieTemp_SAR_TEMP_OFFSET_MULT);

    if(tInitial >= DieTemp_DUAL_SLOPE_CORRECTION)
    {
        /* Shift (100 - tInitial) by 4 bits to prevent scale-adjustment from overflowing. */
        /* Then divide by the integer bits of (100 - cutoff) to end up with a Q16.16 tAdjust */
        tAdjust = (DieTemp_SCALE_ADJUSTMENT * (((int32)DieTemp_HIGH_TEMPERATURE - tInitial)
            / (int32)DieTemp_SCALE_ADJUSTMENT_DIVIDER)) /
            (((int32)DieTemp_HIGH_TEMPERATURE - (int32)DieTemp_DUAL_SLOPE_CORRECTION) /
            DieTemp_SAR_TEMP_DIVIDER);
    }
    else
    {
        /* Shift (40 + tInitial) by 4 bits to prevent scale-adjustment from overflowing. */
        /* Then divide by the integer bits of (40 + cutoff) to end up with a Q16.16 tAdjust */
        tAdjust = ((int32)DieTemp_SCALE_ADJUSTMENT * (((int32)DieTemp_LOW_TEMPERATURE + tInitial)
           / (int32)DieTemp_SCALE_ADJUSTMENT_DIVIDER)) /
            (((int32)DieTemp_LOW_TEMPERATURE + (int32)DieTemp_DUAL_SLOPE_CORRECTION) /
            (int32)DieTemp_SAR_TEMP_DIVIDER);
    }

    /* Add tInitial + tAdjust + 0.5 to round to nearest int. Shift off frac bits, and return. */
    tempCelsius = tInitial + tAdjust + DieTemp_HALF_OF_ONE;

    return (tempCelsius / DieTemp_SAR_TEMP_OFFSET_DIVIDER);
}

/* [] END OF FILE */
