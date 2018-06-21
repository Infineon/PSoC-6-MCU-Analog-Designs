/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description:  This example demonstrates how to sample and hold the VDAC 
*				output while in Deep Sleep mode using a PSoC 6 MCU.
*
* Related Document: CE220925.pdf
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
#include "project.h"
#include "ctdac/cy_ctdac.h"
#include "ctb/cy_ctb.h"
#include "sysanalog/cy_sysanalog.h"

/* This project assumes a configuration tool will automatically configure system */
/* resources. To demonstrate how PDL drivers are used to manually configure */
/* the components, set the PDL_CONFIGURATION #define to 1, */
/* otherwise leave set to 0. */
#define PDL_CONFIGURATION   (0u)

/* Set the VDAC output to be slightly above the internal 1.20 V bandgap.
* The actual value required will depend on VDDA. */
#define DAC_VALUE			(1210u) /* 1.21 volts */

/* Constant to check if comparator interrupt was triggered. */
#define COMP_NOT_TRIGGERED  (0u)

void SampleAndHold(void);

void Comparator_Interrupt(void);

#if PDL_CONFIGURATION
    #define CTB_INTR_PRIORITY   7
    
	/* Comparator interrupt initialization structure. */
	const cy_stc_sysint_t Comparator_IRQ_cfg = {
		.intrSrc 		= pass_interrupt_ctbs_IRQn,
		.intrPriority 	= CTB_INTR_PRIORITY
	};
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Initializes and enable VDAC and Comparator Components
*   2. Configure the Comparator interrupt
*   3. Sample and hold the VDAC output
*   4. Put the device into Deep Sleep mode
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
	int32_t dacValue = DAC_VALUE;
	
    /* Enable global interrupts. */
    __enable_irq();

    #if PDL_CONFIGURATION
		/* Initialize and enable the comparator interrupt. */
		(void)Cy_SysInt_Init(&Comparator_IRQ_cfg, Comparator_Interrupt);
		NVIC_EnableIRQ(Comparator_IRQ_cfg.intrSrc);
		
		/* Initialize and enable the analog reference block (AREF). */
		Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);
        Cy_SysAnalog_Enable();
		/* Enable all blocks of the AREF for Deep Sleep operation. */
		Cy_SysAnalog_SetDeepSleepMode(CY_SYSANALOG_DEEPSLEEP_IPTAT_IZTAT_VREF);

		/* Configure CTDAC for VDDA reference and buffered output. */
		Cy_CTDAC_FastInit(CTDAC0, &Cy_CTDAC_Fast_VddaRef_BufferedOut);
		
		/* Enable VDAC for Deep Sleep operation so that Sample and Hold
		 * switches in the CTBm can remain functional. */
		Cy_CTDAC_SetDeepSleepMode(CTDAC0, CY_CTDAC_DEEPSLEEP_ENABLE);
		
		/* Configure OA0 of the CTBm to be the VDAC output buffer 
		 * with the sample and hold capacitor connected.
		 * The buffered VDAC output will be on Pin 9_2. 
		 * Configure OA1 of the CTBm to be a comparator. */
		Cy_CTB_FastInit(CTBM0, &Cy_CTB_Fast_Opamp0_Vdac_Out_SH, &Cy_CTB_Fast_Opamp1_Comp);
		
		/* Configure the CTBm to use 1 uA bias current reference and 
		 * enable Deep Sleep operation.
		 * This will restrict the input range of the OpAmps and comparator to VDDA - 1.5 V. */
		Cy_CTB_SetCurrentMode(CTBM0, CY_CTB_CURRENT_HIGH_ACTIVE_DEEPSLEEP);
		
		/* Connect 1.20 V bandgap from the AREF to V+ of the comparator.
		 * Connect V- of the comparator to Pin 9_4. */
		Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_OA1_SW, CY_CTB_SW_OA1_POS_AREF_MASK | CY_CTB_SW_OA1_NEG_PIN4_MASK, CY_CTB_SWITCH_CLOSE);
		
		/* Route the 1.20 V bandgap to Pin 9_7 for measurement purposes. */
		Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_OA1_SW, CY_CTB_SW_OA1_POS_PIN7_MASK, CY_CTB_SWITCH_CLOSE);
		
		/* Disable the comparator interrupts until input voltages are stable */
		Cy_CTB_CompSetInterruptEdgeType(CTBM0, CY_CTB_OPAMP_1, CY_CTB_COMP_EDGE_DISABLE);
		Cy_CTB_SetInterruptMask(CTBM0, CY_CTB_OPAMP_NONE);
		
		/* Enable the CTBm and CTDAC hardware blocks. */
		Cy_CTB_Enable(CTBM0);
		Cy_CTDAC_Enable(CTDAC0);

		/* Set the VDAC output to be slightly above the 1.20 V bandgap.
		 * The actual value is adjusted to the VDDA set in the system. */
		dacValue *= CY_CTDAC_UNSIGNED_MAX_CODE_VALUE;
		dacValue /= CYDEV_VDDA_MV;
		Cy_CTDAC_SetValueBuffered(CTDAC0, dacValue);

		/* Allow 10 us for the DAC output to update and settle. */
		Cy_SysLib_DelayUs(10);
		
		/* Enable the comparator interrupt for rising edge.
		 * When V- (VDAC output) drops between V+ (1.2 V), an interrupt will occur. */
		Cy_CTB_CompSetInterruptEdgeType(CTBM0, CY_CTB_OPAMP_1, CY_CTB_COMP_EDGE_RISING);
		Cy_CTB_SetInterruptMask(CTBM0, CY_CTB_OPAMP_1);
    #else  
		/* Initialize and enable the comparator interrupt. */
		(void)Cy_SysInt_Init(&SysInt_1_cfg, Comparator_Interrupt);
		NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);
		
		/* Start the VDAC component. */
		VDAC_1_Start();
		
		/* Set the VDAC output to be slightly above the 1.2 V bandgap.
		 * The actual value is adjusted to the VDDA set in the system. */
		dacValue *= CY_CTDAC_UNSIGNED_MAX_CODE_VALUE;
		dacValue /= CYDEV_VDDA_MV;
		VDAC_1_SetValueBuffered(dacValue);
		
		/* Allow 10 us for the DAC output to update and settle. */
		Cy_SysLib_DelayUs(10);
		
		/* Start the comparator output. */
		Comp_1_Start();
    #endif
    
    /* Perform a sample and hold before entering Deep Sleep mode. */
    SampleAndHold();
    
    for(;;)
    {       
        /* Keep device in Deep Sleep mode. 
        *  Device will wake up each comparator interrupt.
        *  A sample and hold will be executed before going back into Deep Sleep mode. */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}


/*******************************************************************************
* Function Name: SampleAndHold
********************************************************************************
*
*  Summary:
*  Perform sample and hold operation.
*    
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void SampleAndHold(void)
{
    #if PDL_CONFIGURATION  
		/* Sample VDAC output */
		Cy_CTB_DACSampleAndHold(CTBM0, CY_CTB_SH_PREPARE_SAMPLE);
		Cy_CTDAC_SetOutputMode(CTDAC0, CY_CTDAC_OUTPUT_VALUE);
		Cy_CTB_DACSampleAndHold(CTBM0, CY_CTB_SH_SAMPLE);
        
        /* Allow time for voltage across hold capcitor to settle. */
		Cy_SysLib_DelayUs(10);
			
		/* Hold voltage on SH capacitor */
		Cy_CTB_DACSampleAndHold(CTBM0, CY_CTB_SH_PREPARE_HOLD);
		Cy_CTDAC_SetOutputMode(CTDAC0, CY_CTDAC_OUTPUT_HIGHZ);
		Cy_CTB_DACSampleAndHold(CTBM0, CY_CTB_SH_HOLD);
    #else
		/* Sample VDAC output */
		VDAC_1_SetSampleAndHold(VDAC_1_SH_SAMPLE);
        
        /* Allow time for voltage across hold capcitor to settle. */
		Cy_SysLib_DelayUs(10);
        
		/* Hold voltage on SH capacitor */
		VDAC_1_SetSampleAndHold(VDAC_1_SH_HOLD);
    #endif
}

/*******************************************************************************
* Function Name: Comparator_Interrupt
********************************************************************************
*
*  Summary:
*  Interrupt service routine for the comparator interrupt.
*  When the device wakes up from Deep Sleep, toggle the red LED on the
*  kit and perform a sample and hold.
*    
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
#if PDL_CONFIGURATION
	void Comparator_Interrupt(void)
	{        
		/* This ISR will be called when the device wakes up from Deep Sleep
		 * due to a comparator interrupt. */
		uint32_t status;
		
		/* Retrieve the comparator interrupt status */
		status = Cy_CTB_GetInterruptStatus(CTBM0, CY_CTB_OPAMP_1);

		/* Check that OA1 triggered the interrupt. */
		if (COMP_NOT_TRIGGERED != status)
		{
			/* Clear the interrupt. */
			Comp_1_ClearInterrupt();
            
			/* Toggle the red LED pin as a visual cue. */
			Cy_GPIO_Inv(LED_Red_P0_3_PORT, LED_Red_P0_3_NUM);
			
			/* Perform a sample and hold before going back into Deep Sleep mode. */
			SampleAndHold();
		}
	}
#else
	void Comparator_Interrupt(void)
	{        
		/* This ISR will be called when the device wakes up from Deep Sleep
		 * due to a comparator interrupt. */
		
		/* Retrieve the comparator interrupt status. */
		uint32_t status;
		status = Comp_1_GetInterruptStatus();

		/* Check that Comp_1 triggered the interrupt. */
		if (COMP_NOT_TRIGGERED != status)
		{
			/* Clear the interrupt. */
			Comp_1_ClearInterrupt();

			/* Toggle the red LED pin as a visual cue. */
			Cy_GPIO_Inv(LED_Red_P0_3_PORT, LED_Red_P0_3_NUM);
			
			/* Perform a sample and hold before going back into Deep Sleep mode. */
			SampleAndHold();
		}
	}
#endif

/* [] END OF FILE */
