/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description:  This example demonstrates how to configure a basic OpAmp 
*               and Comparator using both components and low-level PDL function 
*               calls in.
*
* Related Document: CE220927.pdf
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
/* the VDAC, set the PDL_CONFIGURATION #define to 1, otherwise leave set to 0. */
#define PDL_CONFIGURATION   (0u)

#define LED_GREEN_PORT    	(GPIO_PRT1)
#define LED_GREEN_PIN     	(1u)
#define LED_ORANGE_PORT   	(GPIO_PRT1)
#define LED_ORANGE_PIN    	(5u)
#define COMP_OUT_PORT     	(GPIO_PRT9)
#define COMP_OUT_PIN      	(2u)
#define INTC_NUMBER     	pass_interrupt_ctbs_IRQn
#define INTC_CORTEXM4_PRIORITY      (7u)

/* Set the VDAC output to be 1.0 V. */
#define DAC_VALUE_MV		(1000u)

void Comparator_Interrupt(void);

#if PDL_CONFIGURATION
	/* Config structure to handle the comparator interrupt.  */
	const cy_stc_sysint_t SysInt_CTB_cfg = {
		.intrSrc			= INTC_NUMBER,
		.intrPriority    	= INTC_CORTEXM4_PRIORITY
	};
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Starts the VDAC to provide stimulus to the Comparator
*   2. Starts the OpAmp and Comparator 
*   3. Continuously update the LED to show the current Comparator state
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
    
    int32_t dacValue = DAC_VALUE_MV;

#if PDL_CONFIGURATION
    /* Clear out any connections made on Port 1 and Port 9 by Creator routing tools. */
    Cy_GPIO_Port_Deinit(GPIO_PRT1);
    Cy_GPIO_Port_Deinit(GPIO_PRT9);
    
    /* Set the drive mode for the green LED and comparator digital output. */
    Cy_GPIO_SetDrivemode(LED_GREEN_PORT, LED_GREEN_PIN, CY_GPIO_DM_STRONG_IN_OFF);
    Cy_GPIO_SetDrivemode(COMP_OUT_PORT, COMP_OUT_PIN, CY_GPIO_DM_STRONG_IN_OFF);  
    
    /* Configure the interrupt and provide the ISR address. */
    (void)Cy_SysInt_Init(&SysInt_CTB_cfg, Comparator_Interrupt);
    /* Enable the interrupt. */
    NVIC_EnableIRQ(SysInt_CTB_cfg.intrSrc);
    
    /* Initialize and enable the AREF block.
     * The CTB OpAmps require reference currents from the AREF. */
    Cy_SysAnalog_Init(&Cy_SysAnalog_Fast_Local);
    Cy_SysAnalog_Enable();

    /* ====== CTDAC config ====== */
    /* Initialize the CTDAC to use VDDA as reference and output to P9_6. */
    Cy_CTDAC_FastInit(CTDAC0, &Cy_CTDAC_Fast_VddaRef_UnbufferedOut);
    Cy_CTDAC_Enable(CTDAC0);
    
    /* Configure the CTDAC to output a constant reference voltage. */
    dacValue *= CY_CTDAC_UNSIGNED_MAX_CODE_VALUE;
    dacValue /= CYDEV_VDDA_MV;
    Cy_CTDAC_SetValueBuffered(CTDAC0, dacValue);
    
    /* ====== CTBM config ====== */
    /* Configure the OA0 as a comparator and OA1 as an opamp. */
    Cy_CTB_FastInit(CTBM0, &Cy_CTB_Fast_Opamp0_Comp, &Cy_CTB_Fast_Opamp1_Opamp10x);
    Cy_CTB_Enable(CTBM0);
    
    /* ====== OA1 Routing ====== */
    /* Close input switches for OA1.
     * Non-inverting input: Pin 9_5
     * Inverting input: Pin 9_4
     * Output: Pin 9_7 */
    Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_OA1_SW, CY_CTB_SW_OA1_NEG_PIN4_MASK | CY_CTB_SW_OA1_POS_PIN5_MASK, CY_CTB_SWITCH_CLOSE);
    /* OA1 output is routed to P9_3 directly, but P9_3 is not connected to any jumpers on the pioneer kit.
     * Connecting P9_3 to J2_P9_3 would require 0 Ohm on R131.
     * Instead we will internally route P9_3 to P9_7 using the AmuxBusB. */
    Cy_GPIO_SetHSIOM(P9_3_PORT, P9_3_NUM, P9_3_AMUXB);
    Cy_GPIO_SetHSIOM(P9_7_PORT, P9_7_NUM, P9_7_AMUXB);
    
    /* ====== CTDAC to OA0 Routing ====== */
    /* Route the output of the CTDAC to the negative terminal of the comparator (OA0).
     * This involves closing two switches, A81 and COB.*/
    /* Switch A81: Connect OA0 1x output to negative input terminal. */
    Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_OA0_SW, CY_CTB_SW_OA0_NEG_OUT_MASK, CY_CTB_SWITCH_CLOSE);
    /* Switch COB: Connect the CTDAC output to OA0 1x output. */
    Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_CTD_SW, CY_CTB_SW_CTD_OUT_OA0_1X_OUT_MASK, CY_CTB_SWITCH_CLOSE);
    
    /* ====== OA1 to OA0 Routing ====== */
    /* Route Pin 9_5, which is the non-inverting input of OA1, to the non-inverting input of the comparator (OA0). 
     * This involves closing two switches, A00 and CIS, in the CTBm IP block
     * and one switch in the HSIOM.
     * A00 and CIS connects the non-inverting input of OA0 to AMUXBUSA.
     * Using the HSIOM, we can connect Pin 9_5 to AMUX bus A. */
    Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_OA0_SW, CY_CTB_SW_OA0_POS_AMUXBUSA_MASK, CY_CTB_SWITCH_CLOSE);
    Cy_CTB_SetAnalogSwitch(CTBM0, CY_CTB_SWITCH_CTD_SW, CY_CTB_SW_CTD_CHOLD_OA0_POS_ISOLATE_MASK, CY_CTB_SWITCH_CLOSE);
    Cy_GPIO_SetHSIOM(P9_5_PORT, P9_5_NUM, P9_5_AMUXA);
    
    /* ====== Comparator config ====== */
    /* Configure the comparator DSI trigger output to be a synchronized version of comparator output instead of a pulse */
    Cy_CTB_CompSetConfig(CTBM0, CY_CTB_OPAMP_0, CY_CTB_COMP_DSI_TRIGGER_OUT_LEVEL, CY_CTB_COMP_BYPASS_SYNC, CY_CTB_COMP_HYST_10MV);
    
    /* Configure the comparator interrupt to trigger on both comparator edges */
    Cy_CTB_CompSetInterruptEdgeType(CTBM0, CY_CTB_OPAMP_0, CY_CTB_COMP_EDGE_BOTH);
    
    /* ====== Comparator DSI output routing ====== */
    /* Route the comparator DSI trigger output to P9_2 */  
    Cy_GPIO_SetHSIOM(COMP_OUT_PORT, COMP_OUT_PIN, P9_2_PASS_DSI_CTB_CMP0);
    
    /* The comparator DSI trigger output can also be routed to the following GPIO pins using trigger muxes.
     * - P11_3
     * - P11_4
     * - P0_4
     * - P0_5
     * - P6_4
     * - P6_5
     * The following lines of code show how to connect the comparator output (OA0) to P11_4.
     */
    (void)Cy_TrigMux_Connect(TRIG14_IN_PASS_DSI_CTB_CMP0, TRIG14_OUT_TR_GROUP8_INPUT42, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_PASS_DSI_CTB_CMP0__LEVEL);
	(void)Cy_TrigMux_Connect(TRIG8_IN_TR_GROUP14_OUTPUT15, TRIG8_OUT_PERI_TR_IO_OUTPUT1, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_TR_GROUP_OUTPUT__LEVEL);
    Cy_GPIO_SetDrivemode(P11_4_PORT, P11_4_NUM, CY_GPIO_DM_STRONG_IN_OFF);
    Cy_GPIO_SetHSIOM(P11_4_PORT, P11_4_NUM, P11_4_PERI_TR_IO_OUTPUT1);
    
#else /* Use the Component API */
    /* Configure the comparator interrupt and provide the ISR address. */
    (void)Cy_SysInt_Init(&SysInt_1_cfg, Comparator_Interrupt);
    /* Enable the interrupt. */
    NVIC_EnableIRQ(SysInt_1_cfg.intrSrc);
    
    /* Start the VDAC component. */
    VDAC_1_Start();
    
    /* Configure the CTDAC to output a constant reference voltage. */
    dacValue *= CY_CTDAC_UNSIGNED_MAX_CODE_VALUE;
    dacValue /= CYDEV_VDDA_MV;
    Cy_CTDAC_SetValueBuffered(CTDAC0, dacValue);
    
    /* Start the OpAmp and comparator components. */
    Opamp_1_Start();
    Comp_1_Start();
#endif

    for(;;)
    {
    }
}


/*******************************************************************************
* Function Name: Comparator_Interrupt
********************************************************************************
*
*  Summary:
*  Interrupt service routine called when a comparator interrupt occurs.
*    
*  Parameters:
*  None
*
*  Return:
*  None
*
**********************************************************************************/
void Comparator_Interrupt(void)
{
	uint8_t intrStatus;

	#if PDL_CONFIGURATION		
		intrStatus = Cy_CTB_GetInterruptStatus(CTBM0, CY_CTB_OPAMP_0);
		/* Ensure that OpAmp0 caused the interrupt and not another analog source sharing this ISR */
		if (CY_CTB_OPAMP_0 == (intrStatus & CY_CTB_OPAMP_0))
		{
    		Cy_CTB_ClearInterrupt(CTBM0, CY_CTB_OPAMP_0);
        
			/* Drive the green LED with the comparator status. */
			Cy_GPIO_Write(LED_GREEN_PORT, LED_GREEN_PIN, Cy_CTB_CompGetStatus(CTBM0, CY_CTB_OPAMP_0));
		}
	#else
		intrStatus = Comp_1_GetInterruptStatus();
		if (0u != intrStatus)
		{
    		Comp_1_ClearInterrupt();
            
			/* Drive the green LED with the comparator status. */
			Cy_GPIO_Write(LED_GREEN_PORT, LED_GREEN_PIN, Comp_1_GetStatus());
		}
	#endif
}

/* [] END OF FILE */
