/******************************************************************************
* File Name: main_cm4.c
* Version 1.0
*
* Description:
*   LPComp Code Example. This code example demonstrates how to set the 
*   Low-Power Comparator Component options for the internal reference voltage. 
*
* Related Document: CE218472.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE PSoC 6 BLE Pioneer Kit
*
******************************************************************************
* Copyright (C) 2017, Cypress Semiconductor Corporation.
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

#define MY_LPCOMP_TRIGGERED                 (1u)
#define MY_LPCOMP_DATA_WAIT                 (0u)

/* This project assumes a configuration tool will automatically configure system 
* resources. To demonstrate how PDL drivers are used to manually configure the 
* components, set the PDL_CONFIGURATION #define to 1, otherwise leave set to 0.*/
#define PDL_CONFIGURATION   (0u)

void LPComp_ISR_Callback(void);

#if PDL_CONFIGURATION
    #define MY_LPCOMP_ULP_START_UP_DELAY        (50u)
    
    /* LPComp configuration structure; enable the hysteresis. 
       Any outputMode is OK because this example doesn't use the output pin */
    const cy_stc_lpcomp_config_t myLPCompConfig =
    {
        .outputMode = CY_LPCOMP_OUT_DIRECT,
        .hysteresis = CY_LPCOMP_HYST_ENABLE,
    };
    
    /* LPComp interrupt initialization structure. */
    #define LPCOMP_INTR_PRIORITY   7
	const cy_stc_sysint_t myLPComp_IRQ_cfg = {
		.intrSrc 		= lpcomp_interrupt_IRQn,
		.intrPriority 	= LPCOMP_INTR_PRIORITY
	};
#endif

/* This flag sets the status of interrupt */
static uint32_t myIRQ_flag = MY_LPCOMP_DATA_WAIT;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary: The main function checks the LPComp flag and indicate the result on 
* the red LED. This function configures the ultra-low power(ULP) mode for LPComp
* because only the ULP can compare an external voltage and the internal reference 
* in the deep sleep mode.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Side Effects:
*  None  
*
*******************************************************************************/
int main(void)
{
    uint32_t myGSR_cmp = 0u;
    
    #if PDL_CONFIGURATION
        Cy_SysInt_Init(&myLPComp_IRQ_cfg, LPComp_ISR_Callback);
        NVIC_EnableIRQ(myLPComp_IRQ_cfg.intrSrc);
        
        /* Enable the whole LPComp block */
        Cy_LPComp_GlobalEnable(LPCOMP);

        /* Configure LPComp output mode and hysteresis for channel 1 */
        Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_1, &myLPCompConfig);

        /* Enable ULP local VREF */
        Cy_LPComp_UlpReferenceEnable(LPCOMP);

        /* Set the positive terminal to GPIO input and the negative terminal to the internal reference */
        Cy_LPComp_SetInputs(LPCOMP, CY_LPCOMP_CHANNEL_1, CY_LPCOMP_SW_GPIO, CY_LPCOMP_SW_LOCAL_VREF);

        /* Set channel 1 power mode - Ultra Low Power mode */
        Cy_LPComp_SetPower(LPCOMP, CY_LPCOMP_CHANNEL_1, CY_LPCOMP_MODE_ULP);

        /* Set an interrupt trigger mode for the both edges */
        Cy_LPComp_SetInterruptTriggerMode(LPCOMP, CY_LPCOMP_CHANNEL_1, CY_LPCOMP_INTR_BOTH);

        /* Set the interrupt for LPComp channel 1 */
        Cy_LPComp_SetInterruptMask(LPCOMP, CY_LPCOMP_CHANNEL_1);

        /* It needs 50us start-up time to configure interrupts in ULP mode after the block is enabled */
        Cy_SysLib_DelayUs(MY_LPCOMP_ULP_START_UP_DELAY);
        
        /* Enable global interrupts. */
        __enable_irq(); 
       
        /* Indicate the comparison result for the initial LED_R status */
        Cy_GPIO_Write(LED_R_0_PORT, LED_R_0_NUM, Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_1));
    #else
        /* Initialize the Global Signal Reference and register the ISR callback */
        Cy_SysInt_Init(&LPComp_IRQ_cfg, LPComp_ISR_Callback);
        NVIC_EnableIRQ(LPComp_IRQ_cfg.intrSrc);
        
        /* Start LPComp Component */
        LPComp_1_Start();
        
        /* Enable global interrupts. */
        __enable_irq(); 
       
        /* Indicate the comparison result for the initial LED_R status */
        Cy_GPIO_Write(LED_R_0_PORT, LED_R_0_NUM, LPComp_1_GetCompare());    
    #endif
    
    for(;;)
    {
        /* Check the myIRQ_flag if the comparator was triggered */
        if(myIRQ_flag == MY_LPCOMP_TRIGGERED)
        {
            /* Get the comparison result from channel 1 */
            myGSR_cmp = Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_1);

            /* Indicate the comparison result using LED */
            Cy_GPIO_Write(LED_R_0_PORT, LED_R_0_NUM, myGSR_cmp);
            
            /* Set myIRQ_flag to avoid the additional comparison without an interrupt */
            myIRQ_flag = MY_LPCOMP_DATA_WAIT;
        }
        
        /* Go to deep sleep mode and only wake up if an interrupt occurs */
        Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
    }
}

/*******************************************************************************
* Function Name: LPComp_ISR_Callback
********************************************************************************
* Summary: This interrupt service routine sets the flag to notify the LPComp 
* triggering to the main loop. 
*
* Parameters:
*  None
*
* Return:
*  None
*
*******************************************************************************/
#if PDL_CONFIGURATION
    void LPComp_ISR_Callback(void)
    {
        /* Clear pending LPComp interrupt */
        Cy_LPComp_ClearInterrupt(LPCOMP, CY_LPCOMP_CHANNEL_1);
        
        /* Set the flag for notifying the triggering to the main loop */
        myIRQ_flag = MY_LPCOMP_TRIGGERED;

        /* Clear pending IRQ interrupt */
        NVIC_ClearPendingIRQ(myLPComp_IRQ_cfg.intrSrc);
    }
#else
    void LPComp_ISR_Callback(void)
    {
        /* Clear pending LPComp interrupt */
        LPComp_1_ClearInterrupt(LPComp_1_INTR_MASK);
        
        /* Set the flag for notifying the triggering to the main loop */
        myIRQ_flag = MY_LPCOMP_TRIGGERED;

        /* Clear pending IRQ interrupt */
        NVIC_ClearPendingIRQ(LPComp_IRQ_cfg.intrSrc);
    }
#endif

/* [] END OF FILE */
