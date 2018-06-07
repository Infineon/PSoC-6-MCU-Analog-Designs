/******************************************************************************
* File Name: main_cm4.c
* Version 1.0
*
* Description:
*   LPComp Code Example. This code example demonstrates how to configure 
*   the register setting for comparing two external inputs. 
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

#define MY_LPCOMP_LP_START_UP_DELAY     (10u)

/* This project assumes a configuration tool will automatically configure system 
* resources. To demonstrate how PDL drivers are used to manually configure the 
* components, set the PDL_CONFIGURATION #define to 1, otherwise leave set to 0.*/
#define PDL_CONFIGURATION   (0u)

#if PDL_CONFIGURATION
    /* LPComp configuration structure; enable the hysteresis. 
       The outputMode is CY_LPCOMP_OUT_DIRECT which is bypassing the output to pin */
    const cy_stc_lpcomp_config_t myLPCompConfig =
    {
        .outputMode = CY_LPCOMP_OUT_DIRECT,
        .hysteresis = CY_LPCOMP_HYST_ENABLE,
    };
#endif

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary: The main function configures LPComp for the two external inputs in the 
* low power mode.
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
    #if PDL_CONFIGURATION
        /* Enable the whole LPComp block */
        Cy_LPComp_GlobalEnable(LPCOMP);
        
        /* Configure LPComp output mode and hysteresis for channel 1 */
        Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_1, &myLPCompConfig);
        
        /* Set both terminals to GPIO inputs */
        Cy_LPComp_SetInputs(LPCOMP, CY_LPCOMP_CHANNEL_1, CY_LPCOMP_SW_GPIO, CY_LPCOMP_SW_GPIO);

        /* Set channel 1 power mode - Low Power mode */
        Cy_LPComp_SetPower(LPCOMP, CY_LPCOMP_CHANNEL_1, CY_LPCOMP_MODE_LP);
        
        /* It needs 10us start-up time to settle LPComp channel in LP mode after power up */
        Cy_SysLib_DelayUs(MY_LPCOMP_LP_START_UP_DELAY);
    #else
        /* Start LPComp */
        LPComp_1_Start();
    #endif
    
    for(;;)
    {
        /* Empty the main loop - LED_R toggles directly from LPComp output */
    }
}

/* [] END OF FILE */
