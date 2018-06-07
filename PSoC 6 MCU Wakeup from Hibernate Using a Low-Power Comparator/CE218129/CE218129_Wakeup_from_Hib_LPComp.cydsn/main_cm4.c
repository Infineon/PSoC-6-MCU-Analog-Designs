/******************************************************************************
* File Name: main_cm4.c
* Version 1.10
*
* Description:
*   This code example demonstrates how to configure the register setting for wake
*   up from Hibernate using LPComp input. 
*
* Related Document: CE218129_Wakeup_from_Hib_LPComp.pdf
*
* Hardware Dependency: CY8CKIT-062-BLE Kit
*
*******************************************************************************
* Copyright (2017-2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/
#include "project.h"

/* This project assumes a configuration tool will automatically configure LPComp. 
* To demonstrate how PDL drivers are used to manually configure the components, 
* set the PDL_CONFIGURATION #define to 1, otherwise leave set to 0. */
#define PDL_CONFIGURATION   (0u)

#define LED_OFF                     (1u)
#define LED_ON                      (0u)
#define MY_LPCOMP_ULP_SETTLE        (50u)
#define MY_LPCOMP_OUTPUT_LOW        (0u)
#define MY_LPCOMP_OUTPUT_HIGH       (1u)
#define TOGGLE_LED_PERIOD           (500u)
#define LED_ON_2S_BEFORE_HIB        (2000u)

#if PDL_CONFIGURATION
    /* LPComp configuration structure */
    const cy_stc_lpcomp_config_t myLPCompConfig =
    {
        .outputMode = CY_LPCOMP_OUT_DIRECT,
        .hysteresis = CY_LPCOMP_HYST_DISABLE,
    };
#endif

static void MyLPComp_SetHibernateMode(cy_en_syspm_hib_wakeup_source_t MyLPComp_WakeUpSrc);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  The main function checks the wake-up status and toggles LED once the system 
*  wakes up from the Hibernate mode.
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
        
        /* Configure LPComp output mode and hysteresis for channel 0 */
        Cy_LPComp_Init(LPCOMP, CY_LPCOMP_CHANNEL_0, &myLPCompConfig);
        
        /* Enable the local reference voltage */
        Cy_LPComp_UlpReferenceEnable(LPCOMP);
        /* Set the local reference voltage to the negative terminal and set a GPIO input on the 
           positive terminal for the wake up signal */
        Cy_LPComp_SetInputs(LPCOMP, CY_LPCOMP_CHANNEL_0, CY_LPCOMP_SW_GPIO, CY_LPCOMP_SW_LOCAL_VREF);

        /* Set channel 0 power mode - Ultra Low Power mode */
        Cy_LPComp_SetPower(LPCOMP, CY_LPCOMP_CHANNEL_0, CY_LPCOMP_MODE_ULP);
        
        /* It needs 50us start-up time to settle in ULP mode after the block is enabled */
        Cy_SysLib_DelayUs(MY_LPCOMP_ULP_SETTLE);
    #else
        /* Start the LPComp Component */ 
        LPComp_1_Start();
    #endif
    
    
    /* Check the IO status. If current status is frozen, unfreeze the system. */
    if(Cy_SysPm_GetIoFreezeStatus())
    {   /* Unfreeze the system */
        Cy_SysPm_IoUnfreeze();
    }
    else
    {
        /* Do nothing */    
    }
    
    for(;;)
    {
        /* If the comparison result is high, toggles LED every 500ms */
        if(Cy_LPComp_GetCompare(LPCOMP, CY_LPCOMP_CHANNEL_0) == MY_LPCOMP_OUTPUT_HIGH)
        {
            /* Toggle LED every 500ms */
            Cy_GPIO_Inv(LED_0_PORT, LED_0_NUM);
            Cy_SysLib_Delay(TOGGLE_LED_PERIOD); 
        }
        /* If the comparison result is low, goes to the hibernate mode */
        else    
        {   
            /* System wakes up when LPComp channel 0 output is high */
            MyLPComp_SetHibernateMode(CY_SYSPM_LPCOMP0_HIGH);         
        }
    }
}

/*******************************************************************************
* Function Name: MyLPComp_SetHibernateMode
********************************************************************************
*  The function enters into the hibernation mode. The LED is on 2 seconds before 
*  the Hibernation.
*
* \param MyLPComp_WakeUpSrc
*  CY_SYSPM_LPCOMP0_LOW  : Wake-up source for LPComp CH0 output low 
*  CY_SYSPM_LPCOMP0_HIGH : Wake-up source for LPComp CH0 output high 
*  CY_SYSPM_LPCOMP1_LOW  : Wake-up source for LPComp CH1 output low 
*  CY_SYSPM_LPCOMP1_HIGH : Wake-up source for LPComp CH1 output high 
*  CY_SYSPM_HIBALARM 	 : Wake-up source for RTC alarm 
*  CY_SYSPM_HIBWDT 	     : Wake-up source for watchdog 
*  CY_SYSPM_HIBPIN0_LOW  : Wake-up source for Pin0 input low 
*  CY_SYSPM_HIBPIN0_HIGH : Wake-up source for Pin0 input high 
*  CY_SYSPM_HIBPIN1_LOW  : Wake-up source for Pin1 input low 
*  CY_SYSPM_HIBPIN1_HIGH : Wake-up source for Pin1 input high 
*
*******************************************************************************/
static void MyLPComp_SetHibernateMode(cy_en_syspm_hib_wakeup_source_t MyLPComp_WakeUpSrc)
{
    /* Turn on LED for 2 seconds to indicate the hibernate mode. */
    Cy_GPIO_Write(LED_0_PORT, LED_0_NUM, LED_ON);
    Cy_SysLib_Delay(LED_ON_2S_BEFORE_HIB);
    Cy_GPIO_Write(LED_0_PORT, LED_0_NUM, LED_OFF);

    /* Set the wake-up signal from Hibernate */
    Cy_SysPm_SetHibWakeupSource(MyLPComp_WakeUpSrc);
    
    /* Jump into Hibernate */
    Cy_SysPm_Hibernate();
}

/* [] END OF FILE */
