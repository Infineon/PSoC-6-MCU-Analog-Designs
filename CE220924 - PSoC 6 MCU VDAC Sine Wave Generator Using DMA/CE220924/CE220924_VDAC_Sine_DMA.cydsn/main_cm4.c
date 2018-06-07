/******************************************************************************
* File Name: main_cm4.c
*
* Version: 1.00
*
* Description:  This example demonstrates how to use the Voltage DAC (12-bit)
*               component and CTDAC PDL as a sine wave generator using DMA
*               transfers from a lookup table in a PSoC 6 MCU.
*
* Related Document: CE220924.pdf
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
#include "dma/cy_dma.h"

/* This project assumes a configuration tool will automatically configure system */
/* resources. To demonstrate how PDL drivers are used to manually configure */
/* the VDAC, set the PDL_CONFIGURATION #define to 1, otherwise leave set to 0. */
#define PDL_CONFIGURATION   (0u)

/* Number of points in the lookup table. */
#define NUM_POINTS          (100u)

/* Lookup table for a sine wave in unsigned format. */
uint32_t sineWaveLUT[] = {0x7FF, 0x880, 0x900, 0x97F, 0x9FC, 0xA78, 0xAF1, 0xB67, 0xBD9, 0xC48,
                         0xCB2, 0xD18, 0xD79, 0xDD4, 0xE29, 0xE77, 0xEC0, 0xF01, 0xF3C, 0xF6F,
                         0xF9A, 0xFBE, 0xFDA, 0xFEE, 0xFFA, 0xFFF, 0xFFA, 0xFEE, 0xFDA, 0xFBE,
                         0xF9A, 0xF6F, 0xF3C, 0xF01, 0xEC0, 0xE77, 0xE29, 0xDD4, 0xD79, 0xD18,
                         0xCB2, 0xC48, 0xBD9, 0xB67, 0xAF1, 0xA78, 0x9FC, 0x97F, 0x900, 0x880,
                         0x7FF, 0x77E, 0x6FE, 0x67F, 0x602, 0x586, 0x50D, 0x497, 0x425, 0x3B6,
                         0x34C, 0x2E6, 0x285, 0x22A, 0x1D5, 0x187, 0x13E, 0x0FD, 0x0C2, 0x08F,
                         0x064, 0x040, 0x024, 0x010, 0x004, 0x000, 0x004, 0x010, 0x024, 0x040,
                         0x064, 0x08F, 0x0C2, 0x0FD, 0x13E, 0x187, 0x1D5, 0x22A, 0x285, 0x2E6,
                         0x34C, 0x3B6, 0x425, 0x497, 0x50D, 0x586, 0x602, 0x67F, 0x6FE, 0x77E};

/*******************************************************************************
* Function Name: main
********************************************************************************
*
* Summary:
*  The main function performs the following actions:
*   1. Initializes VDAC and DMA Components
*   2. DMA continuously updates VDAC value from lookup table
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
    /* Configure VDAC for VDDA reference and unbuffered output.
     * FastInit will configure the VDAC for unsigned mode and buffered updates
     * with a 500 kHz clock by default. */
    Cy_CTDAC_FastInit(CTDAC0, &Cy_CTDAC_Fast_VddaRef_UnbufferedOut);

    /* Configure Trigger Mux to route VDAC trigger output to DMA input trigger */
    (void)Cy_TrigMux_Connect(TRIG14_IN_PASS_TR_CTDAC_EMPTY, TRIG14_OUT_TR_GROUP0_INPUT50, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_PASS_TR_CTDAC_EMPTY);
    (void)Cy_TrigMux_Connect(TRIG1_IN_TR_GROUP14_OUTPUT7, TRIG1_OUT_CPUSS_DW1_TR_IN0, CY_TR_MUX_TR_INV_DISABLE, TRIGGER_TYPE_TR_GROUP_OUTPUT__LEVEL);

    /* Enable CTDAC hardware blocks */
    Cy_CTDAC_Enable(CTDAC0);

    DMA_1_Start(sineWaveLUT, &CTDAC0->CTDAC_VAL_NXT);
#else
    VDAC_1_Start();
    DMA_1_Start(sineWaveLUT, (uint32_t *)&(CTDAC0->CTDAC_VAL_NXT));
#endif

    /* No CPU operations are required because the design uses DMA for all memory transfers. */
    for(;;)
    {
    }
}

/* [] END OF FILE */
