/******************************************************************************
 * File Name:   main.c
 *
 * Description: This is the source code for the Slave select inversion
 * with smart IO PSoC 4 application for ModusToolbox
 *
 * Related Document: See README.md 
 *
 *
 *******************************************************************************
 * Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 * Include Header files
 ********************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"

/*******************************************************************************
 * Macros and Constants
 ********************************************************************************/
#define TX_ITERATION     (5000u)
#define DUMMY_DATA       (0x55UL)
#define TX_DELAY_MS      (500u)

#if defined  COMPONENT_PSOC4100SP256KB || COMPONENT_PSOC4HVMS128K || COMPONENT_PSOC4HVMS64K
#define SMARTIO_PORT     PRGIO_PRT1
#else
#define SMARTIO_PORT     PRGIO_PRT0
#endif

/*******************************************************************************
 * Global Variables
 ********************************************************************************/
cy_stc_scb_spi_context_t mSPI_context;

/*******************************************************************************
 * Function Name: SmartIO_Start
 ********************************************************************************
 * Summary: This function initializes and enables the Smart IO.
 *
 * Parameters:
 *  None
 *
 * Return
 *  void
 *
 *******************************************************************************/
void SmartIO_Start(void)
{
    /* Configure the Smart I/O */
    if(CY_SMARTIO_SUCCESS != Cy_SmartIO_Init(SMARTIO_PORT, &SmartIO_config))
    {
        CY_ASSERT(0);
    }

    /* Enable Smart I/O */
    Cy_SmartIO_Enable(SMARTIO_PORT);
}

/*******************************************************************************
 * Function Name: mSPI_Start
 *******************************************************************************
 * Summary: This function initializes and enables SPI communication peripheral.
 *
 * Parameters:
 *  None
 *
 * Return
 *  void
 *
 *******************************************************************************/
void mSPI_Start(void)
{
    if(CY_SCB_SPI_SUCCESS != Cy_SCB_SPI_Init(mSPI_HW, &mSPI_config, &mSPI_context))
    {
        CY_ASSERT(0);
    }

    Cy_SCB_SPI_Enable(mSPI_HW);
}

/*******************************************************************************
 * Function Name: main
 ********************************************************************************
 * Summary:
 *  System entrance point. This function performs
 *  1. Initializes the BSP.
 *  2. Calls the functions to setup the Smart IO and SPI peripherals.
 *  3. Periodically switch between slave select 1 and slave select 2.
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
    cy_rslt_t result;
    uint16_t tx_index = 0u;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    SmartIO_Start();
    mSPI_Start();

    for(;;)
    {
        /* Periodically choose slave 1 and then slave 2 and send dummy data */
        Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT1);

        for(tx_index = 0u; tx_index < TX_ITERATION; tx_index++)
        {
            Cy_SCB_SPI_Write(mSPI_HW, DUMMY_DATA);
        }
        Cy_SysLib_Delay(TX_DELAY_MS);

        Cy_SCB_SPI_SetActiveSlaveSelect(mSPI_HW, CY_SCB_SPI_SLAVE_SELECT2);

        for(tx_index = 0u; tx_index < TX_ITERATION; tx_index++)
        {
            Cy_SCB_SPI_Write(mSPI_HW, DUMMY_DATA);
        }
        Cy_SysLib_Delay(TX_DELAY_MS);
    }
}

/* [] END OF FILE */
