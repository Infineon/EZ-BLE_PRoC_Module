/*******************************************************************************
* File Name: app_UART.h
*
* Description:
*  Contains the function prototypes and constants available to the example
*  project.
*
********************************************************************************
* Copyright 2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef APP_UART_H
    
    #define APP_UART_H
    
    #include <project.h>
	#include "main.h"
    #include "stdbool.h"    
    
    /***************************************
    *       Constants
    ***************************************/
    #ifdef LOW_POWER_MODE
		#define UART_IDLE_TIMEOUT 10
	#else
		#define UART_IDLE_TIMEOUT 1000	
	#endif
    
	#define UART_RX_INTR_MASK     0x00000004
    #define MAX_MTU_SIZE          512
    /***************************************
    *       External data references
    ***************************************/
    extern uint16 mtuSize;
    
    /***************************************
    *       Function Prototypes
    ***************************************/
    void HandleUartTxTraffic(uint16);
    void HandleUartRxTraffic(CYBLE_GATTS_WRITE_REQ_PARAM_T *);
    void DisableUartRxInt(void);
    void EnableUartRxInt(void);
    
#endif

/* [] END OF FILE */
