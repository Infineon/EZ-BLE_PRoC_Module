/******************************************************************************
* Project Name		: PRoC_BLE_CapSense_Slider_LED
* File Name			: RGB_PRSm.h
* Version 			: 1.0
* Device Used		: CYBL10563-56LQXI
* Software Used		: PSoC Creator 3.1
* Compiler    		: ARM GCC 4.8.4, ARM RVDS Generic, ARM MDK Generic
* Related Hardware	: CY8CKIT-042-BLE Bluetooth Low Energy Pioneer Kit 
* Owner             : ROIT
*
********************************************************************************
* Copyright (2014-15), Cypress Semiconductor Corporation. All Rights Reserved.
********************************************************************************
* This software is owned by Cypress Semiconductor Corporation (Cypress)
* and is protected by and subject to worldwide patent protection (United
* States and foreign), United States copyright laws and international treaty
* provisions. Cypress hereby grants to licensee a personal, non-exclusive,
* non-transferable license to copy, use, modify, create derivative works of,
* and compile the Cypress Source Code and derivative works for the sole
* purpose of creating custom software in support of licensee product to be
* used only in conjunction with a Cypress integrated circuit as specified in
* the applicable agreement. Any reproduction, modification, translation,
* compilation, or representation of this software except as specified above 
* is prohibited without the express written permission of Cypress.
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
* such use and in doing so indemnifies Cypress against all charges. 
*
* Use of this Software may be limited by and subject to the applicable Cypress
* software license agreement. 
*******************************************************************************/
/*******************************************************************************
* Description:
*  This header file contains registers and constants associated with the
*  Software PRSm Component
*******************************************************************************/

#if !defined(RGB_PRSm__H)
	#define RGB_PRSm__H
#include <cytypes.h>
#include "CyLib.h"
#include <core_cm0_psoc4.h>
#include <core_cm0.h>
#include "RGB_PRSm_Red.h"

#define RGB_PRSm_NO_OF_PWMS	3
	
#if (RGB_PRSm_NO_OF_PWMS > 1)
	#include "RGB_PRSm_Green.h"
#endif
#if (RGB_PRSm_NO_OF_PWMS > 2)
	#include "RGB_PRSm_Blue.h"
#endif
#if (RGB_PRSm_NO_OF_PWMS > 3)
	#include "RGB_PRSm_PWM_4.h"
#endif

#define RGB_PRSm_SYSTICK_IRQN     	(uint8)(SysTick_IRQn + 16)  /* Offset between user and system vectors */
#define RGB_PRSm_SYSTICK_PERIOD 	(uint32)(26000000 / 20000)

#define RGB_PRSm_PERIOD			128
#define RGB_PRSm_COMPARE		0

#define RGB_PRSm_LESS_THAN				0
#define RGB_PRSm_LESS_THAN_EQUAL		1
#define RGB_PRSm_GREATER_THAN			2
#define RGB_PRSm_GREATER_THAN_EQUAL		3

#define RGB_PRSm_Red_COMPARE_TYPE			3
#define RGB_PRSm_Green_COMPARE_TYPE			3
#define RGB_PRSm_Blue_COMPARE_TYPE			3


#define RGB_PRSm_EDGE_ALIGN				0
#define RGB_PRSm_CENTER_ALIGN			1
#define RGB_PRSm_PWM_ALIGNMENT			0

#define RGB_PRSm_SINGLE_PERIOD			0
#define RGB_PRSm_MULTI_PERIOD			1
#define RGB_PRSm_PWM_SHARING			0

#define RGB_PRSm_UP_COUNT				0
#define RGB_PRSm_DOWN_COUNT				1

#define RGB_PRSm_DONOT_DISABLE_PWM		0
#define RGB_PRSm_DISABLE_PWM			1

#define RGB_PRSm_ENABLE_BUFFER			1

void RGB_PRSm_Start(uint8);
void RGB_PRSm_Stop(uint8);
void RGB_PRSm_Resume(uint8);
#if (RGB_PRSm_NO_OF_PWMS <= 1)
	#if(RGB_PRSm_ENABLE_BUFFER)
		void RGB_PRSm_WritePeriodBuffer(uint32);
		void RGB_PRSm_WriteCompareBuffer(uint32);
	#endif
	void RGB_PRSm_WritePeriod(uint32);	
	void RGB_PRSm_WriteCompare(uint32);
	uint32 RGB_PRSm_ReadPeriod(void);
	uint32 RGB_PRSm_ReadCompare(void);
	uint32 RGB_PRSm_ReadCounter(void);	
	void RGB_PRSm_Enable_PWM(uint8);
	void RGB_PRSm_Disable_PWM(void);
	uint8 RGB_PRSm_GetTCStatus(void);
	void RGB_PRSm_ClearTCStatus(void);
#else 
	
	#if (RGB_PRSm_PWM_SHARING == RGB_PRSm_MULTI_PERIOD)
		void RGB_PRSm_WritePeriod(uint32, uint8);
		uint8 RGB_PRSm_GetTCStatus(uint8);
		void RGB_PRSm_ClearTCStatus(uint8);
		#if(RGB_PRSm_ENABLE_BUFFER)
			void RGB_PRSm_WritePeriodBuffer(uint32, uint8);
		#endif
	#else
		void RGB_PRSm_WritePeriod(uint32);
		uint8 RGB_PRSm_GetTCStatus(void);
		void RGB_PRSm_ClearTCStatus(void);
		#if(RGB_PRSm_ENABLE_BUFFER)
			void RGB_PRSm_WritePeriodBuffer(uint32);
		#endif
	#endif
	#if(RGB_PRSm_ENABLE_BUFFER)
		void RGB_PRSm_WriteCompareBuffer(uint32, uint8);
	#endif
	void RGB_PRSm_WriteCompare(uint32, uint8);
	
	#if (RGB_PRSm_PWM_SHARING == RGB_PRSm_MULTI_PERIOD)
		uint32 RGB_PRSm_ReadPeriod(uint8);
		uint32 RGB_PRSm_ReadCounter(uint8);
	#else
		uint32 RGB_PRSm_ReadPeriod(void);
		uint32 RGB_PRSm_ReadCounter(void);
	#endif
	
	uint32 RGB_PRSm_ReadCompare(uint8);
	void RGB_PRSm_Disable_PWM(uint8);
	void RGB_PRSm_Enable_PWM(uint8, uint8);
#endif

#endif
/* [] END OF FILE */
