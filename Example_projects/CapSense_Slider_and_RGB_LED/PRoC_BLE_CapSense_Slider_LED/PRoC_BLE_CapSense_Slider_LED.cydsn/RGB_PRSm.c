/******************************************************************************
* Project Name		: PRoC_BLE_CapSense_Slider_LED
* File Name			: RGB_PRSm.c
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
*  This file provides source code for the Software PRSm component's API.
*******************************************************************************/
#include "RGB_PRSm.h"


static uint32 RGB_PRSm_period;
uint32 RGB_PRSm_periodBuffer;
uint32 RGB_PRSm_count;
static uint8 RGB_PRSm_TcFlag;

static uint32 RGB_PRSm_compare[RGB_PRSm_NO_OF_PWMS];
uint32 RGB_PRSm_compareBuffer[RGB_PRSm_NO_OF_PWMS];	

/*******************************************************************************
* Function Name: RGB_PRSm_Clock_ISR
********************************************************************************
*
* Summary:
*  SysTick ISR where the PWM counter and compare takes place.
*
* Parameters:
*  None
*
* Return:
*  None
*
* Reentrant:
*  No.
*
*******************************************************************************/
CY_ISR(RGB_PRSm_Clock_ISR)
{
	static uint8 count;
	
	/* Set the LED Pin outs depending on the current counter value, compare value & compare type */
	#if (RGB_PRSm_Red_COMPARE_TYPE == RGB_PRSm_LESS_THAN)
		if(RGB_PRSm_count < RGB_PRSm_compare[0])
	#elif (RGB_PRSm_Red_COMPARE_TYPE == RGB_PRSm_LESS_THAN_EQUAL)
		if(RGB_PRSm_count <= RGB_PRSm_compare[0])
	#elif (RGB_PRSm_Red_COMPARE_TYPE == RGB_PRSm_GREATER_THAN)
		if(RGB_PRSm_count > RGB_PRSm_compare[0])
	#else
		if(RGB_PRSm_count >= RGB_PRSm_compare[0])
	#endif
	{
		RGB_PRSm_Red_DR |= RGB_PRSm_Red_MASK;
	}
	else
	{
		RGB_PRSm_Red_DR &= ~RGB_PRSm_Red_MASK;
	}
	
	#if (RGB_PRSm_Green_COMPARE_TYPE == RGB_PRSm_LESS_THAN)
		if(RGB_PRSm_count < RGB_PRSm_compare[1])
	#elif (RGB_PRSm_Green_COMPARE_TYPE == RGB_PRSm_LESS_THAN_EQUAL)
		if(RGB_PRSm_count <= RGB_PRSm_compare[1])
	#elif (RGB_PRSm_Green_COMPARE_TYPE == RGB_PRSm_GREATER_THAN)
		if(RGB_PRSm_count > RGB_PRSm_compare[1])
	#else
		if(RGB_PRSm_count >= RGB_PRSm_compare[1])
	#endif
	{
		RGB_PRSm_Green_DR |= RGB_PRSm_Green_MASK;
	}
	else
	{
		RGB_PRSm_Green_DR &= ~RGB_PRSm_Green_MASK;
	}
		
	
	#if (RGB_PRSm_Blue_COMPARE_TYPE == RGB_PRSm_LESS_THAN)
		if(RGB_PRSm_count < RGB_PRSm_compare[2])
	#elif (RGB_PRSm_Blue_COMPARE_TYPE == RGB_PRSm_LESS_THAN_EQUAL)
		if(RGB_PRSm_count <= RGB_PRSm_compare[2])
	#elif (RGB_PRSm_Blue_COMPARE_TYPE == RGB_PRSm_GREATER_THAN)
		if(RGB_PRSm_count > RGB_PRSm_compare[2])
	#else
		if(RGB_PRSm_count >= RGB_PRSm_compare[2])
	#endif
	{
		RGB_PRSm_Blue_DR |= RGB_PRSm_Blue_MASK;
	}
	else
	{
		RGB_PRSm_Blue_DR &= ~RGB_PRSm_Blue_MASK;
	}
	
	/* Update the counter with PRSm code (7-bit PRS generator with polynomial [7,6,5,2]) -
		1. Shift left the current count value
		2. Apply polynomial [7,6,5,2] tap to new bit 0
		3. Mask out unwanted bits */
	RGB_PRSm_count = (RGB_PRSm_count<<1);
	RGB_PRSm_count |= (((((RGB_PRSm_count>>7) ^ (RGB_PRSm_count>>6)) ^ ((RGB_PRSm_count>>5)^(RGB_PRSm_count>>2))) & 0x01));
	RGB_PRSm_count &= 0x7F;
	
	count++;
	/* terminal count */
	if(count == (RGB_PRSm_PERIOD-1))
	{
		/* Clear the counter when the counter reaches terminal count (127 for 7 bit PRS) */
		count = 0;
		
		/* update the compare & period register from respective buffer */
		#if(RGB_PRSm_ENABLE_BUFFER)
			RGB_PRSm_period = RGB_PRSm_periodBuffer;						
			RGB_PRSm_compare[0] = RGB_PRSm_compareBuffer[0];
			RGB_PRSm_compare[1] = RGB_PRSm_compareBuffer[1];			
			RGB_PRSm_compare[2] = RGB_PRSm_compareBuffer[2];			
		#endif
		
		/* Set TC flag - flag will be set till cleared */
		RGB_PRSm_TcFlag = 1;
	}
		
				
}

/*******************************************************************************
* Function Name: RGB_PRSm_Start
********************************************************************************
*
* Summary:
*  Initializes the counter variables, enables PWM line and 
*	configures SysTick timer ISR.
*
* Parameters:
*  uint8 mode - Drive mode with which the PWM line(s) needs to configured.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void RGB_PRSm_Start(uint8 mode)
{	
	uint8 count;
	
	/* Set the period & period buffer */
	RGB_PRSm_period = RGB_PRSm_PERIOD;
	#if(RGB_PRSm_ENABLE_BUFFER)
		RGB_PRSm_periodBuffer = RGB_PRSm_PERIOD;
	#endif
	
	/* Initialize the counter & clear TC flag */
	RGB_PRSm_count = 1;			
	RGB_PRSm_ClearTCStatus();
	
	/* Initialize the compare registers/buffers & enable the PWM lines */
	for(count = 0; count < RGB_PRSm_NO_OF_PWMS; count++)
	{
		RGB_PRSm_compare[count] = RGB_PRSm_COMPARE;
		#if(RGB_PRSm_ENABLE_BUFFER)
			RGB_PRSm_compareBuffer[count] = RGB_PRSm_COMPARE;	
		#endif
		RGB_PRSm_Enable_PWM(mode, count);			
	}
	
	/* Configure the SysTick timer to generate the ISR at set rate */
	CyIntSetSysVector(RGB_PRSm_SYSTICK_IRQN, RGB_PRSm_Clock_ISR);
	SysTick_Config(RGB_PRSm_SYSTICK_PERIOD);
}

/*******************************************************************************
* Function Name: RGB_PRSm_Stop
********************************************************************************
*
* Summary:
*  Stops the SysTick timer and disables PWM lines if requested
*
* Parameters:
*  uint8 disablePWM - non-zero value passed to this argument disables all PWM.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void RGB_PRSm_Stop(uint8 disablePwm)
{
	uint8 count;
	SysTick->CTRL  = 0u;  /* Disable interrupt and deactivate timer */
	
	if(disablePwm)
	{
		#if(RGB_PRSm_NO_OF_PWMS > 1)
		for(count = 0; count < RGB_PRSm_NO_OF_PWMS; count++)
		{
			RGB_PRSm_Disable_PWM(count);
			
		}
		#else
			RGB_PRSm_Disable_PWM();
		#endif
	}
}

/*******************************************************************************
* Function Name: RGB_PRSm_Resume
********************************************************************************
*
* Summary:
*  Enables the PWM and resumes the SysTick counter by counting from last left 
*	value(s). Does not clear the counter variables like Start() API.
*
* Parameters:
*  uint8 mode - Drive mode with which PWMs need to enabled.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
void RGB_PRSm_Resume(uint8 mode)
{		
	uint8 count;
	for(count = 0; count < RGB_PRSm_NO_OF_PWMS; count++)
	{
		RGB_PRSm_Enable_PWM(mode, count);
		
	}
	
	SysTick_Config(RGB_PRSm_SYSTICK_PERIOD);
}	
	

/*******************************************************************************
* Function Name: RGB_PRSm_WritePeriod
********************************************************************************
*
* Summary:
*  Writes a 32-bit value to the period register and also updates the buffer if
*	enabled. Period for all the enabled PWMs
*
* Parameters:
*  uint32 - value to be written to the period register.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
		void RGB_PRSm_WritePeriod(uint32 value)
		{
			#if(RGB_PRSm_ENABLE_BUFFER)
				RGB_PRSm_periodBuffer = value;
			#endif
			RGB_PRSm_period = value;
		}
		#if(RGB_PRSm_ENABLE_BUFFER)
			
/*******************************************************************************
* Function Name: RGB_PRSm_WritePeriodBuffer
********************************************************************************
*
* Summary:
*  Writes a 32-bit value to the period buffer register
*
* Parameters:
*  uint32 - value to be written to the period buffer.
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
			void RGB_PRSm_WritePeriodBuffer(uint32 value)
			{
				RGB_PRSm_periodBuffer = value;
			}
		#endif
		
/*******************************************************************************
* Function Name: RGB_PRSm_GetTCStatus
********************************************************************************
*
* Summary:
*  Returns the terminal Count status of the counter/PWM and clear the TC flag
*
* Parameters:
*  none
*
* Return:
*  Non-zero value - Terminal count flag was set.
*	0 - No terminal count since last check/clear.
*
* Reentrant:
*  No.
*
*******************************************************************************/
		uint8 RGB_PRSm_GetTCStatus(void)
		{
			uint8 status = RGB_PRSm_TcFlag;
			if(status)
			{
				RGB_PRSm_TcFlag = 0;
			}
			return status;
		}
		
/*******************************************************************************
* Function Name: RGB_PRSm_ClearTCStatus
********************************************************************************
*
* Summary:
*  Clears the TC flag
*
* Parameters:
*  none
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
		void RGB_PRSm_ClearTCStatus(void)
		{			
			RGB_PRSm_TcFlag = 0;			
		}
	
	#if(RGB_PRSm_ENABLE_BUFFER)
		
/*******************************************************************************
* Function Name: RGB_PRSm_WriteCompareBuffer
********************************************************************************
*
* Summary:
*  Writes a 32-bit value to the compare buffer register
*
* Parameters:
*  uint32 - value to be written to the compare buffer.
*	uint8 - PWM index whose compare buffer needs to be updated
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
		void RGB_PRSm_WriteCompareBuffer(uint32 value, uint8 index)
		{
			RGB_PRSm_compareBuffer[index] = value;
		}
	#endif
	
/*******************************************************************************
* Function Name: RGB_PRSm_WriteCompare
********************************************************************************
*
* Summary:
*  Writes a 32-bit value to the compare register. Also updates compare buffer if
*	enabled.
*
* Parameters:
*  uint32 - value to be written to the compare.
*	uint8 - PWM index whose compare buffer needs to be updated
*
* Return:
*  None.
*
* Reentrant:
*  No.
*
*******************************************************************************/
	void RGB_PRSm_WriteCompare(uint32 value, uint8 index)
	{
		#if(RGB_PRSm_ENABLE_BUFFER)
			RGB_PRSm_compareBuffer[index] = value;
		#endif
		RGB_PRSm_compare[index] = value;
	}
	
	
		
/*******************************************************************************
* Function Name: RGB_PRSm_ReadPeriod
********************************************************************************
*
* Summary:
*  Returns the 32-bit value in the period register. 
*
* Parameters:
*  none
*
* Return:
*  uint32 - period value.
*
* Reentrant:
*  No.
*
*******************************************************************************/
		uint32 RGB_PRSm_ReadPeriod(void)
		{
			return RGB_PRSm_period;
		}
		
/*******************************************************************************
* Function Name: RGB_PRSm_ReadCounter
********************************************************************************
*
* Summary:
*  Returns the 32-bit value in the count register. 
*
* Parameters:
*  none
*
* Return:
*  uint32 - count value.
*
* Reentrant:
*  No.
*
*******************************************************************************/
		uint32 RGB_PRSm_ReadCounter(void)
		{
			return RGB_PRSm_count;
		}
	

/*******************************************************************************
* Function Name: RGB_PRSm_ReadCompare
********************************************************************************
*
* Summary:
*  Returns the 32-bit value in the compare register. 
*
* Parameters:
*  uint8 - PWM index whose compare value is requested
*
* Return:
*  uint32 - compare value.
*
* Reentrant:
*  No.
*
*******************************************************************************/
	uint32 RGB_PRSm_ReadCompare(uint8 index)
	{
		return RGB_PRSm_compare[index];
	}
	
/*******************************************************************************
* Function Name: RGB_PRSm_Disable_PWM
********************************************************************************
*
* Summary:
*  Disables the PWM line by putting it in Hi-Z. Disabling the PWM does not stop
*	counter for that PWM, it just disables the output line. In order to stop the
*	counter, use RGB_PRSm_Stop() API
*
* Parameters:
*  uint8 - PWM index which needs to be disabled
*
* Return:
*  none.
*
* Reentrant:
*  No.
*
*******************************************************************************/
	void RGB_PRSm_Disable_PWM(uint8 index)
	{
		switch (index)
		{
			case 0:
				RGB_PRSm_Red_DR &= ~RGB_PRSm_Red_MASK;
				RGB_PRSm_Red_SetDriveMode(RGB_PRSm_Red_DM_DIG_HIZ);
				break;
			case 1: 
				RGB_PRSm_Green_DR &= ~RGB_PRSm_Green_MASK;
				RGB_PRSm_Green_SetDriveMode(RGB_PRSm_Green_DM_DIG_HIZ);
				break;
			#if(RGB_PRSm_NO_OF_PWMS > 2)
			case 2:
				RGB_PRSm_Blue_DR &= ~RGB_PRSm_Blue_MASK;
				RGB_PRSm_Blue_SetDriveMode(RGB_PRSm_Blue_DM_DIG_HIZ);
				break;
			#endif
			#if(RGB_PRSm_NO_OF_PWMS > 3)
			case 3: 
				RGB_PRSm_PWM_4_DR &= ~RGB_PRSm_Blue_MASK;
				RGB_PRSm_PWM_4_SetDriveMode(RGB_PRSm_PWM_4_DM_DIG_HIZ);
				break;
			#endif
			default:
			break;
		}
	}
	
/*******************************************************************************
* Function Name: RGB_PRSm_Enable_PWM
********************************************************************************
*
* Summary:
*  Enables the PWM line and puts it in the drive mode specified 
*
* Parameters:
*  mode - 
*	RGB_PRSm_PWM_x_DM_ALG_HIZ        
*  	RGB_PRSm_PWM_x_DM_DIG_HIZ
*	RGB_PRSm_PWM_x_DM_RES_UP          
*	RGB_PRSm_PWM_x_DM_RES_DWN        
*	RGB_PRSm_PWM_x_DM_OD_LO          
*	RGB_PRSm_PWM_x_DM_OD_HI          
*	RGB_PRSm_PWM_x_DM_STRONG         
*	RGB_PRSm_PWM_x_DM_RES_UPDWN
*
* index - PWM index which needs to be enabled.
*
* Return:
*  none.
*
* Reentrant:
*  No.
*
*******************************************************************************/
	void RGB_PRSm_Enable_PWM(uint8 mode, uint8 index)
	{
		switch (index)
		{
			case 0:
				RGB_PRSm_Red_SetDriveMode(mode);
				break;
			case 1: 
				RGB_PRSm_Green_SetDriveMode(mode);
				break;
			#if(RGB_PRSm_NO_OF_PWMS > 2)
			case 2:
				RGB_PRSm_Blue_SetDriveMode(mode);
				break;
			#endif
			#if(RGB_PRSm_NO_OF_PWMS > 3)
			case 3: 
				RGB_PRSm_PWM_4_SetDriveMode(mode);
				break;
			#endif
			default:
			break;
		}
	}


