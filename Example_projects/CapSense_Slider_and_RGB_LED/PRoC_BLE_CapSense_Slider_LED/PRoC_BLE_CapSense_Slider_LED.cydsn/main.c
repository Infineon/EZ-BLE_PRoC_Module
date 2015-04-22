/******************************************************************************
* Project Name		: PRoC_BLE_CapSense_Slider_LED
* File Name			: main.c
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

/******************************************************************************
*                           THEORY OF OPERATION
*******************************************************************************
* This project will showcase the capability of PRoC BLE to communicate 
* bi-directionally with a BLE Central device. The centroid value of CapSense
* slider is sent from PRoC BLE to central device using a custom service. Also, 
* RGB LED control data is sent from central device to PRoC BLE over another 
* custom service. The CapSense custom service allows notifications to be sent to
* central device when notifications are enabled. On the other hand, the RGB LED 
* custom service allows read and write of attributes under the RGB characteristics.
* This project utilizes CapSense component to check finger position on slider 
* and report this to central device over BLE. Also, the control values sent to 
* PRoC BLE is converted to respective color and intensity on the onboard RGB LED
* The BLE central device can be any BLE central device, including CySmart mobile
* app or CySmart PC tool. 
* This project also inludes low power mode operation, idle for battery operated 
* devices. The project utlizes Deep Sleep feature of both BLESS and CPU to remain 
* in low power mode as much as possible, while maintaining the BLE connection and  
* data transfer. This allows the device to run on coin cell battery for long time.
*
* Note:
* The programming pins have been configured as GPIO, and not SWD. This is because 
* when programming pins are configured for SWD, then the silicon consumes extra
* power through the pins. To prevent the leakage of power, the pins have been set 
* to GPIO. With this setting, the kit can still be acquired by PSoC Creator or
* PSoC Programmer software tools for programming, but the project cannot be 
* externally debugged. To re-enable debugging, go to PRoC_BLE_CapSense_Slider_LED.cydwr 
* from Workspace Explorer, go to Systems tab, and set the Debug Select option to 'SWD'.
* Build and program this project to enable external Debugging.
*
* Refer to BLE Pioneer Kit user guide for details.
************************************************************************************
* Hardware connection required for testing -
* Slider pins 	- P2[1]-P2[5] (hard-wired in the BLE Pioneer Kit)
* Cmod pin		- P4[0] (hard-wired in the PRoC BLE module)
* R-G-B LED 	- P2[6], P3[6] and P3[7] (hard-wired on the BLE Pioneer Kit)
* User Switch	- P2[7] (hard-wired on the BLE Pioneer Kit)
***********************************************************************************/
#include <main.h>

/*This flag is set when the Central device writes to CCCD of the 
* CapSense slider Characteristic to enable notifications */
extern uint8 sendCapSenseSliderNotifications;	

/* Array to store the present RGB LED control data. The 4 bytes 
* of the array represents {R,G,B,Intensity} */
extern uint8 RGBledData[RGB_CHAR_DATA_LEN];						

/* This flag is used by application to know whether a Central 
* device has been connected. This is updated in BLE event callback 
* function*/
extern uint8 deviceConnected;

/* 'restartAdvertisement' flag provided the present state of power mode in firmware */
extern uint8 restartAdvertisement;

/* 'connectionHandle' stores connection parameters */
extern CYBLE_CONN_HANDLE_T  connectionHandle;

/*'enableCapSenseData' flag is set after the Connection LED off status is receieved.
  This flag ensures that the CapSense related scanning does not increase the loop 
  time of firmare, causing the LED to be always ON.*/
extern uint8 enableCapSenseData;

/* 'initializeCapSenseBaseline' flag is used to call the function once that initializes 
* all CapSense baseline. The baseline is initialized when the first advertisement 
* is started. This is done so that any external disturbance while powering the kit does 
* not infliuence the baseline value, that may cause wrong readings. */
uint8 initializeCapSenseBaseline = TRUE;
/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*        System entrance point. This calls the initializing function and
* continuously process BLE and CapSense events.
*
* Parameters:
*  void
*
* Return:
*  int
*

*******************************************************************************/
int main()
{
	/* This function will initilize the system resources such as BLE and CapSense */
    InitializeSystem();
	
    for(;;)
    {
        /*Process event callback to handle BLE events. The events generated and 
		* used for this application are inside the 'CustomEventHandler' routine*/
        CyBle_ProcessEvents();
		
		/* Updated LED for status during BLE active states */
		HandleLEDs();
		
		if(TRUE == deviceConnected)
		{
			/* After the connection, send new connection parameter to the Client device 
			* to run the BLE communication on desired interval. This affects the data rate 
			* and power consumption. High connection interval will have lower data rate but 
			* lower power consumption. Low connection interval will have higher data rate at
			* expense of higher power. This function is called only once per connection. */
			UpdateConnectionParam();
			
			/* When the Client Characteristic Configuration descriptor (CCCD) is written
			* by Central device for enabling/disabling notifications, then the same
			* descriptor value has to be explicitly updated in application so that
			* it reflects the correct value when the descriptor is read */
			UpdateNotificationCCCD();
			
			/* Send CapSense Slider data when respective notification is enabled */
			if(sendCapSenseSliderNotifications & CCCD_NTF_BIT_MASK)
			{
				/* Check for CapSense slider swipe and send data accordingly */
				HandleCapSenseSlider();
			}
		}
		
		#ifdef ENABLE_LOW_POWER_MODE
			/* Put system to Deep sleep, including BLESS, and wakeup on interrupt. 
			* The source of the interrupt can be either BLESS Link Layer in case of 
			* BLE advertisement and connection or by User Button press during BLE 
			* disconnection */
			HandleLowPowerMode();
		#endif
		
		if(restartAdvertisement)
		{
			/* Reset 'restartAdvertisement' flag*/
			restartAdvertisement = FALSE;
			
			/* If CapSense Initialize Baseline API has not been called yet, call the
			* API and reset the flag. */
			if(initializeCapSenseBaseline)
			{
				/* Reset 'initializeCapSenseBaseline' flag*/
				initializeCapSenseBaseline = FALSE;
				
				/* Initialize all CapSense Baseline */
				CapSense_InitializeAllBaselines();
			}

			/* Start Advertisement and enter Discoverable mode*/
			CyBle_GappStartAdvertisement(CYBLE_ADVERTISING_FAST);	
		}
    }	/* End of for(;;) */
}

/*******************************************************************************
* Function Name: InitializeSystem
********************************************************************************
* Summary:
*        Start the components and initialize system 
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void InitializeSystem(void)
{
	/* Enable global interrupt mask */
	CyGlobalIntEnable; 

	/* Start BLE component and register the CustomEventHandler function. This 
	* function exposes the events from BLE component for application use */
    CyBle_Start(CustomEventHandler);	

	/* Start the Button ISR to allow wakeup from sleep */
	isr_user_button_StartEx(MyISR);
	
	#ifdef CAPSENSE_ENABLED
	/*Initialize CapSense component and initialize baselines*/
	CapSense_Start();
	#endif
	
	/* Set the Watchdog Interrupt vector to the address of Interrupt routine 
	* WDT_INT_Handler. This routine counts the 3 seconds for LED ON state during
	* connection. */
	CyIntSetVector(WATCHDOG_INT_VEC_NUM, &WDT_INT_Handler);
}

/*******************************************************************************
* Function Name: HandleCapSenseSlider
********************************************************************************
* Summary:
*        This function scans for finger psoition on CapSense slider, and if the  
* position is different, trigger separate routine for BLE notification
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void HandleCapSenseSlider(void)
{
	#ifdef CAPSENSE_ENABLED
	/* Last read CapSense slider position value */
	static uint16 lastPosition;	
	
	/* Present slider position read by CapSense */
	uint16 sliderPosition;
	
	/* Scan the slider widget */
	CapSense_ScanEnabledWidgets();			
	
	/* Wait for CapSense scanning to be complete. This could take about 5 ms */
	while(CapSense_IsBusy());
	
	/* Update CapSense baseline for next reading*/
	CapSense_UpdateEnabledBaselines();		
	
	/* Read the finger position on the slider */
	sliderPosition = CapSense_GetCentroidPos(CapSense_LINEARSLIDER0__LS);	
	
	/* If finger position on the slider is changed then send data as BLE notifications */
	if(sliderPosition != lastPosition)
	{
		/*If finger is detected on the slider and finger position is in range of slider*/
		if((sliderPosition == NO_FINGER) || (sliderPosition <= SLIDER_MAX_VALUE))
		{
			/* Update global variable with present finger position on slider*/
			lastPosition = sliderPosition;
			
			/* Send updated data as part of BLE notification */
			SendDataOverCapSenseNotification((uint8)sliderPosition);

		}	/* if(sliderPosition != NO_FINGER) */
	}	/* if(sliderPosition != lastPosition) */
	#endif
}

/******************************************************************************
* Function Name: RGB_AssignIntensity
*******************************************************************************
*
* Summary:
*  The function performs the following function - 
*	1.Updates the PWM compare registers based on the passed values
*
* Parameters:
*  rVal - Red LED intensity
*  gVal - Green LED intensity.
*  bVal - Blue LED intensity.
*
* Return:
*  none
*
* Side Effects:
*	None
*
******************************************************************************/
void RGB_AssignIntensity(uint8 rVal, uint8 gVal, uint8 bVal)
{
	/* Because of unequal max intensity levels of the LEDs on the board (RED being
		more bright) the RED LED intensity is scaled to 87.5% of the calculated 
		intensity. */
	/* Scale down red only if SCALE_DOWN_RED macro is enabled */
	
	#ifdef SCALE_DOWN_RED
		if(rVal < (((uint16)MAX_BRIGHTNESS*MAX_PERCENT) >> BRIGHTNESS_SHIFT))
		{
			RGB_PRSm_WriteCompareBuffer(((uint16)rVal*MAX_PERCENT)>>BRIGHTNESS_SHIFT, 0);
		}
		else
		{
			RGB_PRSm_WriteCompareBuffer((((uint16)MAX_BRIGHTNESS*MAX_PERCENT) >> BRIGHTNESS_SHIFT), 0);
		}
	#else
		if(rVal < (MAX_BRIGHTNESS))
		{
			RGB_PRSm_WriteCompareBuffer(rVal, 0);
		}
		else
		{
			RGB_PRSm_WriteCompareBuffer(MAX_BRIGHTNESS, 0);
		}
	#endif
		
	if(gVal < MAX_BRIGHTNESS)
	{
		RGB_PRSm_WriteCompareBuffer(gVal, 1);
	}
	else
	{
		RGB_PRSm_WriteCompareBuffer(MAX_BRIGHTNESS, 1);
	}
	
	if(bVal < MAX_BRIGHTNESS)
	{
		RGB_PRSm_WriteCompareBuffer(bVal, 2);
	}
	else
	{
		RGB_PRSm_WriteCompareBuffer(MAX_BRIGHTNESS, 2);
	}
}
/* [] END OF FILE */
