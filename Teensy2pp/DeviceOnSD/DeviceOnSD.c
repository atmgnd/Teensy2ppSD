/*
             LUFA Library
     Copyright (C) Dean Camera, 2019.

  dean [at] fourwalledcubicle [dot] com
           www.lufa-lib.org
*/

/*
  Copyright 2019  Dean Camera (dean [at] fourwalledcubicle [dot] com)

  Permission to use, copy, modify, distribute, and sell this
  software and its documentation for any purpose is hereby granted
  without fee, provided that the above copyright notice appear in
  all copies and that both that the copyright notice and this
  permission notice and warranty disclaimer appear in supporting
  documentation, and that the name of the author not be used in
  advertising or publicity pertaining to distribution of the
  software without specific, written prior permission.

  The author disclaims all warranties with regard to this
  software, including all implied warranties of merchantability
  and fitness.  In no event shall the author be liable for any
  special, indirect or consequential damages or any damages
  whatsoever resulting from loss of use, data or profits, whether
  in an action of contract, negligence or other tortious action,
  arising out of or in connection with the use or performance of
  this software.
*/

/** \file
 *
 *  Main source file for the DeviceOnSD demo. This file contains the main tasks of
 *  the demo and is responsible for the initial application hardware configuration.
 */

#include "DeviceOnSD.h"
#include "Lib/diskio.h"
#include "Lib/mmc_avr.h"
#include "Lib/ini.h"
#include "stdlib.h"

/** LUFA CDC Class driver interface configuration and state information. This structure is
 *  passed to all CDC Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber         = INTERFACE_ID_CDC_CCI,
				.DataINEndpoint                 =
					{
						.Address                = CDC_TX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = CDC_RX_EPADDR,
						.Size                   = CDC_TXRX_EPSIZE,
						.Banks                  = 1,
					},
				.NotificationEndpoint           =
					{
						.Address                = CDC_NOTIFICATION_EPADDR,
						.Size                   = CDC_NOTIFICATION_EPSIZE,
						.Banks                  = 1,
					},
			},
	};

/** LUFA Mass Storage Class driver interface configuration and state information. This structure is
 *  passed to all Mass Storage Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_MS_Device_t Disk_MS_Interface =
	{
		.Config =
			{
				.InterfaceNumber                = INTERFACE_ID_MassStorage,
				.DataINEndpoint                 =
					{
						.Address                = MASS_STORAGE_IN_EPADDR,
						.Size                   = MASS_STORAGE_IO_EPSIZE,
						.Banks                  = 1,
					},
				.DataOUTEndpoint                =
					{
						.Address                = MASS_STORAGE_OUT_EPADDR,
						.Size                   = MASS_STORAGE_IO_EPSIZE,
						.Banks                  = 1,
					},
				.TotalLUNs                      = TOTAL_LUNS,
			},
	};

/** Buffer to hold the previously generated Keyboard HID report, for comparison purposes inside the HID class driver. */
static uint8_t PrevKeyboardHIDReportBuffer[sizeof(USB_KeyboardReport_Data_t)];

/** LUFA HID Class driver interface configuration and state information. This structure is
 *  passed to all HID Class driver functions, so that multiple instances of the same class
 *  within a device can be differentiated from one another.
 */
USB_ClassInfo_HID_Device_t Keyboard_HID_Interface =
	{
		.Config =
			{
				.InterfaceNumber = INTERFACE_ID_Keyboard,
				.ReportINEndpoint =
					{
						.Address = KEYBOARD_EPADDR,
						.Size = KEYBOARD_EPSIZE,
						.Banks = 1,
					},
				.PrevReportINBuffer = PrevKeyboardHIDReportBuffer,
				.PrevReportINBufferSize = sizeof(PrevKeyboardHIDReportBuffer),
			},
	};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;

#define DEBUG
#ifdef DEBUG

typedef enum {
	DL_OK,
	DL_NODISK,
	DL_MOUNT,
	DL_OPEN,
	DL_SIZE,
	DL_INI,
} DLVALUE;

static uint8_t dlvalue = DL_OK;

#define DEBUG_SETONCE(x) do{if(!dlvalue)dlvalue = x; }while(0)
#define DEBUG_CHECK_AND_PRINT do{if(dlvalue) fprintf(&USBSerialStream, "error, dlvalue is %d\r\n", (int)dlvalue);}while(0)
#else
#define DEBUG_SETONCE(x) for (;;)
#define DEBUG_CHECK_AND_PRINT
#endif

static uint16_t Timer7 = 0;
ISR(TIMER0_COMPA_vect)
{
	uint8_t n;

	n = Timer7;
	if (n) Timer7 = --n;

	disk_timerproc();

	DEBUG_CHECK_AND_PRINT;
}

uint32_t media_blocks = 0;

enum MODINID
{
	MOD_RAWSTORAGE,
};

uint8_t whh_config = 0;
#define MOD_BIT_TEST(x) ((whh_config) & (1<<(x)))
#define MOD_BIT_SET(x, p) do{(whh_config |= ( p << (x)));}while(0)

static int ini_cb(void* user, const char* section, const char* name,
	const char* value)
{
	const char * _s = "wahaha";
	uint8_t enabled = (atoi(value) == 1);

#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0
	if (MATCH(_s, "raw")) {
		MOD_BIT_SET(MOD_RAWSTORAGE, enabled);
	}
	else {
		return 0;  /* unknown section/name, error */
	}
	return 1;
}

/** Main program entry point. This routine contains the overall program flow, including initial
 *  setup of all components and the main program loop.
 */
int main(void)
{
	/* Start 100Hz system timer with TC0 */
	OCR0A = F_CPU / 1024 / 100 - 1;
	TCCR0A = _BV(WGM01);
	TCCR0B = 0b101;
	TIMSK0 = _BV(OCIE0A);

	sei();

	SetupHardware();

	FRESULT fr;
	FATFS FatFs;

	fr = f_mount(&FatFs, "", 1);
	if (fr)
	{
		DEBUG_SETONCE(DL_MOUNT);
	}

	/* dump ini config */

	if (ini_parse("wahaha.ini", ini_cb, &whh_config) < 0 || MOD_BIT_TEST(MOD_RAWSTORAGE)) {
		RawStorage = 1;
		if (mmc_disk_ioctl(GET_SECTOR_COUNT, &media_blocks) != RES_OK || media_blocks == 0) {
			DEBUG_SETONCE(DL_SIZE);
		}
	}
	else
	{
		/* udisk setup */
		fr = f_open(&MassStorage_Loopback, "udisk.txt", FA_READ | FA_WRITE | FA_OPEN_ALWAYS);
		if (fr)
		{
			DEBUG_SETONCE(DL_OPEN);
		}
		else
		{
			media_blocks = 262144;
		}
	}


	/* Create a regular character stream for the interface so that it can be used with the stdio.h functions */
	CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);


	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
	GlobalInterruptEnable();

	Timer7 = 500;
	for (;;)
	{
		if (Timer7 == 0)
		{
			fprintf(&USBSerialStream, "config: %d\r\n", (int)whh_config);
			Timer7 = 0;
		}

		while(CDC_Device_BytesReceived(&VirtualSerial_CDC_Interface))
		{
			int c = fgetc(&USBSerialStream);
			switch (c)
			{
			case 't':
				fr = f_rename("wahaha.ini", "wahaha.txt");
				fprintf(&USBSerialStream, "t received, %d\r\n", (int)fr);
				break;
			default:
				break;
			}
		}

		/* Must throw away unused bytes from the host, or it will lock up while waiting for the device */
		/* CDC_Device_ReceiveByte(&VirtualSerial_CDC_Interface); */

		CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
		HID_Device_USBTask(&Keyboard_HID_Interface);
		MS_Device_USBTask(&Disk_MS_Interface);
		USB_USBTask();
	}
}

/** Configures the board hardware and chip peripherals for the demo's functionality. */
void SetupHardware(void)
{
#if (ARCH == ARCH_AVR8)
	/* Disable watchdog if enabled by bootloader/fuses */
	MCUSR &= ~(1 << WDRF);
	wdt_disable();

	/* Disable clock division */
	clock_prescale_set(clock_div_1);
#elif (ARCH == ARCH_XMEGA)
	/* Start the PLL to multiply the 2MHz RC oscillator to 32MHz and switch the CPU core to run from it */
	XMEGACLK_StartPLL(CLOCK_SRC_INT_RC2MHZ, 2000000, F_CPU);
	XMEGACLK_SetCPUClockSource(CLOCK_SRC_PLL);

	/* Start the 32MHz internal RC oscillator and start the DFLL to increase it to 48MHz using the USB SOF as a reference */
	XMEGACLK_StartInternalOscillator(CLOCK_SRC_INT_RC32MHZ);
	XMEGACLK_StartDFLL(CLOCK_SRC_INT_RC32MHZ, DFLL_REF_INT_USBSOF, F_USB);

	PMIC.CTRL = PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_HILVLEN_bm;
#endif

	/* Hardware Initialization */
	LEDs_Init();
	USB_Init();
}

/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_ENUMERATING);
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
	LEDs_SetAllLEDs(LEDMASK_USB_NOTREADY);
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	bool ConfigSuccess = true;

	ConfigSuccess &= CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
	ConfigSuccess &= MS_Device_ConfigureEndpoints(&Disk_MS_Interface);
	ConfigSuccess &= HID_Device_ConfigureEndpoints(&Keyboard_HID_Interface);

	USB_Device_EnableSOFEvents();

	LEDs_SetAllLEDs(ConfigSuccess ? LEDMASK_USB_READY : LEDMASK_USB_ERROR);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
	MS_Device_ProcessControlRequest(&Disk_MS_Interface);
	HID_Device_ProcessControlRequest(&Keyboard_HID_Interface);
}

/** Event handler for the USB device Start Of Frame event. */
void EVENT_USB_Device_StartOfFrame(void)
{
	HID_Device_MillisecondElapsed(&Keyboard_HID_Interface);
}

/** CDC class driver callback function the processing of changes to the virtual
 *  control lines sent from the host..
 *
 *  \param[in] CDCInterfaceInfo  Pointer to the CDC class interface configuration structure being referenced
 */
void EVENT_CDC_Device_ControLineStateChanged(USB_ClassInfo_CDC_Device_t *const CDCInterfaceInfo)
{
	/* You can get changes to the virtual CDC lines in this callback; a common
	   use-case is to use the Data Terminal Ready (DTR) flag to enable and
	   disable CDC communications in your application when set to avoid the
	   application blocking while waiting for a host to become ready and read
	   in the pending data from the USB endpoints.
	*/
	bool HostReady = (CDCInterfaceInfo->State.ControlLineStates.HostToDevice & CDC_CONTROL_LINE_OUT_DTR) != 0;

	(void)HostReady;
}

/** Mass Storage class driver callback function the reception of SCSI commands from the host, which must be processed.
 *
 *  \param[in] MSInterfaceInfo  Pointer to the Mass Storage class interface configuration structure being referenced
 */
bool CALLBACK_MS_Device_SCSICommandReceived(USB_ClassInfo_MS_Device_t* const MSInterfaceInfo)
{
	bool CommandSuccess;

	LEDs_SetAllLEDs(LEDMASK_USB_BUSY);
	CommandSuccess = SCSI_DecodeSCSICommand(MSInterfaceInfo);
	LEDs_SetAllLEDs(LEDMASK_USB_READY);

	return CommandSuccess;
}

/** HID class driver callback function for the creation of HID reports to the host.
 *
 *  \param[in]     HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in,out] ReportID    Report ID requested by the host if non-zero, otherwise callback should set to the generated report ID
 *  \param[in]     ReportType  Type of the report to create, either HID_REPORT_ITEM_In or HID_REPORT_ITEM_Feature
 *  \param[out]    ReportData  Pointer to a buffer where the created report should be stored
 *  \param[out]    ReportSize  Number of bytes written in the report (or zero if no report is to be sent)
 *
 *  \return Boolean \c true to force the sending of the report, \c false to let the library determine if it needs to be sent
 */
bool CALLBACK_HID_Device_CreateHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
	uint8_t* const ReportID,
	const uint8_t ReportType,
	void* ReportData,
	uint16_t* const ReportSize)
{
	USB_KeyboardReport_Data_t* KeyboardReport = (USB_KeyboardReport_Data_t*)ReportData;

	uint8_t UsedKeyCodes = 0;
	uint8_t ButtonStatus_LCL = Buttons_GetStatus();

	if (ButtonStatus_LCL & BUTTONS_BUTTON1)
		KeyboardReport->KeyCode[UsedKeyCodes++] = HID_KEYBOARD_SC_F;

	if (UsedKeyCodes)
	{
		KeyboardReport->Modifier = HID_KEYBOARD_MODIFIER_LEFTSHIFT;
	}

	*ReportSize = sizeof(USB_KeyboardReport_Data_t);
	return false;
}

/** HID class driver callback function for the processing of HID reports from the host.
 *
 *  \param[in] HIDInterfaceInfo  Pointer to the HID class interface configuration structure being referenced
 *  \param[in] ReportID    Report ID of the received report from the host
 *  \param[in] ReportType  The type of report that the host has sent, either HID_REPORT_ITEM_Out or HID_REPORT_ITEM_Feature
 *  \param[in] ReportData  Pointer to a buffer where the received report has been stored
 *  \param[in] ReportSize  Size in bytes of the received HID report
 */
void CALLBACK_HID_Device_ProcessHIDReport(USB_ClassInfo_HID_Device_t* const HIDInterfaceInfo,
	const uint8_t ReportID,
	const uint8_t ReportType,
	const void* ReportData,
	const uint16_t ReportSize)
{
	uint8_t  LEDMask = LEDS_NO_LEDS;
	uint8_t* LEDReport = (uint8_t*)ReportData;

	if (*LEDReport & HID_KEYBOARD_LED_NUMLOCK)
		LEDMask |= LEDS_LED1;

	if (*LEDReport & HID_KEYBOARD_LED_CAPSLOCK)
		LEDMask |= LEDS_LED3;

	if (*LEDReport & HID_KEYBOARD_LED_SCROLLLOCK)
		LEDMask |= LEDS_LED4;

	LEDs_SetAllLEDs(LEDMask);
}

