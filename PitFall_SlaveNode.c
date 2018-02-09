/*********************************************************************************************
 * PitFall_SlaveNode Project Main File                                                       *
 *===========================================================================================*
 *  Created on: Sep 26, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 * This is the main file of the project. It initializes every needed subsystem of the micro- *
 * controller and starts the TI-RTOS BIOS.                                                   *
 *                                                                                           *
 * Hardware Description                                                                      *
 *----------------------                                                                     *
 * The hardware of a slave node is developed based on a CC1310-LaunchXL Texas Instruments'   *
 * Launchpad. It uses its buttons for user actions and its leds as indicators. Its UART is   *
 * used for communication to the MSP430 subsystem of the final trap. The connections to the  *
 * buttons and the leds (I/O pins) can be seen at Board.h file in the root dorectory of this *
 * project. The development environment used is Code Composer Studio 6.2.0 but also tested   *
 * in Code Composer Studio 7.4.0 and run fine. In brief:                                     *
 * -DIO_2 (package pin 7): UART_RX, Data from MSP430 (Input)                                 *
 * -DIO_3 (package pin 8): UART_TX, Data to MSP430 (Output)                                  *
 * -DIO_6 (package pin 11): Red Led (Output), indicates a fault condition when on, or UART   *
 *   activity (communication with MSP430) when flashing.                                     *
 * -DIO_7 (package pin 12): Green Led (Output), indicates RF activity.                       *
 * -DIO_13 (package pin 19): Left button (Input), when pressed, the system simulates an      *
 *   event record coming through UART from MSP and adds it to the events queue to be sent to *
 *   the master node. It's used for UART and RF communication debugging purposes.            *
 * -DIO_14 (package pin 20): Right button (Input), used for registering the node to a master *
 *   network (long press), or synchronizing the communication window to the master network   *
 *   (short press)                                                                           *
 *                                                                                           *
 * Software Description                                                                      *
 *----------------------                                                                     *
 * The software is based on TI-RTOS and EasyLink Framework. It contains two tasks, one for   *
 * handling the UART communication with the MSP430 subsystem (optical sensor and recognition)*
 * and another for handling the RF communications. The latter is based on EasyLink framework *
 * provided by Texas Instruments, slightly modified to also perform a kind of CSMA/CA for    *
 * RF communication collision avoidance. More information about these two is provided in the *
 * respective task files.                                                                    *
 * TI_RTOS handles all the necessary power conditions and provides an ultra low power        *
 * environment for battery power applications, as this one.                                  *
 * TI-RTOS uses RTC clock for its own purpose, so our code should not alter its settings.    *
 * The application needs RTC clock also, so it uses it in a way to not disturb the function  *
 * overall behavior of TI-RTOS. More on this in the respective files for RTC handling.       *
 *                                                                                           *
 *                                                                                           *
 * Project File Tree                                                                         *
 *-------------------                                                                        *
 * /                                                                                         *
 * |- CC1310_LAUNCHXL.cmd                   Linker command file, provided by Texas           *
 * |                                         Instruments.                                    *
 * |- CC1310_LAUNCHXL.h                     Header and source files for Launchpad helping    *
 * |- CC1310_LAUNCHXL.c                      initialization of the system. These files are   *
 * |                                         provided by Texas Instruments. No need to alter *
 * |                                         anything there.                                 *
 * |- ccfg.c                                Cuctomer Configuration file. This file is also   *
 * |                                         provided by Texas Instruments containing the    *
 * |                                         default settings. User may need to alter some   *
 * |                                         ccfg register. In our case, no need to alter    *
 * |                                         any of these settings.                          *
 * |- Board.h                               Board file declaring the pins used at the PCB    *
 * |- PitFall_SlaveNode.cfg                 Configuration file used for XDC Tools to         *
 * |                                         configure TI-RTOS subsystems.                   *
 * |- PitFall_SlaveNode.h                   Header and source files of the main program.     *
 * |- PitFall_SlaveNode.c                                                                    *
 * |- GenericFuncs.h                        Some generic functions needed throughout the     *
 * |- GenericFuncs.c                         whole application, no matter which task.        *
 * |- UART2MSP.h                            Header and source files for the UART task, used  *
 * |- UART2MSP.c                             to control the communication to the MSP430      *
 * |                                        hardware subsystem (optical sensor)              *
 * |-Flash                                                                                   *
 * |  |- AppFlash.h                         Header and source files for manipulating the     *
 * |  `- AppFlash.c                          Flash memory                                    *
 * |-RF                                                                                      *
 * |  |-EasyLink                                                                             *
 * |  |  |- EasyLink.h                      Header and source files provided by Texas        *
 * |  |  `- EasyLink.c                       Instruments. This is the EasyLink framework the *
 * |  |                                      RF communication is based on. The source file   *
 * |  |                                      slightly altered to provide a kind of CSMA/CA.  *
 * |  |-SmartRF_Settings                                                                     *
 * |  |  |- CSMACA_settings.h               Header and source files that provide the commands*
 * |  |  |- CSMACA_settings.c                to the RF core of the microcontroller for       *
 * |  |  |                                   CSMA/CA.                                        *
 * |  |  |- smartrf_settings_predefined.h   Header and source files for the predefined       *
 * |  |  |- smartrf_settings_predefined.c    commands to be issued to the RF core. They are  *
 * |  |  |                                   provided by SmartRF application.                *
 * |  |  |- smartrf_settings.h              Header and source files for specific commands    *
 * |  |  `- smartrf_settings.c               needed to be issued to the RF core of the micro-*
 * |  |                                      controller. They are also provided by SmartRF   *
 * |  |                                      application of Texas Instruments.               *
 * |  |- RadioProtocol.h                    Structures describing all the possible packets   *
 * |  |                                      used for the RF communication between the Slave *
 * |  |                                      and the Master nodes.                           *
 * |  |- RadioTask.h                        The task of the application that handles RF      *
 * |  `- RadioTask.c                         communications.                                 *
 * `-RTC                                                                                     *
 *    |- RTCFuncs.h                         Header and source files to handle the RTC clock  *
 *    `- RTCFuncs.c                          of the microcontroller. The RTC is also used by *
 *                                           TI-RTOS, so special handling must be used to    *
 *                                           not disturb the operating system's functioning. *
 *                                                                                           *
 * DISCLAMER NOTICE:                                                                         *
 *-------------------                                                                        *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY       *
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF   *
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL    *
 * THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,       *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) *
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR  *
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                              *
 ********************************************************************************************/

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
/* XDCtools Header files */
#include <RF/RadioTask.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/power/PowerCC26XX.h>

/* TI-RTOS Header files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>

/* Board Header files */
#include <Board.h>
#include "PitFall_SlaveNode.h"
#include "Flash/AppFlash.h"
#include "UART2MSP.h"


/*********************************************************************************************
 * Variables definitions                                                                     *
 ********************************************************************************************/
/*Array of events from MSP and its helper variables.*/
EventStamp EventsArray[EVENTSARRAY_SIZE];	//Array that holds the timestamps MSP has recorded
NodePars NodeParams;						//Node's parameters from MSP
uint8_t EventsStrt;							//Starting index for the first event in queue
uint8_t EventsLen;							//Number of events recorded in the EventsArray

/*Pin driver handles.*/
PIN_Handle ledPinHandle;					//Handle for LEDs manipulation
PIN_State ledPinState;						//State of pins that drive the LEDs
PIN_Handle swPinHandle;						//Handle for Key Switches
PIN_State swPinState;						//State of pins that are driven by key switches

/*Callback functions for board switch buttons.*/
CallbackFunc KeyLeftCb;						//Callback function of Left Button press
CallbackFunc KeyRightCb;					//Callback function of Right Button press


/*********************************************************************************************
 * Constants definitions                                                                     *
 ********************************************************************************************/
/*Table of LEDs of the board.*/
const PIN_Config ledPinTable[] = {
	BoardLedR | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	BoardLedG | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
	PIN_TERMINATE
};

/*Table of switch buttons of the board.*/
const PIN_Config swPinTable[] = {
	BUTTON_L	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	BUTTON_R	| PIN_INPUT_EN		| PIN_PULLUP	| PIN_IRQ_NEGEDGE	| PIN_BM_HYSTERESIS,
	PIN_TERMINATE
};


/*********************************************************************************************
 * Function declarations                                                                     *
 ********************************************************************************************/
void ButtonCbFxn(PIN_Handle handle, PIN_Id pinId);	//Callback function for RxD interrupt


/*********************************************************************************************
 * Main code                                                                                 *
 ********************************************************************************************/
//********************************************************************************************
/*Lights up the red led to inform for error condition and enters an infinite loop disabling
interrupts. The input variable points to an explanation string for the error condition. The
function never returns.*/
void Halt_abort(char* ErrorTxt, void* ptr) {
	if(ledPinHandle) {
		PIN_setOutputValue(ledPinHandle, BoardLedR, 1);
	}
	Hwi_disable();
	while(1);
}

//********************************************************************************************
/*Callback function for key presses. This function is a dispatcher, so every other task can
use the buttons.*/
void ButtonCbFxn(PIN_Handle handle, PIN_Id pinId) {
	if((pinId == BUTTON_L) && (KeyLeftCb != NULL)) {
		KeyLeftCb();
	}
	if((pinId == BUTTON_R) && (KeyRightCb != NULL)) {
		KeyRightCb();
	}
}


//********************************************************************************************
/*Sets callback functions for a key button
Input:
	pinId: is the ID of the button to be used. Can be one of BUTTON_L, BUTTON_R.
	cbFxn: is the callback function to be used. It is of type CallbackFunc.
*/
void registerButtonCb(PIN_Id pinId, CallbackFunc cbFxn) {
	switch(pinId) {
	case BUTTON_L:
		KeyLeftCb = cbFxn;
		break;

	case BUTTON_R:
		KeyRightCb = cbFxn;
		break;
	}
}


//********************************************************************************************
/*Main Function of the whole application.                                                    *
*===========================================================================================*/
int main(void)
{
	//Call board initialization functions.
	Board_initGeneral();					//Initialize the board pins
	Board_initUART();						//Initialize UART pins
	InitFlash();							//Initialize the Flash memory subsystem

	/*Open LEDs. The application normally does not use LEDs but during development they are a
	great feedback :)*/
	ledPinHandle = PIN_open(&ledPinState, ledPinTable);
	if(!ledPinHandle) {
		Halt_abort("Error initializing board LED pins\n", &ledPinState);
	}

	/*Open Switch pins and reset switches callback functions. Finally, register the callback
	function for interrupts generated by switches.*/
	swPinHandle = PIN_open(&swPinState, swPinTable);
	if(!swPinHandle) {
		Halt_abort("Error initializing board switch pins\n", &swPinState);
	}
	KeyLeftCb = NULL;						//No callback function for Left key button, yet
	KeyRightCb = NULL;						//No callback function for Right key button, yet
	PIN_registerIntCb(swPinHandle, ButtonCbFxn);	//Bind the interrupt function of the keys

	/*Initialize other tasks of the application.*/
	InitRFTask();							//Initialize the RF Task
	InitUARTTask();							//Initialize the UART task for communication to
											// MSP processor of the trap

	BIOS_start();							//Start BIOS

	return (0);								//Normally the code never reaches this statement
}
