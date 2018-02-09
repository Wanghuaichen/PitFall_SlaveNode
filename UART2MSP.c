/*********************************************************************************************
 * UART2MSP.c                                                                                *
 *===========================================================================================*
 *  Created on: Oct 12, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 ********************************************************************************************/

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Event.h>

/* TI-RTOS Driver files */
#include <ti/drivers/PIN.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTCC26XX.h>

/* Board Header files */
#include <Board.h>

/* Include other application header files */
#include "PitFall_SlaveNode.h"
#include "RTC/RTCFuncs.h"
#include "RF/RadioTask.h"
#include "UART2MSP.h"

/* Include Standard C libraries header files*/
#include <string.h>


/*********************************************************************************************
 * Definitions to control the task behavior                                                  *
 ********************************************************************************************/
#define UART_TASK_STACK_SIZE	1024
#define UART_TASK_PRIORITY		3

#define MSPBUFFER_SIZE			32


/*********************************************************************************************
 * New types definitions                                                                     *
 ********************************************************************************************/
/*MSPStamp defines the meaning of each byte that comes from MSP part of the design through the
UART port. After synchronization of UART to the incoming stream of data and the command, the
following bytes have the meaning defined in this typedef. More specific, the incoming packet
contains two sections (not separated by anything). The first one is the EventStamp and it
contains:
	1 Byte for the Year (interval from 2000)
	1 Byte for the Type of the event (temperature, bug, or reset) at it high nibble and the
		Month (1 based)
	1 Byte for the Day of the month (1 based)
	1 Byte for the Hour of the event
	1 Byte for the Minute of the event
	1 Byte for the Second of the event
	2 Bytes for the temperature ADC value, low byte first
The second section contains data that do not really belong to the event, but are parameters
the master node needs to know about:
	2 Bytes for the calibration value of the MSP temperature sensor at 30 degrees, low byte
		first. This is the constant value CALADC12_15V_30C
	2 Bytes for the calibration value of the same sensor at 85C (CALADC12_15V_85C), low byte
		first
	2 Bytes for the MasterCounter, low byte first
	2 Bytes for the Battery ADC value, low byte first
 */
typedef struct {
	EventStamp stmp;
	NodePars pars;
} MSPStamp;


/*********************************************************************************************
 * Variable definitions                                                                      *
 ********************************************************************************************/
//Main task for manipulating the UART communication to the MSP microcontroller of the device
static Task_Params UARTTaskParams;
Task_Struct UARTTask;
static uint8_t UARTTaskStack[UART_TASK_STACK_SIZE];

/*The task needs an event to be notified for events and synchronize its states.*/
Event_Struct UARTEvent;
static Event_Handle UARTEventHandle;

/*The system communicates to the module using the UART of the processor.
 * Here are the necessary variables */
UART_Params MSPUartPars;					//Parameters of the UART subsystem
UART_Handle MSPUartHandle;					//Handle of the UART object
UARTCC26XX_Object *UartParams;				//Need to tweak UART parameters...
uint8_t MSPInBuffer[MSPBUFFER_SIZE];		//Buffer that accepts the UART input stream

//Pins used for UART Task's job
PIN_Handle RxPinHandle;
PIN_State RxPinState;


/*********************************************************************************************
 * Constants definitions                                                                     *
 ********************************************************************************************/
//Tables of pins used, etc.
const PIN_Config RxPinTable[] = {
	MSP_RxD	| PIN_INPUT_EN		| PIN_NOPULL	| PIN_IRQ_NEGEDGE,
	PIN_TERMINATE
};

//Messages returned to MSP as Ack/NAck
const char* MSPMess[] = {
	"OK\r\n",
	"SAME\r\n",
	"NOMEM\r\n",
	"NACK\r\n",
	"RESET\r\n",
	"RTC:\r\n",
	"ACK\r\n",
	"INFO OK\r\n"
};
#define MESS_OK		0
#define MESS_VOK	1
#define MESS_MEM	2
#define MESS_ERR	3
#define MESS_RES	4
#define MESS_RTC	5
#define MESS_ACK	6
#define MESS_INFO	7


/*********************************************************************************************
 * Function declarations                                                                     *
 ********************************************************************************************/
void cbRxInt(PIN_Handle handle, PIN_Id pinId);//Callback function for tracking MSP RxD state
static void UARTTaskFxn(UArg arg0, UArg arg1);//The task function for the UART task
void InitRxAsPin(void);						//UART Rx pin is set as normal I/O with interrupt
											// capability
void UseRxAsUART(void);						//UART Rx pin is set as UART function


/*********************************************************************************************
 * Helper Functions                                                                          *
 ********************************************************************************************/
void cbRxInt(PIN_Handle handle, PIN_Id pinId) {
	uint_t key;

	if(pinId == MSP_RxD) {
		key = PIN_getInputValue(MSP_RxD);
		if(key == 0) {
			PIN_setInterrupt(handle, MSP_RxD | PIN_IRQ_DIS);
			Event_post(UARTEventHandle, UARTEVENT_START);
		}
	}
	PIN_clrPendInterrupt(handle, pinId);
}


void InitRxAsPin(void) {
	if(MSPUartHandle) {
		UART_close(MSPUartHandle);
		MSPUartHandle = NULL;
	}
	RxPinHandle = PIN_open(&RxPinState, RxPinTable);
	if(!RxPinHandle) {
		Halt_abort("Error initializing board UART pins\n", &RxPinState);
	}
	PIN_registerIntCb(RxPinHandle, cbRxInt);
	PIN_setOutputValue(ledPinHandle, UARTACTIVITY_LED, 0);
}


void UseRxAsUART(void) {
	if(RxPinHandle) {
		PIN_close(RxPinHandle);
		RxPinHandle = NULL;
	}
	MSPUartHandle = UART_open(0, &MSPUartPars);
	if(!MSPUartHandle) {
		Halt_abort("Error opening the MSP UART\n", NULL);
	}
	PIN_setOutputValue(ledPinHandle, UARTACTIVITY_LED, 1);
	UartParams = (UARTCC26XX_Object*) MSPUartHandle->object;
//	UART_control(MSPUartHandle, UARTCC26XX_CMD_RETURN_PARTIAL_ENABLE, NULL);
}


void TriggerReset(void) {
	Event_post(UARTEventHandle, UARTEVENT_RESET);
}


void TriggerRTC(void) {
	Event_post(UARTEventHandle, UARTEVENT_RTC);
}


void TriggerData(void) {
	if((RFStatusFlags & RFS_RTCOK) != 0) {
		Event_post(UARTEventHandle, UARTEVENT_DATA);
	}
}


void TriggerAck(uint32_t InVal) {
	Event_post(UARTEventHandle, InVal);
}


void InitUARTTask(void) {

    /* Create event used internally for state changes */
    Event_Params eventParam;
    Event_Params_init(&eventParam);
    Event_construct(&UARTEvent, &eventParam);
    UARTEventHandle = Event_handle(&UARTEvent);

    /* Create the UART task */
    Task_Params_init(&UARTTaskParams);
    UARTTaskParams.stackSize = UART_TASK_STACK_SIZE;
    UARTTaskParams.priority = UART_TASK_PRIORITY;
    UARTTaskParams.stack = &UARTTaskStack;
    Task_construct(&UARTTask, UARTTaskFxn, &UARTTaskParams, NULL);

    /*Need to use interrupt from RxD pin of UART. The idea is to have UART powered down to
    preserve power and enable it only if there is activity from MSP serial port. So, in the
    beginning, the RxD port of the UART module is used as I/O with interrupt enabled. When the
    Start Bit of the communication comes, it fires an interrupt and the pin is reverted to
    UART function and the UART is enabled. When the UART activity ends and all characters read
    from the input stream, the UART is deactivated again and the RxD pin is reverted back to
    normal I/O with interrupts enabled.*/
	//Lets initialize all the pins used for the UART task
//	RxPinHandle = PIN_open(&RxPinState, RxPinTable);
//	if(!RxPinHandle) {
//		System_abort("Error initializing board GSM pins\n");
//	}
//	PIN_registerIntCb(RxPinHandle, cbRxInt);
//  InitRxAsPin();

	/*In order to store an event in the events array the UART task receives events from MSP
	through UART. If the event that came is the same as the last one stored, it is ignored.
	For that reason element 0 of EventsArray is reset and treated as "The last event stored"
	and the first incoming event is stored in element 1 of this array. The maximum number of
	stored events in the EvantsArray is (EVENTSARRAY_SIZE -1), so the last event is always in
	memory and can be compared to the incoming one, even when all the timestamps have been
	sent to the master node.*/
	//Lets clear the first element of the Events array.
	EventsArray[0].Year = 0;
	EventsArray[0].TypeMonth = 0xF0;
	EventsArray[0].Day = 0;
	EventsArray[0].Hour = 0;
	EventsArray[0].Minute = 0;
	EventsArray[0].Second = 0;
	EventsArray[0].Temperature = 0;
	EventsStrt = 1;

	UART_init();							//Initialize the UART module
	UART_Params_init(&MSPUartPars);			//Initialize the UART parameters
	/*The following commented out lines are the default ones, or for testing purposes */
//	MSPUartPars.baudRate = 115200;
//	MSPUartPars.dataLength = UART_LEN_8;
//	MSPUartPars.parityType = UART_PAR_NONE;
//	MSPUartPars.stopBits = UART_STOP_ONE;
//	MSPUartPars.readReturnMode = UART_RETURN_NEWLINE;
	MSPUartPars.readEcho = UART_ECHO_OFF;	//No need to echo back the input characters
	MSPUartPars.readTimeout = 100000 /Clock_tickPeriod;	//Means 100m second
}


/*********************************************************************************************
 * MainTask Function                                                                         *
 ********************************************************************************************/
static void UARTTaskFxn(UArg arg0, UArg arg1) {
	UInt ev;								//Event received
	int InChrs;								//Number of characters received
	MSPStamp* EvStmpPtr;					//Pointer to MSP Input Buffer as EventStamp
	char* InEvChar;							//Pointer to bytes for comparing incoming event
	char* LastEvChar;						//Pointer to bytes for comparing last stored event
	RTCStruct CurrTime;						//RTC Structure that will hold the current time
	uint32_t i;								//Helper variable

	RTCInit();
	registerButtonCb(UARTEVENT_BUTTON, TriggerData);
	InitRxAsPin();							//Rx pin is used as a normal I/O to sense activity

	while(1) {								//Task loop starts here
		ev = Event_pend(UARTEventHandle, Event_Id_NONE, UARTEVENT_ALL, BIOS_WAIT_FOREVER);
		UseRxAsUART();
		if((ev & (UARTEVENT_START | UARTEVENT_DATA)) != 0) {
			if((ev & UARTEVENT_START) != 0) {
				InChrs = UART_read(MSPUartHandle, MSPInBuffer, MSPBUFFER_SIZE);
				while((UartParams->status != UART_OK) && (UartParams->status != UART_TIMED_OUT)) {
					InChrs = UART_read(MSPUartHandle, MSPInBuffer, MSPBUFFER_SIZE);
				}
			} else {
				MSPInBuffer[0] = 'S';
				MSPInBuffer[1] = ':';
				EvStmpPtr = (MSPStamp*)&MSPInBuffer[2];
				EvStmpPtr->pars.CALADC12_15V_30C = 10;
				EvStmpPtr->pars.CALADC12_15V_85C = 2000;
				EvStmpPtr->pars.Battery = 0xEC;
				EvStmpPtr->pars.MasterCounter = 25;
				RTCtoStruct(RTCGetCurrTime(), &CurrTime);
				EvStmpPtr->stmp.Year = CurrTime.Year;
				EvStmpPtr->stmp.TypeMonth = CurrTime.Month | (1 << 4);
				EvStmpPtr->stmp.Day = CurrTime.Day;
				EvStmpPtr->stmp.Hour = CurrTime.Hour;
				EvStmpPtr->stmp.Minute = CurrTime.Minute;
				EvStmpPtr->stmp.Second = CurrTime.Second;
				EvStmpPtr->stmp.Temperature = 0xEC;
				InChrs = 2 + sizeof(MSPStamp);
			}
			if(InChrs > 0) {
				for(i = 0; i < InChrs; i++) {
					if(MSPInBuffer[i] == ':') {
						break;
					}
				}
				if((i < (InChrs -sizeof(MSPStamp))) && (i > 0) && (MSPInBuffer[i -1] == 'S')) {
					if(EventsLen < (EVENTSARRAY_SIZE -1)) {
						EvStmpPtr = (MSPStamp*)&MSPInBuffer[i +1];
						InEvChar = (char*)&MSPInBuffer[i +1];
						i = EventsStrt + EventsLen -1;
						if(i >= EVENTSARRAY_SIZE) {
							i -= EVENTSARRAY_SIZE;
						}
						LastEvChar = (char*)&EventsArray[i];
						for(InChrs = 0; InChrs < sizeof(EventStamp); InChrs++) {
							if(InEvChar[InChrs] != LastEvChar[InChrs]) {
								break;
							}
//							InEvChar++;
//							LastEvChar++;
						}
						if(InChrs != sizeof(EventStamp)) {
							i++;
							if(i >= EVENTSARRAY_SIZE) {
								i -= EVENTSARRAY_SIZE;
							}
							memcpy(&EventsArray[i], EvStmpPtr, sizeof(EventStamp));
							NodeParams.CALADC12_15V_30C = EvStmpPtr->pars.CALADC12_15V_30C;
							NodeParams.CALADC12_15V_85C = EvStmpPtr->pars.CALADC12_15V_85C;
							NodeParams.MasterCounter = EvStmpPtr->pars.MasterCounter;
							NodeParams.Battery = EvStmpPtr->pars.Battery;
							EventsLen++;
							UART_write(MSPUartHandle, MSPMess[MESS_OK], 4);
						} else {
							UART_write(MSPUartHandle, MSPMess[MESS_VOK], 6);
						}
					} else {
						UART_write(MSPUartHandle, MSPMess[MESS_MEM], 7);
					}
				} else {
					UART_write(MSPUartHandle, MSPMess[MESS_ERR], 6);
				}
			}
		}

		if((ev & UARTEVENT_RESET) != 0) {
			UART_write(MSPUartHandle, MSPMess[MESS_RES], 7);
		}

		if((ev & UARTEVENT_RTC) != 0) {
			i = RTCGetCurrTime();
			RTCtoStruct(i, &CurrTime);
			UART_write(MSPUartHandle, MSPMess[MESS_RTC], 4);
			UART_write(MSPUartHandle, (void *)&CurrTime, 6);
			UART_write(MSPUartHandle, MSPMess[MESS_RTC] +4, 2);
		}

		if((ev & UARTEVENT_ACK) != 0) {
			UART_write(MSPUartHandle, MSPMess[MESS_ACK], 5);
		}

		if((ev & UARTEVENT_INFO) != 0) {
			UART_write(MSPUartHandle, MSPMess[MESS_INFO], 9);
		}

		InitRxAsPin();
	}
}

