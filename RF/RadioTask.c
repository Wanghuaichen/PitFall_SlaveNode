/*********************************************************************************************
 * RadioTask.c                                                                               *
 *===========================================================================================*
 *  Created on: Oct 12, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 ********************************************************************************************/

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/hal/Hwi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Clock.h>

/* TI-RTOS Driver files */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>

/* Board Header files */
#include <Board.h>
#include "PitFall_SlaveNode.h"

/* Include other application header files */
#include <RF/RadioProtocol.h>
#include <RF/RadioTask.h>
#include "RF/EasyLink/EasyLink.h"
#include "Flash/AppFlash.h"
#include "RTC/RTCFuncs.h"
#include "UART2MSP.h"
#include "GenericFuncs.h"


/*********************************************************************************************
 * Definitions to control the task behavior                                                  *
 ********************************************************************************************/
#define RFTASK_STACK_SIZE 1024
#define RFTASK_PRIORITY   2

#define RFEVENT_ALL							0xFFFFFFFF
//#define RFEVENT_START						Event_Id_00
#define RFEVENT_BEACON						Event_Id_01
#define RFEVENT_RTC							Event_Id_02
#define RFEVENT_SYNC						Event_Id_03
#define RFEVENT_REGISTER					Event_Id_04
#define RFEVENT_REGOK						Event_Id_05
#define RFEVENT_ACK							Event_Id_06
#define RFEVENT_EXTENDTIME					Event_Id_07
#define RFEVENT_INFO						Event_Id_08
#define RFEVENT_VALID_PACKET_RECEIVED		Event_Id_29
#define RFEVENT_INVALID_PACKET_RECEIVED		Event_Id_30
#define RFEVENT_SLEEP						Event_Id_31

#define RFACTIVITY_TIMEOUT					(115 * 1000000 / Clock_tickPeriod)
#define RFSYNC_TIMEOUT						(5 * 1000000 / Clock_tickPeriod)
#define REGISTER_TIMEOUT					(30 * 1000000 / Clock_tickPeriod)
#define LONGPRESS_TIMEOUT					(3 * 1000000 / Clock_tickPeriod)
#define RFACK_TIMEOUT						(160 * 1000 / Clock_tickPeriod)
#define LEDON_TIMEOUT						(10 * 1000 / Clock_tickPeriod)
#define LEDOFF_TIMEOUT						(990 * 1000 / Clock_tickPeriod)

#define RF_MAX_RETRIES	3					//Number of maximum retries in case of erroneous
											// transmission


/*********************************************************************************************
 * New types definitions                                                                     *
 ********************************************************************************************/
typedef enum {
	SENDEVENTS_NEW,
	SENDEVENTS_RETRY
} SEFlag;

typedef enum {
	SENDEVENTS_OK,
	SENDEVENTS_EXHAUSTED
} SEStatus;

/*********************************************************************************************
 * Variable definitions                                                                      *
 ********************************************************************************************/
//Main task variables to control the RF activity
static Task_Params RFTaskParams;			//RF Task parameters
Task_Struct RFTask;							//RF Task structure
static uint8_t RFTaskStack[RFTASK_STACK_SIZE];//RF Task stack

//The event object that controls and synchronizes the activity of the RF task
Event_Struct RFEvent;						//RF Event Structure
Event_Handle RFEventHandle;					//RF Event Handle

//Clock object for triggering the start and stop of the RF activity
Clock_Handle RFActivityClk;					//Activity clock handle
//Clock_Params RFActivityPars;				//Activity clock parameters

//Clock object for packet acknowledge reception timeout
Clock_Handle RFTimeoutClk;					//Timeout for packet reception clock handle
//Clock_Params RFTimeoutPars;				//Timeout for packet reception clock parameters

//Clock object for Long press of the button to trigger registration to a master's network
Clock_Handle RFLongPressClk;				//Timeout for packet reception clock handle
//Clock_Params RFLongPressPars;				//Timeout for packet reception clock parameters

//Clock object for turning off the RF led to create a flashing effect
Clock_Handle RFFlashClk;					//Timeout for Led flashing
//Clock_Params RFLongPressPars;				//Timeout for packet reception clock parameters

volatile uint32_t RFStatusFlags;			//Flags to show the status of the RF activity
uint8_t ValidAddrs[2];						//Acceptable addresses for the RF module packets


static union RFPacket latestRxPacket;		//The latest packet received (not beacon)
static union RFPacket latestTxPacket;		//The latest packet transmitted (not registration)
struct BeaconPacket latestBeacon;			//The latest beacon packet received
struct RegisterNodePacket registerPacket;	//A registration packet to be sent to a master
struct GetInfoPacket infoPacket;			//A Get Information packet to be sent to a master
uint32_t BLastTime;							//Beacon's last time
uint32_t BeaconEnd;							//When current beacon session ends
static EasyLink_TxPacket txPacket;			//Packet to be transmitted
uint16_t RFRestart;							//When is the next beacon session from master
uint8_t CurrSession;						//The current communication session
uint8_t CurrPacket;							//The current data packet in this session
uint8_t RetriesCnt;							//Retries counter for NAck'ed data packets


/*********************************************************************************************
 * Constants definitions                                                                     *
 ********************************************************************************************/


/*********************************************************************************************
 * Function declarations                                                                     *
 ********************************************************************************************/
void RFActivityFxn(UArg InArg);				//Starts the RF activity of the node
void RFLongPressFxn(UArg InArg);			//Counts the long press of a button
void RFTimeoutFxn(UArg InArg);				//Counts the RF ACK reception timeout
void RFFlashFxn(UArg InArg);				//Counts the led flashing timeout
void TriggerRF(void);						//Starts RF in SCAN mode
static void SendReg(void);					//Sends a registration request packet
static SEStatus SendEvents(SEFlag flag);	//Sends a packet of events
static void SendNodePars(void);				//Sends a packet requesting node's parameters
static void SendAckBack(uint8_t packetSession, uint8_t packetNo);
											//Sends an Ack back to the master
static void RFTaskFxn(UArg arg0, UArg arg1);//Main task function
static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status);


/*********************************************************************************************
 * Helper Functions                                                                          *
 ********************************************************************************************/
void RFActivityFxn(UArg InArg) {
	if((RFStatusFlags & RFS_ONAIR) != 0) {
		//Running? then we need to sleep if not in Sync, or extend the timeout
		Event_post(RFEventHandle, ((RFStatusFlags & RFS_SYNC) != 0)?
			RFEVENT_EXTENDTIME : RFEVENT_SLEEP);
	} else {
		//Not running? then we need to start
		Event_post(RFEventHandle, RFEVENT_START);
	}
}


void RFLongPressFxn(UArg InArg) {
	if((RFStatusFlags & RFS_REGISTER) == 0) {
		Event_post(RFEventHandle, RFEVENT_REGISTER);
	}
}


void RFTimeoutFxn(UArg InArg) {
	//Clear the "timeout clock in use" flag and the flags that set the state machine in
	// "Expecting Answer" mode
	RFStatusFlags &= ~(RFS_TIMEOUTRUN | RFS_DATASEND | RFS_INFOSEND | RFS_REGSEND |
		RFS_PARAMSREQ);
}


void RFFlashFxn(UArg InArg) {
	Clock_stop(RFFlashClk);
	if((PIN_getInputValue(RFACTIVITY_LED) == 1) || (InArg == 2)) {
		PIN_setOutputValue(ledPinHandle, RFACTIVITY_LED, 0);
		Clock_setTimeout(RFFlashClk, ((RFStatusFlags & RFS_REGISTER)!= 0) ?
			(LEDOFF_TIMEOUT /2) : LEDOFF_TIMEOUT);
	} else {
		PIN_setOutputValue(ledPinHandle, RFACTIVITY_LED, 1);
		Clock_setTimeout(RFFlashClk, LEDON_TIMEOUT);
	}
	if(InArg != 2) {
		Clock_start(RFFlashClk);
	}
}


void TriggerRF(void) {
	if(PIN_getInputValue(RFBUTTON) == 0) {	//Make key
		//Need to post a SYNC event, only if the RF module is off
		if((RFStatusFlags & RFS_ONAIR) == 0) {
			Event_post(RFEventHandle, RFEVENT_SYNC);
		}
		Clock_start(RFLongPressClk);		//Start long press timeout counter
	} else {								//Break key
		Clock_stop(RFLongPressClk);			//Stop long press timeout counter
	}
}


void InitRFTask(void) {
	Clock_Params RFClockPars;				//Clock parameters for created clock objects

	//Create event used internally for state changes.
	Event_Params eventParam;
	Event_Params_init(&eventParam);
	Event_construct(&RFEvent, &eventParam);
	RFEventHandle = Event_handle(&RFEvent);

	//Create the radio protocol task.
	Task_Params_init(&RFTaskParams);
	RFTaskParams.stackSize = RFTASK_STACK_SIZE;
	RFTaskParams.priority = RFTASK_PRIORITY;
	RFTaskParams.stack = &RFTaskStack;
	Task_construct(&RFTask, RFTaskFxn, &RFTaskParams, NULL);

	/*The system needs a clock that will start the RF reception just before master is expected
	to start sending beacons.*/
	Clock_Params_init(&RFClockPars);
	RFClockPars.startFlag = false;
	RFClockPars.arg = 0;
	/*The real timeout of this clock object varies according to the time the needed data
	transaction is finished. Its purpose is to activate the RX just a few seconds before the
	master node starts sending beacons. Besides that, in here we set a default value that the
	main task function will alter when necessary.*/
	RFActivityClk = Clock_create(RFActivityFxn, RFACTIVITY_TIMEOUT, &RFClockPars, NULL);

	/*The system needs a clock that will count the long pressing of a button. When a button is
	pressed, it activates the SYNC mode. After a LONGPRESS_TIMEOUT time the clock expires and
	a long press of that button is considered.*/
	RFLongPressClk = Clock_create(RFLongPressFxn, LONGPRESS_TIMEOUT, &RFClockPars, NULL);

	/*Another needed clock is the one that defines the timeout of an ACK packet reception.
	When this clock expires, the already sent packet is considered lost and a re-transmission
	is scheduled.*/
	RFTimeoutClk = Clock_create(RFTimeoutFxn, RFACK_TIMEOUT, &RFClockPars, NULL);

	/*The following clock object is used for RF Activity LED flashing. When the RX is active
	the RF Activity led flashes. The flashing effect keeps the LED on for LEDON_TIMEOUT time
	and keeps the LED off for LEDOFF_TIMEOUT time. The timeout of the timer alternates between
	these two values.*/
	RFFlashClk = Clock_create(RFFlashFxn, LEDON_TIMEOUT, &RFClockPars, NULL);

	/*Reset the valid addresses table. This table contains the broadcast address and the node
	address, in order for the node to accept both broadcast packets and packets targeted to
	it, specifically.*/
	ValidAddrs[0] = 0;
	ValidAddrs[1] = NodeAddr;

	/*Lets prepare a registration packet just in case the user needs to register to another
	network and a data packet that will be used to send events. A Get information packet is
	also needed to be sent every time there are no data events to be sent, in order for the
	node to get the parameters set by the server.*/
	registerPacket.header.packetType = RFPacket_Register;
	registerPacket.header.sourceAddress = 0;
	EasyLink_getIeeeAddr(registerPacket.NodeMAC);
	latestTxPacket.rfData.header.packetType = RFPacket_Data;
	latestTxPacket.rfData.header.sourceAddress = NodeAddr;
	latestTxPacket.rfData.packetSession = 0;
	latestTxPacket.rfData.GSMCounter = 0;
	//Need to also fill in the MasterMAC and CRC fields. Lets prepare the Get info packet
	infoPacket.header.packetType = RFPacket_GetInfo;
	infoPacket.header.sourceAddress = NodeAddr;
	memcpy(&infoPacket.NodeMAC, &registerPacket.NodeMAC, 8);

	//Initialize variables
	RFRestart = DEF_RFRESTART;
	RFStatusFlags = 0;
	CurrSession = 0;
	CurrPacket = 0;
}


static void SendReg(void) {
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint8_t i;								//Helper counter for "for" loops
	uint8_t* ptr;							//Pointer to manipulate the raw bytes of a packet
	EasyLink_Status stat;					//Status of EasyLink commands

	//Set the destination address of the packet.
	txPacket.dstAddr[0] = DEF_MASTER_ADDRESS;
	/*Master expects its MAC address in the correct field of the packet to ensure that the
	registration packet is targeted to it.*/
	memcpy(registerPacket.MasterMAC, latestBeacon.SourceMAC, 8);
	ptr = (uint8_t*)&registerPacket.header;
	//Create the CRC of the packet
	CRCCheck = DEF_CRC16;	//Initialize CRC Checksum
	for(i = 0; i < RFREG_SIZE; i++) {
		if(i == RFHEAD_SIZE) {//If i points to CRC value => ...
			i += RFCRC_SIZE;// ... then skip it. It is not included in final
							// CRC Checksum
		}
		CRCCheck = CalcCRC16(ptr[i], CRCCheck);
	}
	registerPacket.CRC = CRCCheck;
	/*Copy Registration packet to payload, skipping the destination address byte. Note that
	the EasyLink API will implicitly both add the length byte and the destination address.*/
	memcpy(txPacket.payload, &registerPacket.header, RFREG_SIZE);
	txPacket.len = RFREG_SIZE;
	//OK. Now the packet is ready to be transmitted.
	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if (stat != EasyLink_Status_Success) {
		Halt_abort("Registration Packet EasyLink_transmit failed", &stat);
	}
	RFStatusFlags |= (RFS_REGSEND | RFS_TIMEOUTRUN);
	Clock_start(RFTimeoutClk);
}


static void SendNodePars(void) {
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint8_t i;								//Helper counter for "for" loops
	uint8_t* ptr;							//Pointer to manipulate the raw bytes of a packet
	EasyLink_Status stat;					//Status of EasyLink commands

	txPacket.dstAddr[0] = DEF_MASTER_ADDRESS;
	infoPacket.header.sourceAddress = NodeAddr;
	infoPacket.CALADC12_15V_30C = NodeParams.CALADC12_15V_30C;
	infoPacket.CALADC12_15V_85C = NodeParams.CALADC12_15V_85C;
	infoPacket.MasterCounter = NodeParams.MasterCounter;
	infoPacket.Battery = NodeParams.Battery;
	//Create the CRC of the packet
	ptr = (uint8_t*)&infoPacket.header;
	CRCCheck = DEF_CRC16;					//Initialize CRC Checksum
	for(i = 0; i < RFREG_SIZE; i++) {
		if(i == RFHEAD_SIZE) {				//If i points to CRC value => ...
			i += RFCRC_SIZE;				// ... then skip it. It is not included in final
											// CRC Checksum
		}
		CRCCheck = CalcCRC16(ptr[i], CRCCheck);
	}
	infoPacket.CRC = CRCCheck;			//Store the calculated CRC

	/*The packet to be transmitted is ready. Now it is needed to be in a EasyLink TxPacket
	payload in order to be transmitted.*/
	memcpy(&txPacket.payload, &infoPacket.header, RFGETINFO_SIZE);
	txPacket.len = RFGETINFO_SIZE;
	//OK. Now the packet is ready to be transmitted.
	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if (stat != EasyLink_Status_Success) {
		Halt_abort("Get Information Packet EasyLink_transmit failed", &stat);
	}
	RFStatusFlags |= (RFS_INFOSEND | RFS_TIMEOUTRUN);
	Clock_start(RFTimeoutClk);
}


static SEStatus SendEvents(SEFlag flag) {
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint8_t i, iMax;						//Helper variables for loops
	uint8_t EvIdx; 							//Pointer to current event in queue
	EasyLink_Status stat;					//Status of EasyLink commands
	RTCStruct CurrTime;						//Helper structure for timestamp converting

	//If we need a retransmission of the last packet then the packet is ready
	if(flag == SENDEVENTS_NEW) {
		//Set the destination address of the packet.
		txPacket.dstAddr[0] = DEF_MASTER_ADDRESS;
		/*The packet that will be sent is the latestTxPacket. Some fields were initialized
		before. The rest will be filled in here.*/
		latestTxPacket.rfData.CALADC12_15V_30C = NodeParams.CALADC12_15V_30C;
		latestTxPacket.rfData.CALADC12_15V_85C = NodeParams.CALADC12_15V_85C;
		latestTxPacket.rfData.Battery = NodeParams.Battery;
		latestTxPacket.rfData.MasterCounter = NodeParams.MasterCounter;
		latestTxPacket.rfData.packetNo++;
		/*Events needed to be transmitted are inserted in the packet. The length of the packet
		is specified by EasyLink. So the maximum number of events in the packet is calculated
		by the macro MAXEVENTS. Lets fill in the packet with as many events as it can hold.*/
		iMax = EventsLen;
		if(iMax > MAXEVENTS) {				//More event than the space provided?
			iMax = MAXEVENTS;				//Yes => Truncate the number to maximum
		}
		latestTxPacket.rfData.EventsLen = iMax;	//Number of events stored in this packet
		EvIdx = EventsStrt;					//Index used in the EventsArray (event queue)
		for(i = 0; i < iMax; i++) {
			//Prepare the timestamp of the event to be inserted in the packet
			CurrTime.Year = EventsArray[EvIdx].Year;
			CurrTime.Month = EventsArray[EvIdx].TypeMonth & 0x0F;
			CurrTime.Day = EventsArray[EvIdx].Day;
			CurrTime.Hour = EventsArray[EvIdx].Hour;
			CurrTime.Minute = EventsArray[EvIdx].Minute;
			CurrTime.Second = EventsArray[EvIdx].Second;
			//Fill the event in the packet with the necessary data.
			latestTxPacket.rfData.Events[i].TimeStamp = RTCStructToTstamp(&CurrTime);
			latestTxPacket.rfData.Events[i].EventType = EventsArray[EvIdx].TypeMonth >> 4;
			latestTxPacket.rfData.Events[i].Temperature = EventsArray[EvIdx].Temperature;
			latestTxPacket.rfData.Events[i].EventFlags = 0;
			//Proceed to next event in the events queue. Remember that it is a cyclic buffer
			EvIdx++;
			if(EvIdx >= EVENTSARRAY_SIZE) {	//Passed the end of the events queue?
				EvIdx = 0;					//Yes => Revert to its beginning
			}
		}
		/*Last but not least, we must calculate the CRC of this packet. We iterate through the
		raw bytes of the packet.*/
		CRCCheck = DEF_CRC16;				//Initialize CRC Checksum
		iMax = RFDATA_SIZE(iMax);			//Get the number of bytes in the whole packet
		for(i = 0; i < iMax; i++) {
			if(i == RFHEAD_SIZE) {			//If i points to CRC value => ...
				i += RFCRC_SIZE;			// ... then skip it. It is not included in final
											// CRC Checksum
			}
			CRCCheck = CalcCRC16(latestTxPacket.raw[i +RFPREAMP_SIZE], CRCCheck);
		}
		latestTxPacket.rfData.CRC = CRCCheck;
		/*In order for the packet to be transmitted, EasyLink wants it to be the payload of an
		EasyLink_TxPacket. So copy the full rfData packet in the payload of the txPacket.*/
		memcpy(txPacket.payload, &latestTxPacket.rfData.header, iMax);
		txPacket.len = iMax;
		RetriesCnt = RF_MAX_RETRIES +1;
	}
	//OK. Now the packet is ready to be transmitted, but first stop any async action
	RetriesCnt--;							//This is the first transmission try
	if(RetriesCnt > 0) {
		EasyLink_abort();
		stat = EasyLink_transmit(&txPacket);
		if (stat != EasyLink_Status_Success) {
			Halt_abort("Data Packet EasyLink_transmit failed", &stat);
		}
		RFStatusFlags |= (RFS_DATASEND | RFS_TIMEOUTRUN);
		Clock_start(RFTimeoutClk);
		return SENDEVENTS_OK;
	}
	return SENDEVENTS_EXHAUSTED;
}


static void SendAckBack(uint8_t packetSession, uint8_t packetNo) {
	struct RawPAckPacket* ptr;				//Pointer to manipulate EasyLink TX Packet payload
	uint16_t CRCCheck;						//The calculated CRC values
	uint8_t i;								//Helper for CRC bytes counting
	EasyLink_Status stat;					//Status of EasyLink commands

	txPacket.dstAddr[0] = DEF_MASTER_ADDRESS;
	txPacket.len = RFACK_SIZE;				//The packet will be used as an Ack packet
	ptr = (struct RawPAckPacket*)&txPacket.payload;
	ptr->header.packetType = RFPacket_Ack;
	ptr->header.sourceAddress = NodeAddr;
	ptr->packetNo = packetNo;
	ptr->packetSession = packetSession;
	ptr->flags = 0;
	memcpy(ptr->NodeMAC, registerPacket.NodeMAC, 8);
	CRCCheck = DEF_CRC16;					//Initialize CRC Checksum
	for(i = 0; i < RFACK_SIZE; i++) {
		if(i == RFHEAD_SIZE) {				//If i points to CRC value => ...
			i += RFCRC_SIZE;				// ... then skip it. It is not included in final
											// CRC Checksum
		}
		CRCCheck = CalcCRC16(txPacket.payload[i], CRCCheck);
	}
	ptr->CRC = CRCCheck;
	EasyLink_abort();
	stat = EasyLink_transmit(&txPacket);
	if (stat != EasyLink_Status_Success) {
		Halt_abort("Data Acknowledge EasyLink_transmit back to master failed", &stat);
	}
}


static void rxDoneCallback(EasyLink_RxPacket * rxPacket, EasyLink_Status status)
{
	UInt OutEvent;							//The event to be posted to RF task event loop
	struct PacketHeader* tmpRxHeader;		//Helper pointer to parse the header of a packet
	struct RawRegisterAckPacket* tmpRxRegAck;//Helper pointer to parse the registration answer
											// from a master node
	struct BeaconPacket* tmpBeacon;			//Helper pointer to parse the received beacon
											// packet from a master to decide if it is for us
	struct RawPAckPacket* tmpRxAck;			//Helper pointer to parse an Ack/PAck/NAck packet
	struct RawInfoPacket* tmpRxInfo;		//Helper pointer to access the received packet as
											// a NodeInfoPacket
	uint16_t CRCCheck;						//Helper to calculate the CRC16 of the packet
	uint16_t i, iMax;						//Helper variables for loops

	/*In case of EasyLink_abort() there is also a callback firing with status
	EasyLink_Status_Aborted. In that case we must not send any event, as we didn't receive any
	packet and the main code knows that the EasyLink subsystem is aborted and does not accept
	any packets until it enters a Receive mode.*/
	if(status == EasyLink_Status_Aborted) {
		return;
	}

	/*When there is no abort we must check the received packet. Consider it as invalid until
	proved otherwise.*/
	OutEvent = RFEVENT_INVALID_PACKET_RECEIVED;
	tmpRxHeader = (struct PacketHeader*)rxPacket->payload;
	//If we received a packet successfully we have to parse it
	if (status == EasyLink_Status_Success) {
		//Lets check if this is a known and acceptable packet
		/*The valid packets for the node are RFPacket_Beacon, RFPacket_Ack, RFPacket_NAck,
		RFPacket_PAck, RFPacket_RegisterAck, RFPacket_RegisterNAck and RFPacket_NodeInfo.
		Except RFPacket_Beacon, the rest of the packets are expected according to the packet
		the node sent earlier.*/
		switch(tmpRxHeader->packetType) {
		/*First lets check for the Beacon packet. If it is, copy the packet payload into
		latestBeacon.*/
		case RFPacket_Beacon:
			/*If we have a short address then we belong to a network. We must accept beacons
			only from this master. The only reason to accept ANY beacon is to try to resister
			to a new network or if we do not belong to any network, yet.*/
			tmpBeacon = (struct BeaconPacket*)(rxPacket->payload -RFPREAMP_SIZE);
			i = 0;
			if(NodeAddr != DEF_BROADCAST_ADDRESS) {
				for(i = 0; i < 8; i++) {
					if(tmpBeacon->SourceMAC[i] != MasterNodeEntry->MAC.raw[i]) {
						break;
					}
				}
				if(((RFStatusFlags & RFS_REGISTER) == 0) && (i != 8)) {
					break;
				}
			}
			if(i == 8) {
				RFStatusFlags |= RFS_OURBEACON;
			} else {
				RFStatusFlags &= ~RFS_OURBEACON;
			}
			OutEvent &= ~RFEVENT_INVALID_PACKET_RECEIVED;
			OutEvent |= RFEVENT_BEACON;
			latestBeacon.preamp.rssi = (int8_t)rxPacket->rssi;
			latestBeacon.preamp.len = rxPacket->len;
			memcpy(&latestBeacon.header, rxPacket->payload, rxPacket->len);
			if(i == 8) {
				if(BLastTime == 0) {
					BLastTime = latestBeacon.CurrTimeStamp;
				} else if(BLastTime != latestBeacon.CurrTimeStamp) {
					OutEvent |= RFEVENT_RTC;
				}
			}
			break;

		case RFPacket_RegisterAck:
		case RFPacket_RegisterNAck:
			if((RFStatusFlags & (RFS_REGISTER | RFS_REGSEND)) ==
				(RFS_REGISTER | RFS_REGSEND)) {
				tmpRxRegAck = (struct RawRegisterAckPacket*)rxPacket->payload;
				//Lets check if this packet really belongs to us
				for( i = 0; i < 8; i++) {
					if(tmpRxRegAck->NodeMAC[i] != registerPacket.NodeMAC[i]) {
						break;
					}
				}
				//At this point if i == 8 then we have a match in MAC addresses
				if(i != 8) {
					break;
				}
				//Lets also check the CRC validity of the packet
				iMax = RFREGACK_SIZE;
				if(tmpRxRegAck->header.packetType == RFPacket_RegisterNAck) {
					iMax = RFREGNACK_SIZE;
				}
				CRCCheck = DEF_CRC16;		//Initialize CRC Checksum
				for(i = 0; i < iMax; i++) {
					if(i == RFHEAD_SIZE) {	//If i points to CRC value => ...
						i += RFCRC_SIZE;	// ... then skip it. It is not included in final
											// CRC Checksum
					}
					CRCCheck = CalcCRC16(rxPacket->payload[i], CRCCheck);
				}
				//If the CRC values do not match, ignore this packet
				if(tmpRxRegAck->CRC != CRCCheck) {
					break;
				}
				RFStatusFlags |= RFS_COPYPACKET;
				OutEvent &= ~RFEVENT_INVALID_PACKET_RECEIVED;
				OutEvent |= RFEVENT_REGOK;
			}
			break;

		case RFPacket_Ack:
		case RFPacket_PAck:
		case RFPacket_NAck:
			if((RFStatusFlags & RFS_DATASEND) == RFS_DATASEND) {
				tmpRxAck = (struct RawPAckPacket*)rxPacket->payload;
				//In order for this packet to belong to us, the NodeMAC must be our master's
				for( i = 0; i < 8; i++) {
					if(tmpRxAck->NodeMAC[i] != MasterNodeEntry->MAC.raw[i]) {
						break;
					}
				}
				//At this point if i == 8 then we have a match in MAC addresses
				if(i != 8) {
					break;
				}
				//Lets also check the CRC validity of the packet
				iMax = RFACK_SIZE;
				if(tmpRxAck->header.packetType == RFPacket_NAck) {
					iMax = RFNACK_SIZE;
				} else if(tmpRxAck->header.packetType == RFPacket_PAck) {
					iMax = RFPACK_SIZE;
				}
				CRCCheck = DEF_CRC16;		//Initialize CRC Checksum
				for(i = 0; i < iMax; i++) {
					if(i == RFHEAD_SIZE) {	//If i points to CRC value => ...
						i += RFCRC_SIZE;	// ... then skip it. It is not included in final
											// CRC Checksum
					}
					CRCCheck = CalcCRC16(rxPacket->payload[i], CRCCheck);
				}
				//If the CRC values do not match, ignore this packet
				if(tmpRxAck->CRC != CRCCheck) {
					break;
				}
				RFStatusFlags |= RFS_COPYPACKET;
				OutEvent &= ~RFEVENT_INVALID_PACKET_RECEIVED;
				OutEvent |= RFEVENT_ACK;
			}
			break;

		case RFPacket_NodeInfo:
			if((RFStatusFlags & RFS_INFOSEND) != 0) {
				tmpRxInfo = (struct RawInfoPacket*)rxPacket->payload;
				/*In order for this packet to belong to us, the MasterMAC must be our master's
				and the NodeMAC must be ours.*/
				for( i = 0; i < 8; i++) {
					if((tmpRxInfo->MasterMAC[i] != MasterNodeEntry->MAC.raw[i]) ||
						(tmpRxInfo->NodeMAC[i] != infoPacket.NodeMAC[i])) {
						break;
					}
				}
				//At this point if i == 8 then we have a match in both MAC addresses
				if(i != 8) {
					break;
				}
				//Lets check the CRC value of the packet to see if we can accept the packet
				CRCCheck = DEF_CRC16;		//Initialize CRC Checksum
				for(i = 0; i < RFNODEINFO_SIZE; i++) {
					if(i == RFHEAD_SIZE) {	//If i points to CRC value => ...
						i += RFCRC_SIZE;	// ... then skip it. It is not included in final
											// CRC Checksum
					}
					CRCCheck = CalcCRC16(rxPacket->payload[i], CRCCheck);
				}
				//If the CRC values do not match, ignore this packet
				if(tmpRxInfo->CRC != CRCCheck) {
					break;
				}
				RFStatusFlags |= RFS_COPYPACKET;
				OutEvent &= ~RFEVENT_INVALID_PACKET_RECEIVED;
				OutEvent |= RFEVENT_INFO;
			}
			break;
		}

		/*Every packet preprocessing, from the previous "switch" command, flags if the already
		received packet needs to be copied into latestRxPacket in case it needs further
		processing from the reception event loop of the task. In that case we have to do the
		copying.*/
		if((RFStatusFlags & RFS_COPYPACKET) != 0) {
			RFStatusFlags &= ~RFS_COPYPACKET;
			/*Save the latest RSSI and copy the input packet in the latest one to be
			processed.*/
			latestRxPacket.init.preamp.rssi = (int8_t)rxPacket->rssi;
			latestRxPacket.init.preamp.len = rxPacket->len;
			/*First we have to point to the payload of the incoming EasyLink packet, in
			order to parse the data as the correct communication packet. Remember that the
			communication packet is embedded in an EasyLink one as the payload. Also, the
			first part of a packet in memory contains a small preamp, which must be
			ignored as the received packet contains only the fields from header part to
			the end of packet.*/
			memcpy(&latestRxPacket.raw[RFPREAMP_SIZE], rxPacket->payload, rxPacket->len);
		}
    }
    Event_post(RFEventHandle, OutEvent);
}


/*********************************************************************************************
 * Main Task Function                                                                        *
 ********************************************************************************************/
static void RFTaskFxn(UArg arg0, UArg arg1) {
	EasyLink_Status stat;					//Return status of EasyLink commands
	UInt key;								//Interrupt status key
	int32_t i;								//Loop helper variable
	uint32_t ActivityTime, tmpTime;			//Helper variable to help total activity timer
	uint32_t MasterEv, InnerEv;				//The two events, Master for the On/Off event and
											// Inner for the inner loop event (packet events)
	MACEntry NewMaster;						//New Master node entry in case of registration

	PIN_setInterrupt(swPinHandle, RFBUTTON | PIN_IRQ_BOTHEDGES);
	registerButtonCb(RFBUTTON, TriggerRF);
	while (1) {
		BLastTime = 0;						//Reset the last beacon's current time
		latestTxPacket.rfData.packetSession++;//Increase the number of session
		latestTxPacket.rfData.packetNo = 0;	//Reset the number of packets in this session
		/*Here we use EasyLink framework to create our communication protocol. The problem is
		that after some time the RF module is not in use, it falls asleep to preserve power
		and EasyLink cannot wake it up. In that case the process of RF seems to wait forever.
		To override this problem we have to initialize EasyLink framework every time there is
		a new RF cycle in order to avoid lockups!*/
		MasterEv = Event_pend(RFEventHandle, Event_Id_NONE, RFEVENT_START | RFEVENT_SYNC,
			BIOS_WAIT_FOREVER);
		RFStatusFlags |= RFS_ONAIR;			//Mark that RF module is now in use
		if((MasterEv & RFEVENT_SYNC) != 0) {
			RFStatusFlags |= RFS_SYNC;
		}

		//Initialize EasyLink for the specified modulation
		stat = EasyLink_init(DEF_MODULATION);
		if(stat != EasyLink_Status_Success) {
			Halt_abort("EasyLink_init failed", &stat);
		}

		//Set EasyLink frequency to 868MHz
		stat = EasyLink_setFrequency(DEF_FREQUENCY);
		if(stat != EasyLink_Status_Success) {
			Halt_abort("EasyLink_setFrequency failed", &stat);
		}

		/*Filter only the node's address. EasyLink will listen only to this address. All other
		packets will be ignored. If the node address is not set yet, then only the broadcast
		address is accepted.*/
		EasyLink_enableRxAddrFilter(ValidAddrs, 1, (NodeAddr == 0) ? 1 : 2);

		/*Lets start the RF Activity LED flashing and the timeout counter for the total
		communication.*/
		RFFlashFxn(1);						//Start flashing the RF LED
		ActivityTime = 0;					//Reset Activity starting time
		Clock_setTimeout(RFActivityClk, ((RFStatusFlags & RFS_SYNC) != 0) ?
			RFSYNC_TIMEOUT : (RFSYNC_TIMEOUT + RFACTIVITY_TIMEOUT));
		Clock_start(RFActivityClk);			//Start the activity timeout timer with normal
											// or sync activity time limit.

		//Simulate the event from InnerEv
		InnerEv = MasterEv;

		//On Air reception loop
		do {
			//Do we have to extend the time after SYNC?
			if((InnerEv & RFEVENT_EXTENDTIME) != 0) {
				/*SYNC period expired. Now need the normal activity timeout that is the
				maximum time the RF Rx will remain active. If the communications end before
				activity timeout expires, the system will force sleep.*/
				Clock_setTimeout(RFActivityClk, RFACTIVITY_TIMEOUT);
				Clock_start(RFActivityClk);
				RFStatusFlags &= ~RFS_SYNC;	//No more SYNC mode
			}

			//Lets check the possible events one by one:
			if((InnerEv & RFEVENT_REGISTER) != 0) {
				//User wants to register to a new network
				RFStatusFlags |= RFS_REGISTER;
				/*The system will stay active for "registration timeout", until there is a
				valid registration. This time runs together with the activity timeout. Thus,
				we need to check how much time remains until the activity timeout expires in
				order to restore the correct value after registration.*/
				ActivityTime = Clock_getTimeout(RFActivityClk);
				/*If the system is in SYNC mode (user pressed the SYNC button) the timer was
				counting for SYNC timeout. This means that the real timeout till SLEEP event
				is extended by the RF Activity interval. Since there is a registration event
				we must exit SYNC mode. This is why we have to take this timeout extension
				into account.*/
				if((RFStatusFlags & RFS_SYNC) != 0) {
					//In Sync mode? => Extend the timeout value and exit Scan mode
					ActivityTime += RFACTIVITY_TIMEOUT;
					RFStatusFlags &= ~RFS_SYNC;
				}
				//Reschedule the activity clock
				Clock_stop(RFActivityClk);
				Clock_setTimeout(RFActivityClk, REGISTER_TIMEOUT);
				Clock_start(RFActivityClk);
			}

			//Lets see if we need to update the RTC from the latest received beacon
			if((InnerEv & RFEVENT_RTC) != 0) {
				/*The EasyLink callback finds out if a beacon is ours or not. It triggers an
				RTC event only if the beacon belongs to our master. That means we must already
				be registered to a network to have our own master node. So there is no need to
				check those parameters; they are already checked!*/
				RTCSetCurrTime(latestBeacon.CurrTimeStamp);
				RFStatusFlags |= RFS_RTCSET;
			}

			//Lets follow the beacons received. They may be from different masters.
			if((InnerEv & RFEVENT_BEACON) != 0) {
				if((RFStatusFlags & (RFS_REGISTER | RFS_TIMEOUTRUN | RFS_OURBEACON)) ==
/*Normally a node will not try to re-register at its master. If this is not the needed action,
comment out the previous line of this comment and uncomment the following one. In that case a
node will not check if the master node is the one this slave node is registered to and will
retry registering sequence.*/
//				if((RFStatusFlags & (RFS_REGISTER | RFS_TIMEOUTRUN)) ==
					//Lets see if we need to register to a master
					RFS_REGISTER) {
					SendReg();
				} else if((RFStatusFlags & (RFS_REGISTER | RFS_TIMEOUTRUN | RFS_OURBEACON)) ==
					RFS_OURBEACON) {
					RFStatusFlags |= RFS_BEACONOK;
					/*If not in valid registration mode then lets see if we have to send any
					event timestamps.*/
					if((EventsLen > 0) && ((RFStatusFlags & RFS_ENDDATA) == 0)) {
						/*If there was a packet sent without confirmation, then we need to
						retry sending the same packet. Otherwise we send a new one. If it is a
						retry, then we have to see if there are no more tries and in that case
						we stop sending data packets.*/
						RFStatusFlags |= RFS_PARAMSREQ;
						i = SendEvents(((RFStatusFlags & RFS_DATASEND) != 0) ?
							SENDEVENTS_RETRY : SENDEVENTS_NEW);
						if(i == SENDEVENTS_EXHAUSTED) {
							RFStatusFlags &= ~RFS_DATASEND;
							RFStatusFlags |= RFS_ENDDATA;
						}
					} else if(EventsLen == 0) {
						if((RFStatusFlags & (RFS_PARAMSREQ | RFS_PARAMSOK)) != 0) {
							RFStatusFlags &= ~RFS_DATASEND;
							RFStatusFlags |= RFS_ENDDATA;
						} else {
							RFStatusFlags |= RFS_PARAMSREQ;
							SendNodePars();
						}
					}
				}
			}

			/*Next event to process is the REGOK event that shows a registration acceptance
			from a master node. The reception of such a packet must be during the timeout
			counting from the packet sending.*/
			if(((InnerEv & RFEVENT_REGOK) != 0) &&
				((RFStatusFlags & (RFS_TIMEOUTRUN | RFS_REGSEND)) ==
					(RFS_TIMEOUTRUN | RFS_REGSEND))) {
				Clock_stop(RFTimeoutClk);	//Stop counting the packet answer timeout
				/*Clear the associated flags. If the packet is a Registration NAck then the
				master has rejected us. Ignore it and assume we are not registered. If the
				packet is a Registration Ack then we register ourselves into the new network.
				*/
				RFStatusFlags &= ~(RFS_REGISTER | RFS_TIMEOUTRUN | RFS_REGSEND);
				if(latestRxPacket.regAck.header.packetType == RFPacket_RegisterAck) {
					//Positive registration answer? => Need to prepare Flash memory
					NewMaster.flg8 = 0xFF;	//Remember that flags 8 & 16 are inverted (Flash)
					NewMaster.flg16 = 0xFFFF;
					NewMaster.ShortAddr = latestRxPacket.regAck.NodeShortAddr;
					memcpy(NewMaster.MAC.raw, registerPacket.MasterMAC, 8);
					//Perform the writing of the new master to Flash memory
					if(SetMasterNode(&NewMaster) == FAPI_STATUS_SUCCESS) {
						/*OK, new master is added in Flash memory. Now prepare the new
						parameters (new short address and EasyLink filtering.*/
						ValidAddrs[1] = NodeAddr;
						EasyLink_enableRxAddrFilter(ValidAddrs, 1, 2);
						//Resume the normal RF Activity clock if its time is not expired
						tmpTime = REGISTER_TIMEOUT -Clock_getTimeout(RFActivityClk);
						if(tmpTime < ActivityTime) {
							//Time not expired => Resume
							ActivityTime -= tmpTime;
							Clock_stop(RFActivityClk);
							Clock_setTimeout(RFActivityClk, ActivityTime);
							Clock_start(RFActivityClk);
						} else {
							/*Time expired, so do not resume the activity clock. Just simulate
							its expiration.*/
							Clock_stop(RFActivityClk);
							RFActivityFxn(0);//Act as if it was really expired
						}
					}
				}
			}

			/*Another event is the ACK event that shows the presence of an Ack, PAck, or NAck
			packet. In case of an Ack, we discard all the events previously sent and proceed
			to the next data packet, if there are more event stamps in the events queue. In
			case of a PAck, we remove the first packets that were accepted by the master and
			stop sending any more events, as the server announces that it partially accepted
			the events sent, due to memory constraints. In case of a NAck there is the
			possibility of a NAck due to CRC error. So, we retransmit the last packet for a
			maximum number of times. If this time is after the last, then there is another
			reason the master cannot accept the packet so we stop retrying and cancel sending
			further packets this time.
			One more thing is that when the master sends flags in the Ack/PAck packet, then it
			expects our Ack answer. In that case we have to prepare an Ack and transmit it to
			the master.
			EasyLink callback function checked just three things in this packet: The first is
			the type of the packet (that defined the ACK event), the second is the MAC address
			of the master is the one that we belong to its network and third is the CRC of the
			packet that needs to be correct.*/
			if(((InnerEv & RFEVENT_ACK) != 0) &&
				((RFStatusFlags & (RFS_TIMEOUTRUN | RFS_DATASEND)) ==
					(RFS_TIMEOUTRUN | RFS_DATASEND))) {
				//Is this a NAck packet and a reply to the last packet we have sent earlier?
				if((latestRxPacket.pack.packetSession ==
						latestTxPacket.rfData.packetSession) &&
					(latestRxPacket.pack.packetNo == latestTxPacket.rfData.packetNo)) {
					//The latestRxPacket contains the received answer from the master
					Clock_stop(RFTimeoutClk);
					RFStatusFlags &= ~(RFS_TIMEOUTRUN | RFS_DATASEND | RFS_RESET |
						RFS_PARAMSREQ);
					if(latestRxPacket.pack.header.packetType == RFPacket_Ack) {
						//Keep on sending more data until all events are gone
						/*First remove the events that were sent to the master from the queue.
						In order to do that, no-one should disturb the event queue during the
						start and length calculations. So we must first disable the interrupts
						and do the job. After that the interrupts must be restored.*/
						i = latestTxPacket.rfData.EventsLen;
						//Get the flags for further manipulation
						if((latestRxPacket.pack.flags & PF_RESET) != 0) {
							RFStatusFlags |= RFS_RESET;
						}
						RFStatusFlags |= RFS_PARAMSOK;
					} else if(latestRxPacket.pack.header.packetType == RFPacket_PAck) {
						//This was the last data packet sent for this session
						/*Must remove the events accepted by the master, first.*/
						i = latestRxPacket.pack.eventsCount;
						//Get the flags for further manipulation
						if((latestRxPacket.pack.flags & PF_RESET) != 0) {
							RFStatusFlags |= RFS_RESET;
						}
						//Now the queue does not contain the events sent. Flag the end of data
						RFStatusFlags |= (RFS_ENDDATA | RFS_PARAMSOK);
					} else if(latestRxPacket.pack.header.packetType == RFPacket_NAck) {
						//Yes => Just retry sending the last packet
						i = 0;
						if(SendEvents(SENDEVENTS_RETRY) != SENDEVENTS_OK) {
							//Out of tries...
							RFStatusFlags |= RFS_ENDDATA;
						}
					} else {
						i = 0;
					}
					if(i > 0) {				//Are there events to be removed from queue?
						/*Yes => Remove them. Keep in mind that this is a critical block.
						Nothing should disturb the events queue during the removal of events
						from it. So, disable the interrupts during calculations and restore
						them afterwards.*/
						key = Hwi_disable();//Start critical block
						EventsLen -= i;		//i events less now
						EventsStrt += i;	//Move the starting index i events next
						//Did the starting index pass the end of the queue? If yes, revert
						if(EventsStrt >= MAXEVENTS) {
							EventsStrt -= MAXEVENTS;
						}
						Hwi_restore(key);	//Critical block ends here.
						if(EventsLen == 0) {//No more events to send? Stop sending
							RFStatusFlags |= RFS_ENDDATA;
						}
					}
					//Check flags of the received packet
					if((RFStatusFlags & RFS_RESET) != 0) {
						TriggerReset();
						RFStatusFlags &= ~RFS_RESET;
						SendAckBack(latestRxPacket.pack.packetSession,
							latestRxPacket.pack.packetNo);
					}
					if((RFStatusFlags & RFS_ENDDATA) == 0) {
						//Now the queue does not contain the events sent. Send the rest
						SendEvents(SENDEVENTS_NEW);
					}
					TriggerAck(UARTEVENT_ACK);
				}
			}

			/*Another possible answer from the master node is the Node Info packet. This is
			sent to the node only if there was a Get Info request. When there is the answer in
			a Node Info packet the EasyLink callback function sends a RFEVENT_INFO event to
			notify us for the packet. When this packet is received and it contains any flag
			set, there must be an Ack answer to notify the master that the request was
			received correctly.*/
			if(((InnerEv & RFEVENT_INFO) != 0) &&
				((RFStatusFlags & (RFS_INFOSEND | RFS_TIMEOUTRUN)) ==
					(RFS_INFOSEND | RFS_TIMEOUTRUN))) {
				Clock_stop(RFTimeoutClk);
				TriggerAck(UARTEVENT_INFO);
				RFStatusFlags &= ~(RFS_TIMEOUTRUN | RFS_INFOSEND | RFS_PARAMSREQ);
				RFStatusFlags |= RFS_PARAMSOK;
				RFRestart = latestRxPacket.nodeInfo.RadioTm *60;
				if((latestRxPacket.nodeInfo.flags & PF_RESET) != 0) {
					TriggerReset();
					SendAckBack(0, 0);
				}
			}

			/*At this point, if there were all the possible communications done (RTC Sync,
			registration activity, data of new events sent to master) then there is no need
			for the system to stay awake.*/
			if((RFStatusFlags &
				(RFS_SYNC | RFS_REGISTER | RFS_TIMEOUTRUN | RFS_RTCSET | RFS_ENDDATA)) !=
				(RFS_RTCSET | RFS_ENDDATA)) {
				//Enter Asynchronous Receive mode again to keep receiving packets
				stat = EasyLink_receiveAsync(rxDoneCallback, 0);
				if(stat == EasyLink_Status_Busy_Error) {
					EasyLink_abort();
					stat = EasyLink_receiveAsync(rxDoneCallback, 0);
				}
				if(stat != EasyLink_Status_Success) {
					Halt_abort("EasyLink_receiveAsync re-entering failed", &stat);
				}

				InnerEv = Event_pend(RFEventHandle, Event_Id_NONE, RFEVENT_ALL,
					BIOS_WAIT_FOREVER);
			} else {
				Clock_stop(RFActivityClk);
				InnerEv = RFEVENT_SLEEP;
			}
		} while ((InnerEv & RFEVENT_SLEEP) == 0);
		EasyLink_abort();					//Quit possible running receive async command
		RFFlashFxn(2);						//Stop LED flashing
		RFStatusFlags &= ~RFS_ONAIR;		//RF module is not in use anymore
		if((RFStatusFlags & RFS_RTCSET) != 0) {
			TriggerRTC();
			RFStatusFlags &= ~RFS_RTCSET;
			RFStatusFlags |= RFS_RTCOK;
		}
		//Need to set the Activity timer for the time remain till the next Master wake-up.
		/*The restarting time of the RF activity in normal operation is set to 10 minutes as a
		default value. This value is actually set in the master node using an SMS. There are
		three situations for RF synchronization. The first is when the node is not registered
		in any network so it has the broadcast address as its short address. In that case it
		scans for a network, so it wakes up the radio every 10 minutes incremented by half of
		the time its RF stays on. Remember that the rescheduling is done at the end of the RF
		activity, so in order to achieve the time interval needed we have to count it in; the
		formula used is DEF_RFRESTART -(RFACTIVITY_TIMEOUT +RFSYNC_TIMEOUT) +
			((RFACTIVITY_TIMEOUT +RFSYNC_TIMEOUT) /2) =
			DEF_RFRESTART -((RFACTIVITY_TIMEOUT +RFSYNC_TIMEOUT) /2).
		If the node is registered to a network then it has a valid short address. In that case
		if it received a beacon, it knows exactly the time the master will restart its RF
		communication so it synchronizes itself with the master.
		In the case where there was a communication error, noise or something else that made
		the reception of a valid beacon impossible, it estimates the time the master will (or
		should) restart its RF activity.*/
		if(NodeAddr == DEF_BROADCAST_ADDRESS) {
			ActivityTime = DEF_RFRESTART -((RFACTIVITY_TIMEOUT +RFSYNC_TIMEOUT) /2);
		} else if((RFStatusFlags & RFS_BEACONOK) != 0) {
			ActivityTime = latestBeacon.NextWakeUpTime -RTCGetCurrTime();
		} else {
			ActivityTime = RFRestart -(RFACTIVITY_TIMEOUT +RFSYNC_TIMEOUT);
		}
		Clock_setTimeout(RFActivityClk, ActivityTime *1000000 / Clock_tickPeriod);
		Clock_start(RFActivityClk);
		RFStatusFlags &= RFS_RTCOK;
	}
}

