/*********************************************************************************************
 * UART2MSP.h                                                                                *
 *===========================================================================================*
 *  Created on: Oct 12, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 ********************************************************************************************/

#ifndef UART2MSP_H_
#define UART2MSP_H_

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
#include <ti/sysbios/knl/Event.h>


/*********************************************************************************************
 * Definitions                                                                               *
 ********************************************************************************************/
#define UARTEVENT_START	Event_Id_00
#define UARTEVENT_RTC	Event_Id_01
#define UARTEVENT_RESET	Event_Id_02
#define UARTEVENT_DATA	Event_Id_03
#define UARTEVENT_ACK	Event_Id_04
#define UARTEVENT_INFO	Event_Id_05
#define UARTEVENT_ALL	(UARTEVENT_START | UARTEVENT_RTC | UARTEVENT_RESET | UARTEVENT_DATA |\
							UARTEVENT_ACK | UARTEVENT_INFO)


/*********************************************************************************************
 * Function declarations                                                                     *
 ********************************************************************************************/
void InitUARTTask(void);					//Initializes the UART task
void TriggerReset(void);					//Triggers a Reset event
void TriggerRTC(void);						//Triggers an RTC event
void TriggerData(void);						//Triggers an Data event from key (not a real one)
void TriggerAck(uint32_t InVal);			//Triggers a Data Acknowledge Received event


#endif /* UART2MSP_H_ */
