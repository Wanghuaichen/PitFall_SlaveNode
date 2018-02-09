/*********************************************************************************************
 * RadioTask.h                                                                               *
 *===========================================================================================*
 *  Created on: Oct 12, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 * In this file there are definitions and declarations needed to be global to other files of *
 * the application. Only brief information appear in this header section. More details can   *
 * found either in this file at each definition/declaration or at the respective .c file.    *
 *                                                                                           *
 * Definitions:                                                                              *
 *--------------                                                                             *
 * RFStatusFlags is a variable that describes the status of the RF Task. It contains flags   *
 * that are described through the following definitions. Each one is a mask of the bits used *
 * for the corresponding functionality.                                                      *
 * - RFS_TIMEOUTRUN flags that a time expecting an Ack packet as a response runs. The Ack    *
 *     response should be received before the timeout timer expires.                         *
 * - RFS_BEACONOK flags that the system received a beacon that must be parsed.               *
 * - RFS_OURBEACON flags that the beacon received belongs to our master.                     *
 * - RFS_INFOSEND flags that a GetInfo packet was sent to the master.                        *
 *                                                                                           *
 * Macro Definitions:                                                                        *
 *--------------------                                                                       *
 *                                                                                           *
 * Type Definitions:                                                                         *
 *-------------------                                                                        *
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

#ifndef TASKS_RFTASK_H_
#define TASKS_RFTASK_H_

#include "stdint.h"
#include <ti/sysbios/knl/Event.h>
#include "RF/RadioProtocol.h"

//#define PACKETQ_SIZE		8
#define RFEVENT_START		Event_Id_00

#define RFS_TIMEOUTRUN	(1 << 30)			//Timeout clock is in use
#define RFS_BEACONOK	(1 << 14)			//System received a beacon
#define RFS_INFOSEND	(1 << 13)			//A Get Info packet is sent to the master
#define RFS_PARAMSOK	(1 << 12)			//Shows that there was an valid answer for the
											// server's parameters requested from the master
#define RFS_PARAMSREQ	(1 << 11)			//Shows that there was a previous packet send that
											// also requested the node's parameters from
											// server
#define RFS_RTCOK		(1 << 10)			//Flags that the RTC is synced at least once
#define RFS_RTCSET		(1 << 9)			//Flags the synchronizing of RTC from a beacon
#define RFS_RESET		(1 << 8)			//Flags the necessity to send a Reset command to
											// MSP430 through UART
#define RFS_ENDDATA		(1 << 7)			//Flags that no more data packets are going to be
											// transmitted.
#define RFS_DATASEND	(1 << 6)			//There is a data packet on board. Expecting xAck
#define RFS_COPYPACKET	(1 << 5)			//Flags that the received packet is accepted and
											// should be copied into latestPacket for further
											// manipulation (for EasyLink callback)
#define RFS_OURBEACON	(1 << 4)			//The last beacon contains a beacon from our
											// master node and not from another network
#define RFS_REGSEND		(1 << 3)			//Expecting answer for a Registration request
#define RFS_REGISTER	(1 << 2)			//RF Registration request
#define RFS_SYNC		(1 << 1)			//RF sync RTC with the master
#define RFS_ONAIR		(1 << 0)			//RF communication is activated

enum RFStatus {
    RFStatus_Success,
    RFStatus_Failed,
    RFStatus_FailedNotConnected,
};

extern Event_Handle RFEventHandle;
extern volatile uint32_t RFStatusFlags;
//extern union RFPacket PacketQ[PACKETQ_SIZE];
//extern uint8_t PacketQFirst;
//extern uint8_t PacketQLen;

union RFPacket {
	uint8_t raw[EASYLINK_MAX_DATA_LENGTH];	//To handle a packet as raw bytes
    struct PacketInit init;					//Not a real packet. It is the common denominator
    										// of all described packets
	struct AckPacket ack;
	struct PAckPacket pack;
//	struct NAckPacket nack;					//NAck packet is a Ack packet with different type
	struct BeaconPacket beacon;
	struct RegisterNodePacket regNode;
	struct RegisterAckPacket regAck;
//	struct RegisterNAckPacket regNAck;		//RegisterNAck packet is similar to RegisterAck
	struct DataPacket rfData;
	struct GetInfoPacket getInfo;
	struct NodeInfoPacket nodeInfo;
};

typedef void (*RFPacketCb)(union RFPacket* packet, int8_t rssi);

/* Create the ConcentratorRadioTask and creates all TI-RTOS objects */
void InitRFTask(void);

/* Register the packet received callback */
//void RFTask_registerPacketCb(RFPacketCb callback);
void ParseRegPacket(union RFPacket* CurrPacket, uint16_t CRCCheck);
void ParseDataPacket(union RFPacket* CurrPacket);

#endif /* TASKS_RFTASK_H_ */
