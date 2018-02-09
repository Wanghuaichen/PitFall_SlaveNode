/*********************************************************************************************
 * AppFlash.h                                                                                *
 *===========================================================================================*
 *  Created on: Aug 28, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 * In this file only some definitions appear needed to be accessible by other parts of the   *
 * application, for flash memory handling.                                                   *
 *                                                                                           *
 * Definitions:                                                                              *
 *--------------                                                                             *
 * BASE_FLASH defines the starting offset of the node configuration area. In this area some  *
 *   configuration data needed for the connection of the node to the master network is       *
 *   stored.                                                                                 *
 * FLASHSEG_SIZE defines the size of a memory segment. It could be hardcoded but for more    *
 *   flexibility on porting the same code to different microcontrollers it is defined here.  *
 * MAXMACS is the number of MAC records that can be stored in a Flash segment.               *
 * MACENTRY_SIZE contains the size of a MACEntry record.                                     *
 * MNE8_DISABLES (see later at the MACEntry type definition explanation).                    *
 *                                                                                           *
 * Macro Definitions:                                                                        *
 *--------------------                                                                       *
 * MACADDR(x) is a macro that returns the address of MAC entry defined by x, in the Flash    *
 *   memory segment. The address is the absolute address and it is returned as a pointer to  *
 *   a MACEntry type.                                                                        *
 *                                                                                           *
 *                                                                                           *
 * Type Definitions:                                                                         *
 *-------------------                                                                        *
 * MACAddr is a union to let the code handle a node MAC address byte by byte or as two       *
 *   32 bit values. A node has a MAC address that is 64 bits long. The two entries of the    *
 *   union are:                                                                              *
 *   -raw[8]: is used for accessing the MAC address as an array of bytes.                    *
 *   -mac32[2]: is used for accessing the MAC address as an array of double words.           *
 * MACEntry: Is a full configuration record for the network joining. Its records are:        *
 *   -ShortAddr is the short address of the node within the network. It is assigned to the   *
 *     slave node by the master one.                                                         *
 *   -MAC is the MAC address of the master node of the network this node joined.             *
 *   -flg8: holds some flags to define the validity of this record. More specific:           *
 *      MNE8_DISABLED is the mask bit of the flag that disables the MACEntry record.         *
 *   -flg16: It is reserved to be used at later versions of the system. It is used for       *
 *      correct alignment of the entries and should always be FFFFh.                         *
 *                                                                                           *
 * Global Variables:                                                                         *
 *-------------------                                                                        *
 * MasterNodeEntry: is a pointer to the master node MACEntry this slave node has joined its  *
 *   network.                                                                                *
 * NodeAddr: is this node's short address. The short address is assigned by the master node  *
 *   at the phase of joining.                                                                *
 *                                                                                           *
 * Global Functions:                                                                         *
 *-------------------                                                                        *
 * InitFlash(): Is called upon initialization phase of the node (when powering up the node)  *
 *   to search the Flash configuration segment and find if the node has joined a network and *
 *   extract all the necessary parameters for communicating with the master node.            *
 * SetMasterNode(): Is used to set the master node parameters of the network and join it.    *
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

#ifndef APPFLASH_H_
#define APPFLASH_H_

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
/* TI-RTOS Driver files */
#include <driverlib/flash.h>

/* Standard C libraries */
#include <stdint.h>


/*********************************************************************************************
 * Definitions of new types                                                                  *
 ********************************************************************************************/
typedef union {
	uint8_t raw[8];							//Raw access of the MAC address bytes
	uint32_t mac32[2];						//Access of MAC address through two 32 bit words
} MACAddr;

typedef struct {
	uint8_t ShortAddr;						//The short address assigned by the master to this
											// node
	uint8_t flg8;							//Dummy byte. Can be used as flags
	uint16_t flg16;							//Dummy halfword; can also be used as flags
	MACAddr MAC;							//The MAC address of the master node
} MACEntry;
#define MACENTRY_SIZE	(sizeof(MACEntry))	//The size of a single MACEntry in memory


/*********************************************************************************************
 * Definitions                                                                               *
 ********************************************************************************************/
/*Some definitions for the flags of a MAC entry. Definitions that start with MNE8_ are for the
MACEntry.flg8 field and definitions that start with MNE16_ are for the MACEntry.flg16 field.
Due to the fact that in Flash memory you can write a bit with value 0, these fields are all
set if the corresponding bit is zeroed.*/
#define MNE8_DISABLED	(1 << 7)			//Bit7 if 0 shows if this master node is disabled


/*********************************************************************************************
 * Flash Memory Configuration Addresses                                                      *
 ********************************************************************************************/
#define BASE_FLASH		0x0001D000			//Offset of memory the settings stored in Flash
											// memory
#define FLASHSEG_SIZE	0x00001000			//Size of a memory segment
/*A segment of Flash memory can hold up to MAXMACS MAC entries, without erasing it. The goal
is to create a rolling table that can hold the MAC addresses of the master node that accepted
this particular slave. By using a rolling table we ensure longer Flash memory life and less
power consumption, since we do not perform an erase cycle each time a master node is
discovered.*/
#define MAXMACS			(FLASHSEG_SIZE / sizeof(MACEntry))
/*Lets define a macro that returns the address of a specific MAC Entry Index.*/
#define MACADDR(x)		((MACEntry*)(BASE_FLASH +(x*MACENTRY_SIZE)))


/*********************************************************************************************
 * Function declarations, functions other code needs to know about                           *
 ********************************************************************************************/
void InitFlash(void);						//Initializes the flash area with the defaults
uint8_t SetMasterNode(MACEntry *InMAC);		//Adds a new node in the list of known Master ones

/*********************************************************************************************
 * Variable declarations, variables other code needs to know about                           *
 ********************************************************************************************/
extern MACEntry* MasterNodeEntry;
extern uint8_t NodeAddr;

#endif /* APPFLASH_H_ */
