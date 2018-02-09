/*********************************************************************************************
 * AppFlash.c                                                                                *
 *===========================================================================================*
 *  Created on: Aug 28, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 * In this file there are functions that handle the flash memory of the microcontroller. The *
 * flash memory is used for storing parameters necessary for the node to join the network.   *
 * The configuration area occupies a whole segment and is used as a rolling space when the   *
 * node joins a new network. One node can join only one network and thus it can communicate  *
 * only with one master. In order not to perform a segment erase, each time the node joins   *
 * a new network, stressing the flash memory of the microcontroller, the system takes        *
 * advantage of the fact that in this memory only 0s can be written without erasing. So,     *
 * each time a new network is joined, the new configuration record is appended to the table  *
 * and the previous record is disabled by lowering the MNE8_DISABLED bit in its flags.       *
 * An erasure of the flash segment is only performed when the table is full and a new        *
 * network is joined (every MAXMACS times the node joins a new network).                     *
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
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>

/* Application Header files */
#include <string.h>
#include "Flash/AppFlash.h"

/*********************************************************************************************
 * Definitions to control the whole code behavior                                            *
 ********************************************************************************************/


/*********************************************************************************************
 * Constants definitions                                                                     *
 ********************************************************************************************/


/*********************************************************************************************
 * Variable definitions                                                                      *
 ********************************************************************************************/
MACEntry* MasterNodeEntry;					//Pointer to the master entry in use
uint8_t NodeAddr;							//Short address of this node for this master
static uint16_t MastersCnt;					//Counter of Master entries in the Masters table


/*********************************************************************************************
 * Function declarations                                                                     *
 ********************************************************************************************/


/*********************************************************************************************
 * Main Flash Handling code                                                                  *
 ********************************************************************************************/

//********************************************************************************************
/* Finds the correct Master Data Entry in Flash memory and sets the necessary variables
*/
void InitFlash(void) {
	MACEntry* temp;							//Temporary MAC entry pointer

	MastersCnt = 0;

	/*FlashProgram and FlashSectorErase must be called from ROM or RAM and not from Flash
	memory. The same is true for their utilization variables (FlashProgram). During Flash
	programming there cannot be any read access to Flash memory, so we need a buffer in RAM
	to hold the data to be stored in Flash.*/
	NodeAddr = 0;							//Reset this node's address to broadcast
	temp = MACADDR(0);			//Point to the first Master node entry in Flash
	if((temp->MAC.mac32[0] == 0xFFFFFFFF) && (temp->MAC.mac32[1] == 0xFFFFFFFF)) {
		//Table is empty and this node has never been bonded to a master. Variables are set!
		return;
	}
	MastersCnt++;							//There is already a master node in the table

	/*Need to perform a searching loop to find the first working master node.*/
	while(((temp->flg8 & MNE8_DISABLED) == 0) && (MastersCnt <= MAXMACS)) {
		temp += MACENTRY_SIZE;
		if((temp->MAC.mac32[0] == 0xFFFFFFFF) && (temp->MAC.mac32[1] == 0xFFFFFFFF)) {
			break;
		}
		MastersCnt++;
	}
	if(MastersCnt > MAXMACS) {				//Passed the end of this segment?
		temp = MACADDR(0);					//Yes => Revert to its beginning
		MasterNodeEntry = temp;				//Set the master node to be used
		MastersCnt--;						//Bring the number of known masters into limit
		return;								//Short address is still broadcast
	}

	MasterNodeEntry = temp;					//Point to this Master Node
	NodeAddr = MasterNodeEntry->ShortAddr;	//Get the short address assigned by this master

	/*Going to skip the rest of the non empty fields of the table to find the first empty
	entry.*/
	temp += MACENTRY_SIZE;
	while((temp->MAC.mac32[0] != 0xFFFFFFFF) && (temp->MAC.mac32[1] != 0xFFFFFFFF)) {
		temp += MACENTRY_SIZE;
		MastersCnt++;
	}
}


//********************************************************************************************
/*Add a new node into masters table. If there is no room in this table then first erase the
segment and then add the new master entry.
*/
uint8_t SetMasterNode(MACEntry *InMAC) {
	uint32_t result;						//The result of every Flash programming command
	uint32_t flags;							//Gets the flags and short address of the entry

	/*Lets see if this MAC is already our active master. In that case we announce success, as
	nothing else changes and no memory write is needed. Of course, the short address should
	match, too*/
	if((InMAC->MAC.mac32[0] == MasterNodeEntry->MAC.mac32[0]) &&
		(InMAC->MAC.mac32[1] == MasterNodeEntry->MAC.mac32[1]) &&
		(InMAC->ShortAddr == MasterNodeEntry->ShortAddr)) {
		return FAPI_STATUS_SUCCESS;				//Return success
	}
	/*MastersCnt variable shows how many entries are there in the table of masters. The first
	thing that should happen is to check if the table is full. If yes, then first we have to
	erase the segment and store the new entry at its beginning.
	If the table is not full, we must disable the current master node and then add the new
	entry in this table, and also update the variables needed.*/
	if(MastersCnt == MAXMACS) {
		result = FlashSectorErase(BASE_FLASH);//Erase the sector of masters table
		if(result != FAPI_STATUS_SUCCESS) {
			return result;					//Cannot erase the sector...
		}
		MastersCnt = 0;						//Reset the number of entries used in the table
	}

	if(MastersCnt > 0) {					//Is there any used node?
		/*In case of a used master node, we have to disable it first. So get its flags and
		reset the DISABLED bit in flg8.*/
		flags = (MasterNodeEntry->flg16 << 16) || (MasterNodeEntry->flg8 << 8) ||
				(MasterNodeEntry->ShortAddr);//Got a full 32bit word to alter
		flags &= ~(MNE8_DISABLED << 8);		//Disable this master node entry
		//Now we must write the new value into Flash memory, again
		result = FlashProgram((uint8_t*)&flags, (uint32_t)MasterNodeEntry, 4);
		if(result != FAPI_STATUS_SUCCESS) {
			return result;					//Cannot program the new value...
		}
	}

	//Going to write the new entry into Flash memory
	MasterNodeEntry = MACADDR(MastersCnt);	//Get first empty cell in masters' table
	result = FlashProgram((uint8_t*)InMAC, (uint32_t)MasterNodeEntry, MACENTRY_SIZE);
	if(result != FAPI_STATUS_SUCCESS) {
		return result;						//Cannot program the new entry...
	}

	/*Now that the new entry is programmed in the table we have to set the variables,
	according to the new data.*/
	NodeAddr = MasterNodeEntry->ShortAddr;
	MastersCnt++;
	return FAPI_STATUS_SUCCESS;				//Return success
}
