/*********************************************************************************************
* PitFall_SlaveNode Project Main Header File                                                 *
*============================================================================================*
*  Created on: Sep 26, 2016                                                                  *
*      Author: eliaschr                                                                      *
* (c) Elias Chrysocheris and Iraklis Rigakis                                                 *
*--------------------------------------------------------------------------------------------*
* In this file only the necessary definitions appear that needed to be known to other files  *
* of the project.                                                                            *
* Definitions:                                                                               *
*--------------                                                                              *
* EVENTSARRAY_SIZE defines the size of the events array, in records. This array is needed to *
*   hold the events sent by the MSP subsystem of the hardware                                *
*                                                                                            *
* Types Definitions:                                                                         *
*--------------------                                                                        *
* *CallbackFunc defines the type of a callback function. Needed for the functions that will  *
*   be executed when a key event appears.                                                    *
* EventStamp is the type that defines a whole event record. Care must be taken in its fields *
*   to be aligned, according to their memory needs (uint16 values should be word aligned,    *
*   uint32 values should be 32bit aligned, etc.). An EventStamp contains the following       *
*   fields:                                                                                  *
*     -Year: Contains the offset of the year of the recorded event since 2000. 8 bits value. *
*     -TypeMonth: Contains the type of event at its high nibble and the current month at its *
*       lower nibble. An event, can be a bug event (caused by a sensor interrupt), a         *
*       temperature event (used to take periodic measurements of the temperature), or a      *
*       reset request from the user. The month can be 1 to 12 (1 based). Total of 8 bits     *
*       value.                                                                               *
*     -Day: Contains the day of the event. It can be 1 to 31 (according to month, of course) *
*       8 bits value.                                                                        *
*     -Hour: Contains the hour the event happened. 8 bits value.                             *
*     -Minute: Contains the minute the event happened. 8 bits value.                         *
*     -Second: Contains the second the event happened. 8 bits value.                         *
*     -Temperature: Contains the ambient temperature at the time of the event. It is a 16    *
*       bits value, containing the unprocessed ADC value from the temperature sensor ADC     *
*       channel.                                                                             *
* NodePars is a structure that contains parameters of the node itself. It contains the       *
*   following parameters:                                                                    *
*     -CALADC12_15V_30C: It is a 16 bit value, used for temperature ADC value calibration.   *
*       It holds the calibration value at 30 degrees Celcius. More on this value can be      *
*       found at the MSP430F5xx Family User's Guide (Texas Instruments slau208) and the      *
*       MSP430F541xA and MSP430F543xA Datasheet (Texas Instruments slas655).                 *
*     -CALADC12_15V_85C: It is a 16 bit value, used for temperature ADC value calibration.   *
*       It holds the calibration value at 85 degrees Celcius. More on this value can be      *
*       found at the MSP430F5xx Family User's Guide (Texas Instruments slau208) and the      *
*       MSP430F541xA and MSP430F543xA Datasheet (Texas Instruments slas655).                 *
*     -MasterCounter is a 16 bit value that holds the total number of recorded bug events    *
*       since the last reset command from the user.                                          *
*     -Battery contains the measured ADC value of the battery level. It is 16 bits.          *
*                                                                                            *
* Global Variables:                                                                          *
*-------------------                                                                         *
* In the main file there are some variables declared, that needed to be visible from other   *
* files of the project. These are:                                                           *
* ledPinHandle: The PIN Driver of the TI-RTOS needs a handle to manipulate the associated    *
*   pins and observe their status. This is the handle for the pins used for led indications. *
* swPinHandle: The same driver is used for the I/O pins connected to the switches used for   *
*   user interaction.                                                                        *
* EventsArray: is the array that holds the events acknowledged by the MSP430 sybsystem of    *
*   the PitFall trap's hardware. It contains up to EVENTSARRAY_SIZE (defined earlier) of     *
*   EventStamp records.                                                                      *
* NodeParams: Is an array that holds the node's parameters from MSP430. It is of type        *
*   NodePars (defined earlier)                                                               *
* EventsStrt, EventsLen: The EventsArray is used as a cyclic buffer, so an offset to the     *
*   first element stored (EventsStrt) and the number of stored records (EventsLen) needed.   *
*   They are both 8 bits values.                                                             *
*                                                                                            *
* Global Functions:                                                                          *
*-------------------                                                                         *
* There are also some functions needed to be accessible by other parts of the application.   *
* These are:                                                                                 *
* Halt_abort(): It is used for debugging purposes and enters an infinite loop. Causes the    *
*   Red LED of the board to light up and indicate an error. The parameters are for easier    *
*   information about the reason the system entered this function. The first parameter is a  *
*   pointer to a string describing the error and the second is a pointer to a value needed   *
*   to be inspected by the developer for further understanding of the cause of the problem.  *
* registerButtonCb(): Is needed to inform the system that a function should be called upon a *
*   button press (or any other I/O interrupt event). It takes two parameters, the first is   *
*   the Pin ID that triggers the callback function and the second is the callback function   *
*   to be called when that pin triggers an event.                                            *
*                                                                                            *
* DISCLAMER NOTICE:                                                                          *
*-------------------                                                                         *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY        *
* EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF    *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE *
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,   *
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF         *
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)     *
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR   *
* TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS         *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                               *
*********************************************************************************************/

#ifndef PITFALL_SLAVENODE_H_
#define PITFALL_SLAVENODE_H_

#include <stdint.h>
#include <ti/drivers/PIN.h>


/*********************************************************************************************
* Definitions                                                                                *
*********************************************************************************************/
#define EVENTSARRAY_SIZE	64				//Sets the size of the events array. The events
											// come from the MSP430 optical sensor module
											// through UART.


/*********************************************************************************************
* New types definitions                                                                      *
*********************************************************************************************/
typedef void (*CallbackFunc)(void);
typedef struct {
	uint8_t Year;							//Year of the event
	uint8_t TypeMonth;						//Month of the event and Type of Event Record
	uint8_t Day;							//Day of the event
	uint8_t Hour;							//Hour of the event
	uint8_t Minute;							//Minute of the event
	uint8_t Second;							//Second of the event
	uint16_t Temperature;					//Ambient temperature at the time of the event
} EventStamp;
typedef struct {
	uint16_t CALADC12_15V_30C;				//Temperature sensor Calibration value for 30C
	uint16_t CALADC12_15V_85C;				//Temperature Sensor Calibration value for 85C
	uint16_t MasterCounter;					//MasterCoutner of events of this node
	uint16_t Battery;						//Last Battery measurement
} NodePars;


/*********************************************************************************************
 * Variables global to other parts of the code                                               *
 ********************************************************************************************/
extern PIN_Handle ledPinHandle;				//Handle for LEDs manipulation
extern PIN_Handle swPinHandle;				//Handle for Key Switches
extern EventStamp EventsArray[EVENTSARRAY_SIZE];	//Array of timestamps MSP has recorded
extern NodePars NodeParams;					//Node's parameters from MSP
extern uint8_t EventsStrt;					//Starting index for the first event in queue
extern uint8_t EventsLen;					//Number of events recorded in the EventsArray


/*********************************************************************************************
 * Function other parts of the code need to know about                                       *
 ********************************************************************************************/
void Halt_abort(char* ErrorTxt, void* ptr);
void registerButtonCb(PIN_Id pinId, CallbackFunc cbFxn);

#endif /* PITFALL_SLAVENODE_H_ */
