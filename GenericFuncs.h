/*********************************************************************************************
 * GenericFuncs.h                                                                            *
 *===========================================================================================*
 *  Created on: Aug 28, 2016                                                                 *
 *      Author: eliaschr                                                                     *
 * (c) Elias Chrysocheris and Iraklis Rigakis                                                *
 *-------------------------------------------------------------------------------------------*
 * In this file only some definitions appear needed to be accessible by other parts of the   *
 * application, for the generic functions.                                                   *
 *                                                                                           *
 * Definitions:                                                                              *
 *--------------                                                                             *
 * DEF_CRC16 is the initial value of the CRC16 checksum. It is used as the old CRC16 value   *
 *   the first time a CRC16 checksum is needed to be calculated.                             *
 * CRC16_POLY defines the polynomial used for calculating a CRC16 checksum.                  *
 *                                                                                           *
 * Global Functions:                                                                         *
 *-------------------                                                                        *
 * ParseNumber(): Parses an input string as if it is a number. The number can be decimal,    *
 *   binary, or hexadecimal. The notation used for the numbers can be C (0b or 0x prefix),   *
 *   or assembly (b or h postfix). Stores the calculated value (16 bits) in a pointed        *
 *   variable and returns the number of characters used for parsing, including the           *
 *   terminating ones.                                                                       *
 * ParseNumber32(): It is the same as ParseNumber but can handle output values of 32 bits.   *
 * FindCRLF(): Finds the first occurrence of a character sequence of CRLF and counts the     *
 *   offset from the beginning of the string.                                                *
 * SkipCRLF(): Counts the number of characters of CRLF sequences until the next valid text   *
 *   character.                                                                              *
 * Hex2Int(): Converts an ASCII Hexadecimal number to a 16 bit integer.                      *
 * Byte2Hex(): Converts a byte value to its ASCII representation in a pointed buffer.        *
 * Int2Ascii(): Converts an integer value to its ASCIIZ representation in a pointed buffer.  *
 * StrSize(): Returns the size of a string in characters, without the terminating one.       *
 * CalcCRC16(): Calculates the CRC16 of a byte stream. It is called once for each byte that  *
 *   should be included into the checksum.                                                   *
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

#ifndef GENERICFUNCS_H_
#define GENERICFUNCS_H_

/*********************************************************************************************
 * Includes                                                                                  *
 ********************************************************************************************/
#include <stdint.h>


/*********************************************************************************************
 * Definitions                                                                               *
 ********************************************************************************************/
#define DEF_CRC16	0xFFFF					//Default value of CRC16 checksum at the first
											// byte
#define CRC16_POLY	0x8005					//Defines the polynomial used for CRC16 calcu-
											// lation.

/*********************************************************************************************
 * Function declarations, functions other code needs to know about                           *
 ********************************************************************************************/
int8_t ParseNumber(char* InStr, uint16_t* OutVal);	//Parses a string as a Bin, Hex or Dec
int8_t ParseNumber32(char* InStr, uint32_t* OutVal);//Parses a string as a Bin, Hex or Dec
int16_t SkipCRLF(char* InBuffer, int16_t InSize);	//Counts the CRLF characters in InBuffer
int16_t FindCRLF(char* InBuffer, int16_t InSize);	//Counts the number of valid characters
													// until the first CR or LF character
int16_t Hex2Int(char *InVal, uint8_t Len);			//Converts ASCII Hex number to integer
void Byte2Hex(uint8_t InVal, char *OutStr);		//Converts byte value to ASCII Hex string
int16_t Int2Ascii(int16_t InVal, char* Buffer, int16_t BufLen);
													//Converts and integer number into an
													// ASCIIZ representation in a buffer
uint16_t StrSize(char* InStr);						//Finds the length of the input string
uint16_t CalcCRC16(uint8_t InData, uint16_t OldCRC);//Calculates CRC16 of data

#endif /* GENERICFUNCS_H_ */
