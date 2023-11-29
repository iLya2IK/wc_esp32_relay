// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               OWIHighLevelFunctions.c
* \li Compiler:           IAR EWAAVR 3.20a
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All AVRs.
*
* \li Application Note:   AVR318 - Dallas 1-Wire(R) master.
*
*
* \li Description:        High level functions for transmission of full bytes
*                         on the 1-Wire(R) bus and implementations of ROM
*                         commands.
*
*                         $Revision: 1.7 $
*                         $Date: Thursday, August 19, 2004 14:27:18 UTC $
****************************************************************************/
#include "OWIHighLevelFunctions.h"
#include "OWIBitFunctions.h"
#include "OWIPolled.h"
#include "OWIcrc.h"

/*! \brief  Sends one byte of data on the 1-Wire(R) bus(es).
 *
 *  This function automates the task of sending a complete byte
 *  of data on the 1-Wire bus(es).
 *
 *  \param  data    The data to send on the bus(es).
 *
 *  \param  pins    A num of the pin to send the data to.
 */
void OWI_SendByte(unsigned char data, unsigned char pin)
{
    unsigned char temp;
    unsigned char i;

    // Do once for each bit
    for (i = 0; i < 8; i++)
    {
        // Determine if lsb is '0' or '1' and transmit corresponding
        // waveform on the bus.
        temp = data & 0x01;
        if (temp)
        {
            OWI_WriteBit1(pin);
        }
        else
        {
            OWI_WriteBit0(pin);
        }
        // Right shift the data to get next bit.
        data >>= 1;
    }
}


/*! \brief  Receives one byte of data from the 1-Wire(R) bus.
 *
 *  This function automates the task of receiving a complete byte
 *  of data from the 1-Wire bus.
 *
 *  \param  pin     A num of the pin to read from.
 *
 *  \return     The byte read from the bus.
 */
unsigned char OWI_ReceiveByte(unsigned char pin)
{
    unsigned char data;
    unsigned char i;

    // Clear the temporary input variable.
    data = 0x00;

    // Do once for each bit
    for (i = 0; i < 8; i++)
    {
        // Shift temporary input variable right.
        data >>= 1;
        // Set the msb if a '1' value is read from the bus.
        // Leave as it is ('0') else.
        if (OWI_ReadBit(pin))
        {
            // Set msb
            data |= 0x80;
        }
    }
    return data;
}


/*! \brief  Sends the SKIP ROM command to the 1-Wire bus(es).
 *
 *  \param  pins    A num of the pin to send the SKIP ROM command to.
 */
void OWI_SkipRom(unsigned char pin)
{
    // Send the SKIP ROM command on the bus.
    OWI_SendByte(OWI_ROM_SKIP, pin);
}


/*! \brief  Sends the READ ROM command and reads back the ROM id.
 *
 *  \param  romValue    A pointer where the id will be placed.
 *
 *  \param  pin     A num of the pin to read from.
 */
void OWI_ReadRom(unsigned char * romValue, unsigned char pin)
{
    unsigned char bytesLeft = 8;

    // Send the READ ROM command on the bus.
    OWI_SendByte(OWI_ROM_READ, pin);

    // Do 8 times.
    while (bytesLeft > 0)
    {
        // Place the received data in memory.
        *romValue++ = OWI_ReceiveByte(pin);
        bytesLeft--;
    }
}


/*! \brief  Sends the MATCH ROM command and the ROM id to match against.
 *
 *  \param  romValue    A pointer to the ID to match against.
 *
 *  \param  pins    A num of the pin to perform the MATCH ROM command on.
 */
void OWI_MatchRom(unsigned char * romValue, unsigned char pin)
{
    unsigned char bytesLeft = 8;

    // Send the MATCH ROM command.
    OWI_SendByte(OWI_ROM_MATCH, pin);

    // Do once for each byte.
    while (bytesLeft > 0)
    {
        // Transmit 1 byte of the ID to match.
        OWI_SendByte(*romValue++, pin);
        bytesLeft--;
    }
}
