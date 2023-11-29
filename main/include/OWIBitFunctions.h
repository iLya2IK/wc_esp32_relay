// This file has been prepared for Doxygen automatic documentation generation.
/*! \file ********************************************************************
*
* Atmel Corporation
*
* \li File:               OWIBitFunctions.h
* \li Compiler:           IAR EWAAVR 3.20a
* \li Support mail:       avr@atmel.com
*
* \li Supported devices:  All AVRs.
*
* \li Application Note:   AVR318 - Dallas 1-Wire(R) master.
*
*
* \li Description:        Header file for OWIBitFunctions.c
*
*                         $Revision: 1.7 $
*                         $Date: Thursday, August 19, 2004 14:27:18 UTC $
****************************************************************************/

#ifndef _OWI_BIT_FUNCTIONS_H_
#define _OWI_BIT_FUNCTIONS_H_

#include "OWIdefs.h"
#include "OWIPolled.h"

#ifdef OWI_SOFTWARE_DRIVER
void OWI_Init(unsigned char pins);
void OWI_WriteBit1(unsigned char pins);
void OWI_WriteBit0(unsigned char pins);
unsigned char OWI_ReadBit(unsigned char pins);
unsigned char OWI_DetectPresence(unsigned char pins);
#endif

#define OWI_DetectFinished OWI_ReadBit

#define OUT_OFF 0
#define OUT_ON  1

/****************************************************************************
 Macros
****************************************************************************/
/*! \brief Pull 1-Wire bus low.
 *
 *  This macro sets the direction of the 1-Wire pin(s) to output and
 *  pull the line(s) low.
 *
 *  \param bitMask  A num of the pin to pull low.
 */
#define OWI_PULL_BUS_LOW(bitMask) \
            gpio_set_direction(bitMask, GPIO_MODE_OUTPUT); \
            gpio_set_level(bitMask, OUT_OFF);


/*! \def    OWI_RELEASE_BUS(bitMask)
 *
 *  \brief  Release the bus.
 *
 *  This macro releases the bus and enables the internal pull-up if
 *  it is used.
 *
 *  \param  bitMask A num of the pin to release.
 */
#ifdef OWI_USE_INTERNAL_PULLUP
// Set 1-Wire pin(s) to input and enable internal pull-up resistor.
#define OWI_RELEASE_BUS(bitMask) \
            gpio_set_direction(bitMask, GPIO_MODE_INPUT); \
            gpio_set_level(bitMask, OUT_OFF);

#else
// Set 1-Wire pin(s) to input mode. No internal pull-up enabled.
#define OWI_RELEASE_BUS(bitMask) \
            gpio_set_direction(bitMask, GPIO_MODE_INPUT); \
            gpio_set_level(bitMask, OUT_OFF);

#endif


#endif
