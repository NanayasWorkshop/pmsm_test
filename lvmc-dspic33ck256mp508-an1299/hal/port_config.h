/*******************************************************************************
  Hardware specific routine definition and interfaces Header File

  File Name:
    port_config.h

  Summary:
    This file includes subroutine for initializing GPIO pins as analog/digital,
    input or output etc. Also to PPS functionality to Remap-able input or output 
    pins

  Description:
    Definitions in the file are for dsPIC33CK256MP508 on Motor Control 
    Development board from Microchip

*******************************************************************************/
#ifndef _PORTCONFIG_H
#define _PORTCONFIG_H

#include <xc.h>

#ifdef __cplusplus  // Provide C++ Compatibility
    extern "C" {
#endif

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// Digital I/O definitions
// Push button Switches
    
// SW1 :  (RE11)
#define SW1                   PORTEbits.RE11
// SW2 :  (RE12)
#define SW2                   PORTEbits.RE12
        
// S2 : PIM #83 - Used as START/STOP button of Motor
#define BUTTON_START_STOP        SW1
// S3 : PIM #84 - Used as Speed HALF/DOUBLE button of Motor
#define BUTTON_SPEED_HALF_DOUBLE      SW2


// Debug LEDs
// LED2(LD11) : (RE7)
#define LED2                    LATEbits.LATE7
// LED1(LD10) : (RE6)
#define LED1                    LATEbits.LATE6


// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void MapGPIOHWFunction(void);
void SetupGPIOPorts(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of PORTCONFIG_H


