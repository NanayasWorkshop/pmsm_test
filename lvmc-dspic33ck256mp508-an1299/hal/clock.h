/*******************************************************************************
  Oscillator Configuration Routine Header File

  File Name:
    clock.h

  Summary:
    This header file lists Clock Configuration related functions and definitions

  Description:
    Definitions in the file are for dsPIC33CK256MP508 on Motor Control 
    Development board from Microchip

*******************************************************************************/
#ifndef _CLOCK_H
#define _CLOCK_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************
#include <xc.h>
#include <stdint.h>
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// OSCILLATOR Related Definitions
// Oscillator frequency (Hz) - 200,000,000 Hz
#define FOSC                    200000000UL
// Oscillator frequency (MHz) - 200MHz
#define FOSC_MHZ                200U     
// Instruction cycle frequency (Hz) - 100,000,000 Hz
#define FCY                     100000000UL
// Instruction cycle frequency (MHz) - 100 MHz
#define FCY_MHZ                 100U  
        
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitOscillator(void);
void EnableREFCLKOutput(uint16_t);        
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of CLOCK_H


