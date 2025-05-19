/*******************************************************************************
  ADC Configuration Routine Header File

  File Name:
    adc.h

  Summary:
    This header file lists ADC Configuration related functions and definitions

  Description:
    Definitions in the file are for dsPIC33CK256MP508 on Motor Control 
    Development board from Microchip

*******************************************************************************/
#ifndef _ADC_H
#define _ADC_H

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
#include "userparms.h"
// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************
// ADC MODULE Related Definitions
#define ADCBUF_INV_A_IPHASE1    -ADCBUF0
#define ADCBUF_INV_A_IPHASE2    -ADCBUF1
#define ADCBUF_INV_A_IBUS       ADCBUF4
        
#define ADCBUF_SPEED_REF_A      ADCBUF11
#define ADCBUF_VBUS_A           ADCBUF15
#define ADCBUF_MOSFET_TEMP_A    ADCBUF12

/* This defines number of current offset samples for averaging 
 * If the 2^n samples are considered specify n(in this case 2^7(= 128)=> 7*/
#define  CURRENT_OFFSET_SAMPLE_SCALER         7
#ifdef SINGLE_SHUNT       
#define EnableADCInterrupt()   _ADCAN4IE = 1
#define DisableADCInterrupt()  _ADCAN4IE = 0
#define ClearADCIF()           _ADCAN4IF = 0
#define ClearADCIF_ReadADCBUF() ADCBUF4
        
#define _ADCInterrupt _ADCAN4Interrupt  
#else
 #define EnableADCInterrupt()   _ADCAN1IE = 1
#define DisableADCInterrupt()  _ADCAN1IE = 0
#define ClearADCIF()           _ADCAN1IF = 0
#define ClearADCIF_ReadADCBUF() ADCBUF1
        
#define _ADCInterrupt _ADCAN1Interrupt  
#endif
        
// *****************************************************************************
// *****************************************************************************
// Section: Interface Routines
// *****************************************************************************
// *****************************************************************************
void InitializeADCs(void);

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of ADC_H

