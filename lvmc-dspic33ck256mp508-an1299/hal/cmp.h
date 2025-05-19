// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * cmp.h
 *
 * This header file lists interface functions - configuring and enabling the 
 * Comparator Modules and its output
 * 
 * Definitions in this file are for dsPIC33CK256MP508.
 * 
 * Component: HAL - CMP
 * 
 */
// </editor-fold>

#ifndef __CMP_H
#define __CMP_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
    
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__

#include <stdint.h>
#include <stdbool.h>

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
                
// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
            
void CMP_Initialize(void);

void CMP1_ModuleEnable(bool);
void CMP1_ReferenceSet(uint16_t );

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __CMP_H
    