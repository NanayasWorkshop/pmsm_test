#ifndef __BOARD_SERVICE_H
#define __BOARD_SERVICE_H

#include <stdint.h>
#include <stdbool.h>

#include <xc.h>
#include "motor_control_noinline.h"
#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

/* Button Scanning State

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef enum tagBUTTON_STATE
{
    BUTTON_NOT_PRESSED = 0, // wait for button to be pressed
    BUTTON_PRESSED = 1, // button was pressed, check if short press / long press
    BUTTON_DEBOUNCE = 2
} BUTTON_STATE;
    
// *****************************************************************************
/* Button data type

  Description:
    This structure will host parameters required by Button scan routine.
 */
typedef struct
{
   BUTTON_STATE state;
   uint16_t debounceCount;
   bool logicState;
   bool status;
} BUTTON_T;

/** Button De-bounce in milli Seconds */
#define	BUTTON_DEBOUNCE_COUNT      30
/** The board service Tick is set as 1 millisecond - specify the count in terms 
    PWM ISR cycles (i.e. BOARD_SERVICE_TICK_COUNT = 1 milli Second / PWM period)*/
#define BOARD_SERVICE_TICK_COUNT   20


extern void DisablePWMOutputsInverterA(void);
extern void EnablePWMOutputsInverterA(void);
extern void ClearPWMPCIFaultInverterA(void);
extern void BoardServiceInit(void);
extern void BoardServiceStepIsr(void);
extern void BoardService(void);
extern bool IsPressed_Button1(void);
extern bool IsPressed_Button2(void);
extern void InitPeripherals(void);
extern void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *,MC_DUTYCYCLEOUT_T *);
extern void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);
extern void pwmDutyCycleLimitCheck(MC_DUTYCYCLEOUT_T *,uint16_t,uint16_t);

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_SERVICE_H */
