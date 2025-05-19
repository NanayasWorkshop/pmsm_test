# Hardware Abstraction Layer (HAL) Summary

## Overview

The HAL folder contains hardware-specific implementations that interface between the high-level motor control algorithms and the dsPIC33CK256MP508 microcontroller peripherals. These files abstract the hardware details, making the motor control code more portable.

## Critical HAL Components for Single-Shunt FOC

### ADC Module (`adc.c/h`)

The ADC configuration is critical for the single-shunt algorithm as it requires precise timing of current measurements.

#### Key Features:
- Configures ADC cores for 12-bit resolution
- Sets up synchronized ADC sampling with PWM
- Configures multiple trigger sources:
  ```c
  // For single-shunt reconstruction, two precise triggers are used
  ADTRIG1Lbits.TRGSRC4 = 0x5;  // Second trigger source (PWM1 Trigger 2)
  ADTRIG2Hbits.TRGSRC11 = 0x1; // Software trigger for additional measurements
  ```
- Defines conversion channels:
  ```c
  // Current sensing channels
  #define ADCBUF_INV_A_IPHASE1    -ADCBUF0  // Phase A current 
  #define ADCBUF_INV_A_IPHASE2    -ADCBUF1  // Phase B current
  #define ADCBUF_INV_A_IBUS       ADCBUF4   // DC Bus current (single-shunt)
  ```
- Sets up interrupts for synchronized processing
- Provides macros to control ADC operation:
  ```c
  #define EnableADCInterrupt()   _ADCAN4IE = 1
  #define DisableADCInterrupt()  _ADCAN4IE = 0
  #define ClearADCIF()           _ADCAN4IF = 0
  ```

### PWM Module (`pwm.c/h`)

PWM configuration is essential for implementing space vector modulation with specific timing for current sampling windows.

#### Key Features:
- Configures PWM for center-aligned dual-edge mode (critical for single-shunt):
  ```c
  #ifdef SINGLE_SHUNT
      PG1CONLbits.MODSEL = 6;  // Dual Edge Center-Aligned PWM mode
  #else
      PG1CONLbits.MODSEL = 4;  // Center-Aligned PWM mode
  #endif
  ```
- Sets up multiple PWM generators (1, 2, 4) for three-phase control
- Configures PWM triggers for ADC synchronization:
  ```c
  // ADC Trigger setup for current sampling
  PG1TRIGA = ADC_SAMPLING_POINT;  // First trigger
  ```
- Implements bootstrap capacitor charging sequence
- Defines PWM period and dead-time:
  ```c
  #define DEADTIME_MICROSEC       1.0
  #define LOOPTIME_MICROSEC       50
  #define DDEADTIME               (uint16_t)(DEADTIME_MICROSEC*FOSC_MHZ)
  #define LOOPTIME_TCY            (uint16_t)(((LOOPTIME_MICROSEC*FOSC_MHZ)/2)-1)
  ```
- Provides functions to update PWM duty cycles:
  ```c
  void PWMDutyCycleSetDualEdge(MC_DUTYCYCLEOUT_T *, MC_DUTYCYCLEOUT_T *);
  void PWMDutyCycleSet(MC_DUTYCYCLEOUT_T *);
  ```

### Board Service (`board_service.c/h`)

Provides system-level functions for the motor control board.

#### Key Features:
- Button debouncing and control functions:
  ```c
  bool IsPressed_Button1(void);  // START/STOP button
  bool IsPressed_Button2(void);  // Speed control button
  ```
- PWM output control functions:
  ```c
  void DisablePWMOutputsInverterA(void);
  void EnablePWMOutputsInverterA(void);
  void ClearPWMPCIFaultInverterA(void);
  ```
- Fault handling routines
- Board initialization sequence

### Clock Configuration (`clock.c/h`)

Configures the system clock for precise timing requirements.

#### Key Features:
- Sets up 200 MHz system clock with PLL:
  ```c
  #define FOSC                    200000000UL  // Oscillator frequency (Hz)
  #define FCY                     100000000UL  // Instruction cycle frequency
  ```
- PLL configuration for optimal performance:
  ```c
  PLLFBDbits.PLLFBDIV = 150;  // PLL multiplier
  CLKDIVbits.PLLPRE = 1;      // PLL pre-divider
  PLLDIVbits.POST1DIV = 3;    // PLL post-divider 1
  ```

### Comparator (`cmp.c/h`)

Configures analog comparators for fault detection.

#### Key Features:
- Over-current protection setup:
  ```c
  // Set reference level for current limit detection
  CMP1_ReferenceSet(uint16_t data);
  ```
- Comparator initialization for fault monitoring

### Measurement Functions (`measure.c/h`)

Provides signal conditioning for analog feedback signals.

#### Key Features:
- Current measurement and offset calibration:
  ```c
  void MCAPP_MeasureCurrentCalibrate(MCAPP_MEASURE_T *);
  void MCAPP_MeasureCurrentOffset(MCAPP_MEASURE_T *);
  ```
- Temperature measurement and filtering:
  ```c
  void MCAPP_MeasureTemperature(MCAPP_MEASURE_T *, int16_t);
  ```
- Averaging filters for signal noise reduction

### Port Configuration (`port_config.c/h`)

Sets up GPIO pins for motor control functions.

#### Key Features:
- Configures pins for PWM outputs
- Sets up analog inputs for current and voltage sensing:
  ```c
  // Current sensing pins configuration
  ANSELAbits.ANSELA0 = 1;  // Ia Out
  TRISAbits.TRISA0 = 1;    // Pin 16: OA1OUT/AN0
  
  ANSELBbits.ANSELB2 = 1;  // Ib Out
  TRISBbits.TRISB2 = 1;    // PIN 41: OA2OUT/AN1
  
  ANSELAbits.ANSELA4 = 1;  // Ibus Out
  TRISAbits.TRISA4 = 1;    // PIN 23: OA3OUT/AN4
  ```
- Configures pins for operational amplifiers (internal/external)
- Sets up indicator LEDs and buttons
- Configures communication interfaces

### UART Interface (`uart1.c/h`)

Provides serial communication for debugging and monitoring.

#### Key Features:
- Configures UART for 115200 baud communication
- Used by X2CScope for real-time monitoring
- Provides transmission and reception functions:
  ```c
  inline static void UART1_DataWrite(uint16_t data);
  inline static uint16_t UART1_DataRead(void);
  ```

## Hardware Configuration Options

### Internal vs External Op-Amp Configuration

The HAL supports both internal op-amps (built into the dsPIC) and external op-amps for current sensing:

```c
#ifdef INTERNAL_OPAMP_CONFIG
    // Configure internal op-amps
    AMPCON1Hbits.NCHDIS2 = 0;    // Wide input range for Op Amp #2
    AMPCON1Lbits.AMPEN2 = 1;     // Enables Op Amp #2
    // Additional op-amp configuration...
    AMPCON1Lbits.AMPON = 1;      // Enable op-amp modules
#endif
```

### Single-Shunt vs. Dual-Shunt Current Sensing

The HAL provides different ADC configurations based on the current sensing method:

```c
#ifdef SINGLE_SHUNT
    // Enable ADC for DC bus current sampling with multiple triggers
    _IE4 = 1;
    _ADCAN4IP = 7;
    // Configure multiple triggers for the single-shunt algorithm
    ADTRIG1Lbits.TRGSRC4 = 0x5;  // PWM trigger 2
#else
    // Configure dual-shunt sampling
    _IE1 = 1;
    _ADCAN1IP = 7;
    // Configure phase current sampling
    ADTRIG2Hbits.TRGSRC11 = 0x4; // PWM trigger 1
#endif
```

## Key Hardware Timing Parameters

For single-shunt current reconstruction, timing is critical:

```c
// PWM frequency (directly affects current sampling)
#define PWMFREQUENCY_HZ         20000

// Critical timing parameters for current sampling
#define SSTCRITINSEC            3.0E-6  // Minimum current sampling window
#define SSTCRIT                 (uint16_t)(SSTCRITINSEC*FCY*2)
#define SS_SAMPLE_DELAY         100     // Delay to avoid switching noise
```

## Device Configuration

The `device_config.c` file contains crucial fuse settings for the dsPIC33CK256MP508:

```c
// Clock configuration
#pragma config FNOSC = FRC              // Internal Fast RC oscillator
#pragma config FCKSM = CSECMD           // Clock switching enabled

// Watchdog and reset configuration
#pragma config RWDTPS = PS1048576       // Watchdog timer prescaler
#pragma config WINDIS = ON              // Non-Window watchdog mode

// Debug configuration
#pragma config ICS = PGD3               // ICD Communication Channel
#pragma config JTAGEN = OFF             // JTAG disabled
```
