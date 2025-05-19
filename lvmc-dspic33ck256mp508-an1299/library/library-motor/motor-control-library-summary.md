# dsPIC33 Motor Control Library Summary

## Overview
This document summarizes the Microchip dsPIC33 Motor Control Library, a collection of functions designed for implementing Field Oriented Control (FOC) of 3-phase motor control applications on dsPIC® Digital Signal Controllers (DSCs). The library provides optimized building blocks that applications can call in time-critical control loops.

## Library Structure

### Core Files
- **motor_control.h**: Main header file that includes all interfaces
- **motor_control_declarations.h**: Function declarations
- **motor_control_inline_declarations.h**: Inline function declarations
- **motor_control_types.h**: Data structures and type definitions
- **motor_control_inline_dspic.h**: dsPIC specific optimized implementations
- **motor_control_dsp.h**: DSP register definitions
- **motor_control_util.h**: Utility functions for computations
- **motor_control_inline_internal.h**: Implementation details not part of public API

### Library Archive
- **libmotor_control_dspic-elf.a**: Compiled library containing implementations

## Key Data Types

### Reference Frame Types
```c
/* Alpha-Beta reference frame */
typedef struct
{
    int16_t alpha;    // Alpha component
    int16_t beta;     // Beta component
} MC_ALPHABETA_T;

/* D-Q reference frame */
typedef struct
{
    int16_t d;        // D-axis component
    int16_t q;        // Q-axis component
} MC_DQ_T;

/* ABC reference frame */
typedef struct
{
    int16_t a;        // Phase A component 
    int16_t b;        // Phase B component
    int16_t c;        // Phase C component
} MC_ABC_T;

/* Sine-Cosine values */
typedef struct
{
    int16_t cos;      // Cosine component
    int16_t sin;      // Sine component
} MC_SINCOS_T;

/* PWM Duty Cycle */
typedef struct
{
    uint16_t dutycycle1;  // Duty cycle for phase #1
    uint16_t dutycycle2;  // Duty cycle for phase #2
    uint16_t dutycycle3;  // Duty cycle for phase #3
} MC_DUTYCYCLEOUT_T;
```

### Control Types
```c
/* PI Controller State */
typedef struct
{
    int32_t integrator;   // Integrator sum
    int16_t kp;           // Proportional gain
    int16_t ki;           // Integral gain
    int16_t kc;           // Anti-windup gain
    int16_t outMax;       // Maximum output limit
    int16_t outMin;       // Minimum output limit
} MC_PISTATE_T;

/* PI Controller Input */
typedef struct
{
    MC_PISTATE_T piState;     // PI state
    int16_t inReference;      // Input reference
    int16_t inMeasure;        // Input measured value
} MC_PIPARMIN_T;

/* PI Controller Output */
typedef struct
{
    int16_t out;              // Output of the PI controller
} MC_PIPARMOUT_T;
```

## Core Functions

### Clarke and Park Transformations

```c
/* Clarke Transformation: ABC -> αβ */
uint16_t MC_TransformClarke_Assembly(const MC_ABC_T *pABC, 
                                     MC_ALPHABETA_T *pAlphaBeta);

/* Park Transformation: αβ -> dq */
uint16_t MC_TransformPark_Assembly(const MC_ALPHABETA_T *pAlphaBeta, 
                                   const MC_SINCOS_T *pSinCos, 
                                   MC_DQ_T *pDQ);

/* Inverse Park Transformation: dq -> αβ */
uint16_t MC_TransformParkInverse_Assembly(const MC_DQ_T *pDQ, 
                                         const MC_SINCOS_T *pSinCos, 
                                         MC_ALPHABETA_T *pAlphaBeta);

/* Inverse Clarke Transformation: αβ -> ABC */
uint16_t MC_TransformClarkeInverse_Assembly(const MC_ALPHABETA_T *pAlphaBeta, 
                                           MC_ABC_T *pABC);

/* Variant with swapped inputs (for phase-shifted SVM) */
uint16_t MC_TransformClarkeInverseSwappedInput_Assembly(
                                                const MC_ALPHABETA_T *pAlphaBeta, 
                                                MC_ABC_T *pABC);
```

### Trigonometric Functions

```c
/* Calculate sine and cosine for a given angle */
uint16_t MC_CalculateSineCosine_Assembly_Ram(int16_t angle, 
                                            MC_SINCOS_T *pSinCos);
```

### Space Vector Modulation

```c
/* Calculate duty cycles using space vector modulation */
uint16_t MC_CalculateSpaceVector_Assembly(const MC_ABC_T *pABC, 
                                         uint16_t iPwmPeriod, 
                                         MC_DUTYCYCLEOUT_T *pDutyCycleOut);

/* Phase-shifted variant of SVM */
uint16_t MC_CalculateSpaceVectorPhaseShifted_Assembly(const MC_ABC_T *pABC, 
                                                     uint16_t iPwmPeriod, 
                                                     MC_DUTYCYCLEOUT_T *pDutyCycleOut);
```

### PI Control

```c
/* PI controller update function */
uint16_t MC_ControllerPIUpdate_Assembly(int16_t inReference, 
                                       int16_t inMeasure, 
                                       MC_PISTATE_T *pPIState, 
                                       int16_t *pPIParmOutput);
```

## Implementation Details

### Assembly Optimization
Most core functions have assembly-optimized implementations for maximum performance on dsPIC33 architecture.

### Fixed-Point Arithmetic
- The library uses Q15/Q16 fixed-point formats for calculations
- Motor variables typically use Q15 format (-1.0 to 0.9999)
- Angles typically scaled in Q16 format (0 to 2π mapped to 0 to 65535)

### DSP Features
- Makes use of dsPIC DSP accelerator for fast math operations
- Uses DSP accumulators A and B for intermediate calculations

### Function Return Values
Most functions return a status value (typically 1 for success), which can be safely ignored in most cases.

## Inline vs. Assembly Implementations
Many functions have both inline C and assembly implementations:
- `*_Assembly` suffix: Assembly-optimized implementation
- `*_InlineC` suffix: C implementation that may be inlined

## Usage Example
```c
// Example of a typical motor control sequence using the library
// 1. Measure phase currents and convert to ABC frame
MC_ABC_T iabc = {measured_ia, measured_ib, measured_ic};

// 2. Transform currents to alpha-beta frame
MC_ALPHABETA_T ialphabeta;
MC_TransformClarke_Assembly(&iabc, &ialphabeta);

// 3. Calculate sine/cosine of rotor angle
MC_SINCOS_T sincosTheta;
MC_CalculateSineCosine_Assembly_Ram(theta_electrical, &sincosTheta);

// 4. Transform currents to d-q frame
MC_DQ_T idq;
MC_TransformPark_Assembly(&ialphabeta, &sincosTheta, &idq);

// 5. PI control for d-q current
MC_PISTATE_T pi_d_state = {/*...*/};
int16_t vd_out;
MC_ControllerPIUpdate_Assembly(id_ref, idq.d, &pi_d_state, &vd_out);

MC_PISTATE_T pi_q_state = {/*...*/};
int16_t vq_out;
MC_ControllerPIUpdate_Assembly(iq_ref, idq.q, &pi_q_state, &vq_out);

// 6. Transform voltage commands back to alpha-beta
MC_DQ_T vdq = {vd_out, vq_out};
MC_ALPHABETA_T valphabeta;
MC_TransformParkInverse_Assembly(&vdq, &sincosTheta, &valphabeta);

// 7. Transform to ABC frame
MC_ABC_T vabc;
MC_TransformClarkeInverseSwappedInput_Assembly(&valphabeta, &vabc);

// 8. Calculate PWM duty cycles
MC_DUTYCYCLEOUT_T pwmDutyCycle;
MC_CalculateSpaceVectorPhaseShifted_Assembly(&vabc, pwmPeriod, &pwmDutyCycle);

// 9. Apply duty cycles to PWM hardware
// (hardware-specific code)
```
