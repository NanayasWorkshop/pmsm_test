# Step 1: Basic Encoder Integration for Hybrid FOC

## Overview

This document outlines the first phase of implementing a hybrid Field Oriented Control (FOC) system that combines the existing sensorless estimator with an incremental encoder feedback system. The encoder consists of 36 holes with two light barriers placed 90° apart, providing quadrature signals for direction detection.

## Goals for Step 1

1. Add hardware interface for the encoder signals
2. Implement basic encoder position and speed calculation
3. Verify encoder measurements independently of the FOC algorithm
4. Create a simple mechanism to switch between encoder and sensorless modes
5. Test and validate the encoder-based position feedback

## Hardware Configuration

### Encoder Specifications
- **Type**: Incremental encoder with 36 holes (mechanical resolution)
- **Signals**: Two quadrature channels (A and B) via light barriers
- **Resolution**: 
  - 36 pulses per mechanical revolution
  - 4x quadrature decoding = 144 positions per mechanical revolution

### Hardware Interface

#### Option 1: QEI Peripheral (Recommended if available)
The dsPIC33CK256MP508 includes a Quadrature Encoder Interface (QEI) peripheral which is ideal for this application:

1. **Pin Assignment**:
   - QEI1A: Connect to encoder channel A (first light barrier)
   - QEI1B: Connect to encoder channel B (second light barrier)
   - QEI1I: Not used (no index signal)

2. **Peripheral Configuration**:
   - Configure for 4x mode (count on all edges of A and B)
   - Set up position counter reset at 143 (144 positions - 1)
   - Enable QEI interrupts for velocity calculation

#### Option 2: Input Capture with External Interrupts
If QEI is not available or already in use:

1. **Pin Assignment**:
   - IC1: Connect to encoder channel A
   - External interrupt pin: Connect to encoder channel B

2. **Peripheral Configuration**:
   - Configure Input Capture for encoder pulse timing
   - Set up external interrupt for direction detection

### Circuit Integration
1. **Signal Conditioning**:
   - Add pull-up resistors (4.7kΩ recommended) to encoder signals
   - Consider adding a simple RC filter (R=1kΩ, C=100pF) for noise suppression
   - Ensure proper ground connection between encoder and LVMC board

2. **Power Supply**:
   - Use the 3.3V or 5V supply from the LVMC board to power the light barriers
   - Confirm signal levels are compatible with dsPIC33CK (3.3V logic)

## Software Implementation

### 1. Encoder Module (`encoder.c/h`)

Create a new module for encoder handling with the following functions:

```
// encoder.h - Function declarations only (no implementation shown)

// Initialize encoder hardware interface
void Encoder_Init(void);

// Reset encoder position to a known value
void Encoder_Reset(void);

// Get mechanical position (0-144 counts)
uint16_t Encoder_GetMechanicalPosition(void);

// Get electrical position (0-65535 = 0-360 degrees electrical)
int16_t Encoder_GetElectricalPosition(void);

// Get mechanical speed in RPM
int16_t Encoder_GetMechanicalSpeedRPM(void);

// Get electrical speed in internal units matching estimator output
int16_t Encoder_GetElectricalSpeed(void);

// ISR for QEI or Input Capture for speed calculation
void _QEIInterrupt(void);  // or _IC1Interrupt(void);

// Status/diagnostic functions
bool Encoder_IsConnected(void);
uint16_t Encoder_GetErrorCount(void);
```

### 2. Encoder Position Calculation

The key conversion calculations:

1. **Mechanical to Electrical Position**:
   ```
   // Convert mechanical position to electrical position
   // mechanical_position range: 0-143
   // electrical_position range: 0-65535 (Q16 format for 0-360°)
   electrical_position = (mechanical_position * NOPOLESPAIRS * 65536UL) / 144;
   ```

2. **Speed Calculation**:
   ```
   // Calculate mechanical speed using time between encoder counts or timer-based sampling
   // For timer-based (recommended for steady calculation):
   mechanical_speed_rpm = (position_change * 60 * sampling_frequency) / 144;
   
   // Convert to electrical speed
   electrical_speed = mechanical_speed_rpm * NOPOLESPAIRS;
   ```

### 3. Integration with Existing FOC Structure

#### Modifications to `estim.h`

Add new fields to the estimator structure to incorporate encoder data:

```
typedef struct
{
    // Existing fields...
    
    // New fields for encoder integration
    int16_t qRhoEncoder;          // Electrical angle from encoder
    int16_t qVelEncoder;          // Electrical speed from encoder
    uint16_t encoderValid;        // Flag indicating valid encoder signal
    uint16_t useEncoder;          // Control flag: 0=sensorless, 1=encoder, 2=hybrid
    
} ESTIM_PARM_T;
```

#### Modifications to `pmsm.c`

Update the `CalculateParkAngle()` function to use encoder data:

```
void CalculateParkAngle(void)
{
    if (uGF.bits.OpenLoop)
    {
        // Existing open-loop code...
    }
    else 
    {
        // NEW: Determine which position source to use
        if (estimator.useEncoder == 1 && estimator.encoderValid)
        {
            // Use encoder position directly
            thetaElectrical = estimator.qRhoEncoder;
        }
        else
        {
            // Use existing estimated position from PLL estimator
            thetaElectrical = estimator.qRho;
        }
        
        // Rest of existing code...
    }
}
```

Add encoder initialization to `main()`:

```
int main(void)
{
    // Existing initialization...
    
    // Add encoder initialization
    Encoder_Init();
    
    // Rest of main function...
}
```

### 4. Integration with ADC Interrupt

Update the ADC interrupt handler to read encoder data:

```
void __attribute__((__interrupt__,no_auto_psv)) _ADCInterrupt()
{
    // Existing code...
    
    if (singleShuntParam.adcSamplePoint == 0)
    {
        // Read encoder position and update estimator
        estimator.qRhoEncoder = Encoder_GetElectricalPosition();
        estimator.qVelEncoder = Encoder_GetElectricalSpeed();
        estimator.encoderValid = Encoder_IsConnected();
        
        // Rest of existing code...
    }
}
```

### 5. Simple Mode Selection

Add a simple mechanism to select between encoder and sensorless modes:

```
// In board_service.c, add a new function
void TogglePositionSource(void)
{
    estimator.useEncoder = (estimator.useEncoder + 1) % 3;  // Toggle between 0, 1, 2
    // 0 = sensorless only, 1 = encoder only, 2 = reserved for hybrid (Step 3)
}

// Update IsPressed_Button2() to add this function when in a special mode
// For example, long press of Button2 could toggle the source
```

## Testing and Validation

### 1. Basic Encoder Functionality
- Verify encoder pulses are correctly detected
- Confirm quadrature decoding works for direction detection
- Test position counter wrapping at mechanical revolution boundaries

### 2. Encoder Position and Speed
- Validate position mapping from mechanical to electrical angle
- Verify speed calculation accuracy at different RPMs
- Test position tracking through multiple revolutions

### 3. FOC Operation with Encoder
- Test motor startup using encoder position
- Verify stable operation at low speeds
- Compare operation with sensorless mode at various speeds

### 4. Diagnostics
- Add X2CScope variables for monitoring encoder position and speed
- Configure LED indicators or debug output for position source identification
- Implement error detection for encoder signals (missing pulses, noise)

## Implementation Considerations

### Timing
- Encoder reading must be synchronized with the FOC control loop
- Speed calculation requires consistent timing for accuracy
- Consider interrupt priorities to ensure encoder signals are processed promptly

### Resources
- QEI peripheral or Input Capture channels
- Additional RAM for encoder-related variables
- Additional processing time for position and speed calculations

### Error Handling
- Implement signal validation to detect disconnected or faulty encoder
- Create fallback strategy to revert to sensorless mode if encoder fails
- Add detection for implausible encoder readings

## Deliverables for Step 1

1. Complete `encoder.c/h` module with basic functionality
2. Modified `estim.c/h` with encoder integration points
3. Updated `pmsm.c` to accommodate encoder position
4. Configuration parameters in `userparms.h` for encoder settings
5. Test results documenting encoder accuracy and performance
6. Simple user interface to toggle between position sources

## Next Steps

After successful implementation of Step 1, we will proceed to:

1. Refine the encoder performance, especially at low speeds
2. Implement more sophisticated sensor fusion algorithm
3. Add adaptive weighting between encoder and sensorless estimates
4. Create smooth transitions between modes based on operating conditions

## Conclusion

This first step establishes the foundation for the hybrid FOC approach. By starting with a simple, switchable integration of the encoder, we can verify the hardware interface and basic position/speed calculations before moving to more sophisticated fusion algorithms. This modular approach reduces integration risks and allows for incremental testing and validation.
