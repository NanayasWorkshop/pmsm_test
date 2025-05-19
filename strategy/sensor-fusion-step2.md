# Step 2: Implementing Sensor Fusion for Hybrid FOC

## Overview

This document outlines the second phase of our hybrid Field Oriented Control (FOC) system implementation. Building upon the basic encoder integration established in Step 1, we now focus on developing a sophisticated sensor fusion algorithm that combines the strengths of both the sensorless estimator and the encoder feedback.

## Goals for Step 2

1. Design and implement a robust sensor fusion algorithm
2. Develop adaptive weighting mechanisms based on operating conditions
3. Create smooth transitions between different operating modes
4. Implement confidence metrics for both position sources
5. Optimize performance across the entire speed range
6. Test and validate the hybrid control scheme

## Sensor Fusion Strategy

### 1. Fusion Algorithm Options

#### Option A: Complementary Filter (Recommended for First Implementation)

A complementary filter offers a computationally efficient approach that combines:
- High-pass filtered encoder data (accurate at low speeds and transients)
- Low-pass filtered estimator data (reliable at higher speeds)

Basic implementation equation:
```
theta_fused = alpha * theta_encoder + (1-alpha) * theta_estimator;
```

Where `alpha` is adaptively tuned based on operating conditions.

#### Option B: Kalman Filter

A Kalman filter provides more sophisticated fusion with:
- Statistical modeling of measurement and process noise
- Prediction and correction phases
- Optimal estimation based on error covariance

While more powerful, it requires additional computational resources and careful tuning.

#### Option C: Weighted Average with Confidence Metrics

A direct weighted average approach:
```
theta_fused = (w_encoder * theta_encoder + w_estimator * theta_estimator) / (w_encoder + w_estimator);
```

Where weights are calculated based on confidence metrics.

### 2. Adaptive Weight Determination

The fusion algorithm's effectiveness depends on properly adjusting weights based on operating conditions:

1. **Speed-Based Weighting**:
   - Low speed range (0-10% of nominal): High encoder weight (80-100%)
   - Transition range (10-30% of nominal): Gradually shift from encoder to estimator
   - Normal range (30-90% of nominal): Balanced weights based on confidence
   - High speed range (>90% of nominal): Higher estimator weight (60-80%)

2. **Acceleration-Based Adjustment**:
   - During high acceleration/deceleration: Increase encoder weight
   - During steady-state operation: Balance weights based on speed range

3. **Error-Based Adjustment**:
   - Monitor difference between encoder and estimator angles
   - If difference exceeds threshold, favor more reliable source
   - Implement error accumulation with decay to track persistent discrepancies

## Software Implementation

### 1. New Fusion Module (`fusion.c/h`)

Create a dedicated module to handle sensor fusion:

```
// fusion.h - Function declarations only (no implementation shown)

// Initialize fusion algorithm parameters
void Fusion_Init(void);

// Main fusion algorithm
int16_t Fusion_CalculatePosition(int16_t estimator_position, int16_t encoder_position);
int16_t Fusion_CalculateSpeed(int16_t estimator_speed, int16_t encoder_speed);

// Update confidence metrics
void Fusion_UpdateConfidence(void);

// Calculate adaptive weights
void Fusion_UpdateWeights(int16_t speed, int16_t acceleration);

// Get diagnostic information
void Fusion_GetDiagnostics(FUSION_DIAG_T* diagnostics);

// Fusion algorithm selection
void Fusion_SetAlgorithm(FUSION_ALGORITHM_T algorithm);
```

### 2. Fusion Algorithm Implementation

Complementary Filter implementation:

```
// In fusion.c (pseudocode)
int16_t Fusion_CalculatePosition(int16_t estimator_position, int16_t encoder_position)
{
    // Normalize angle difference (handle wraparound)
    int16_t angle_diff = NormalizeAngle(encoder_position - estimator_position);
    
    // Apply complementary filter
    int16_t filtered_diff = ApplyLowPassFilter(angle_diff, fusion.filter_coef);
    
    // Combine
    int16_t fused_position = NormalizeAngle(estimator_position + filtered_diff);
    
    // Store for diagnostics
    fusion.last_fused_position = fused_position;
    fusion.last_position_diff = angle_diff;
    
    return fused_position;
}
```

### 3. Confidence Metrics Calculation

Add logic to assess the reliability of each sensor:

```
// In fusion.c (pseudocode)
void Fusion_UpdateConfidence(void)
{
    // For encoder
    // 1. Check for implausible speed changes
    int16_t encoder_accel = encoder.speed - encoder.last_speed;
    if (abs(encoder_accel) > ENCODER_MAX_ACCEL)
        fusion.encoder_confidence -= CONFIDENCE_DECREMENT;
    else
        fusion.encoder_confidence += CONFIDENCE_INCREMENT;
    
    // 2. Check for missed pulses
    if (encoder.error_count > 0)
        fusion.encoder_confidence -= encoder.error_count * ERROR_CONFIDENCE_FACTOR;
    
    // Limit confidence value
    fusion.encoder_confidence = LIMIT(fusion.encoder_confidence, MIN_CONFIDENCE, MAX_CONFIDENCE);
    
    // Similar logic for estimator confidence
    // Usually based on back-EMF signal strength and operating speed
    if (abs(estimator.qVelEstim) < LOW_SPEED_THRESHOLD)
        fusion.estimator_confidence = ScaleConfidenceBySpeed(estimator.qVelEstim);
    else
        fusion.estimator_confidence += CONFIDENCE_INCREMENT;
    
    fusion.estimator_confidence = LIMIT(fusion.estimator_confidence, MIN_CONFIDENCE, MAX_CONFIDENCE);
}
```

### 4. Adaptive Weight Calculation

Implement logic to adjust weights based on operating conditions:

```
// In fusion.c (pseudocode)
void Fusion_UpdateWeights(int16_t speed, int16_t acceleration)
{
    // Base weights on speed range
    if (abs(speed) < LOW_SPEED_THRESHOLD)
    {
        // Low speed: favor encoder
        fusion.encoder_weight_base = HIGH_WEIGHT;
        fusion.estimator_weight_base = LOW_WEIGHT;
    }
    else if (abs(speed) < MID_SPEED_THRESHOLD)
    {
        // Transition range: linear interpolation
        int16_t ratio = LINEAR_INTERPOLATE(abs(speed), LOW_SPEED_THRESHOLD, MID_SPEED_THRESHOLD);
        fusion.encoder_weight_base = HIGH_WEIGHT - ratio * (HIGH_WEIGHT - MID_WEIGHT);
        fusion.estimator_weight_base = LOW_WEIGHT + ratio * (MID_WEIGHT - LOW_WEIGHT);
    }
    else if (abs(speed) < HIGH_SPEED_THRESHOLD)
    {
        // Normal operation: balanced
        fusion.encoder_weight_base = MID_WEIGHT;
        fusion.estimator_weight_base = MID_WEIGHT;
    }
    else
    {
        // High speed: favor estimator
        fusion.encoder_weight_base = LOW_WEIGHT;
        fusion.estimator_weight_base = HIGH_WEIGHT;
    }
    
    // Adjust for acceleration
    if (abs(acceleration) > ACCEL_THRESHOLD)
    {
        // During transients, slightly favor encoder
        fusion.encoder_weight_base += ACCEL_WEIGHT_ADJUST;
        // Ensure we don't exceed maximum
        fusion.encoder_weight_base = MIN(fusion.encoder_weight_base, MAX_WEIGHT);
    }
    
    // Apply confidence modifiers
    fusion.encoder_weight = (fusion.encoder_weight_base * fusion.encoder_confidence) / MAX_CONFIDENCE;
    fusion.estimator_weight = (fusion.estimator_weight_base * fusion.estimator_confidence) / MAX_CONFIDENCE;
    
    // Ensure minimum weights to prevent division by zero
    fusion.encoder_weight = MAX(fusion.encoder_weight, MIN_WEIGHT);
    fusion.estimator_weight = MAX(fusion.estimator_weight, MIN_WEIGHT);
}
```

### 5. Integration with Estimator Module

Modify the `estim.c/h` files to incorporate the fusion results:

```
// In estim.c (pseudocode)
void Estim(void)
{
    // Original sensorless estimation code
    // ...
    
    // Update encoder position and speed
    estimator.qRhoEncoder = Encoder_GetElectricalPosition();
    estimator.qVelEncoder = Encoder_GetElectricalSpeed();
    
    // Calculate acceleration for weight adjustment
    int16_t acceleration = estimator.qVelEstim - estimator.qVelEstimPrev;
    estimator.qVelEstimPrev = estimator.qVelEstim;
    
    // Update fusion confidence and weights
    Fusion_UpdateConfidence();
    Fusion_UpdateWeights(estimator.qVelEstim, acceleration);
    
    // Apply sensor fusion for final position and speed
    if (estimator.useEncoder == 2) // Hybrid mode
    {
        // Use fusion algorithm
        estimator.qRhoFused = Fusion_CalculatePosition(estimator.qRho, estimator.qRhoEncoder);
        estimator.qVelFused = Fusion_CalculateSpeed(estimator.qVelEstim, estimator.qVelEncoder);
        
        // Set output values to fusion results
        estimator.qRhoOutput = estimator.qRhoFused;
        estimator.qVelOutput = estimator.qVelFused;
    }
    else if (estimator.useEncoder == 1) // Encoder only
    {
        estimator.qRhoOutput = estimator.qRhoEncoder;
        estimator.qVelOutput = estimator.qVelEncoder;
    }
    else // Sensorless only
    {
        estimator.qRhoOutput = estimator.qRho;
        estimator.qVelOutput = estimator.qVelEstim;
    }
}
```

### 6. Update Control Flow in `pmsm.c`

Modify the `CalculateParkAngle()` function to use the fused angle:

```
// In pmsm.c
void CalculateParkAngle(void)
{
    if (uGF.bits.OpenLoop)
    {
        // Existing open-loop code...
        thetaElectrical = thetaElectricalOpenLoop;
    }
    else 
    {
        // Use the output value from estimator, which is already
        // selected based on the current mode (sensorless/encoder/hybrid)
        thetaElectrical = estimator.qRhoOutput;
        
        // Rest of existing code...
    }
}
```

## Transition Management

### 1. Mode Transitions

Implement smooth transitions between different operating modes:

```
// In fusion.c (pseudocode)
void Fusion_HandleModeTransition(uint16_t new_mode)
{
    // If transitioning to or from hybrid mode
    if (fusion.current_mode != new_mode)
    {
        // Initialize transition parameters
        fusion.transition_counter = 0;
        fusion.transition_target = TRANSITION_STEPS;
        fusion.transition_source_mode = fusion.current_mode;
        fusion.transition_target_mode = new_mode;
        
        // Store current position values
        fusion.transition_start_position = estimator.qRhoOutput;
        
        // Determine end position based on target mode
        if (new_mode == ENCODER_MODE)
            fusion.transition_end_position = estimator.qRhoEncoder;
        else if (new_mode == SENSORLESS_MODE)
            fusion.transition_end_position = estimator.qRho;
        else // HYBRID_MODE
            fusion.transition_end_position = Fusion_CalculatePosition(estimator.qRho, estimator.qRhoEncoder);
        
        // Normalize transition angle difference
        fusion.transition_angle_diff = NormalizeAngle(fusion.transition_end_position - fusion.transition_start_position);
    }
    
    fusion.current_mode = new_mode;
}

// Apply transition during position calculation
int16_t Fusion_GetTransitionPosition(void)
{
    if (fusion.transition_counter < fusion.transition_target)
    {
        // Calculate transition ratio (0.0 to 1.0 in Q15 format)
        int16_t ratio = (int16_t)((int32_t)fusion.transition_counter * 32767 / fusion.transition_target);
        
        // Apply interpolation
        int16_t angle_adjust = (int16_t)((int32_t)fusion.transition_angle_diff * ratio / 32767);
        int16_t position = NormalizeAngle(fusion.transition_start_position + angle_adjust);
        
        // Increment counter
        fusion.transition_counter++;
        return position;
    }
    else
    {
        // Transition complete, return based on current mode
        if (fusion.current_mode == ENCODER_MODE)
            return estimator.qRhoEncoder;
        else if (fusion.current_mode == SENSORLESS_MODE)
            return estimator.qRho;
        else // HYBRID_MODE
            return Fusion_CalculatePosition(estimator.qRho, estimator.qRhoEncoder);
    }
}
```

### 2. Speed Range Transitions

Implement smooth weight adjustments for speed range transitions:

```
// In fusion.c (pseudocode)
void Fusion_SmoothWeightTransition(void)
{
    // Apply slew rate limiting to weight changes
    int16_t weight_diff = fusion.encoder_weight_target - fusion.encoder_weight_current;
    
    if (abs(weight_diff) > WEIGHT_CHANGE_RATE)
    {
        if (weight_diff > 0)
            fusion.encoder_weight_current += WEIGHT_CHANGE_RATE;
        else
            fusion.encoder_weight_current -= WEIGHT_CHANGE_RATE;
    }
    else
    {
        fusion.encoder_weight_current = fusion.encoder_weight_target;
    }
    
    // Similarly for estimator weight
    weight_diff = fusion.estimator_weight_target - fusion.estimator_weight_current;
    // ...
}
```

## Diagnostics and Monitoring

### 1. X2CScope Variables

Add diagnostic variables for X2CScope monitoring:

```
// In fusion.c
// Register variables for X2CScope
void Fusion_RegisterScope(void)
{
    // Register fusion-related variables
    X2CScope_Initialise();
    X2CScope_AddVariable(&fusion.encoder_weight, "EncoderWeight");
    X2CScope_AddVariable(&fusion.estimator_weight, "EstimatorWeight");
    X2CScope_AddVariable(&fusion.position_error, "PositionError");
    X2CScope_AddVariable(&fusion.speed_error, "SpeedError");
    X2CScope_AddVariable(&fusion.encoder_confidence, "EncoderConfidence");
    X2CScope_AddVariable(&fusion.estimator_confidence, "EstimatorConfidence");
    X2CScope_AddVariable(&estimator.qRhoEncoder, "EncoderPosition");
    X2CScope_AddVariable(&estimator.qRho, "EstimatorPosition");
    X2CScope_AddVariable(&estimator.qRhoOutput, "FusedPosition");
}
```

### 2. Error Detection and Recovery

Implement monitoring for inconsistencies and recovery strategies:

```
// In fusion.c (pseudocode)
bool Fusion_DetectFault(void)
{
    // Check for large position discrepancy
    if (abs(fusion.position_error) > POSITION_ERROR_THRESHOLD)
    {
        fusion.fault_counter++;
        if (fusion.fault_counter > FAULT_THRESHOLD)
        {
            // Determine which source to trust
            if (fusion.encoder_confidence > fusion.estimator_confidence)
            {
                // Trust encoder, reset estimator
                EstimResetPosition(estimator.qRhoEncoder);
            }
            else
            {
                // Trust estimator, possibly flag encoder fault
                fusion.encoder_fault = true;
            }
            
            fusion.fault_counter = 0;
            return true;
        }
    }
    else
    {
        // Gradually decrease fault counter
        if (fusion.fault_counter > 0)
            fusion.fault_counter--;
    }
    
    return false;
}
```

## Configuration Parameters

Add new parameters to `userparms.h`:

```
/************************ Sensor Fusion Parameters ****************************/
/* Speed thresholds for weight adjustment (in electrical RPM) */
#define LOW_SPEED_THRESHOLD      (NOMINAL_SPEED_RPM*NOPOLESPAIRS*0.1)  /* 10% of nominal */
#define MID_SPEED_THRESHOLD      (NOMINAL_SPEED_RPM*NOPOLESPAIRS*0.3)  /* 30% of nominal */
#define HIGH_SPEED_THRESHOLD     (NOMINAL_SPEED_RPM*NOPOLESPAIRS*0.9)  /* 90% of nominal */

/* Weight values (Q15 format) */
#define LOW_WEIGHT               3277    /* ~0.1 in Q15 */
#define MID_WEIGHT               16384   /* ~0.5 in Q15 */
#define HIGH_WEIGHT              29491   /* ~0.9 in Q15 */
#define MAX_WEIGHT               32767   /* 1.0 in Q15 */
#define MIN_WEIGHT               1638    /* ~0.05 in Q15 */

/* Confidence parameters */
#define MAX_CONFIDENCE           32767   /* 1.0 in Q15 */
#define MIN_CONFIDENCE           3277    /* ~0.1 in Q15 */
#define CONFIDENCE_INCREMENT     328     /* ~0.01 in Q15 */
#define CONFIDENCE_DECREMENT     1638    /* ~0.05 in Q15 */

/* Transition parameters */
#define TRANSITION_STEPS         100     /* Steps for smooth transition */
#define WEIGHT_CHANGE_RATE       328     /* ~0.01 in Q15 per update */

/* Fault detection */
#define POSITION_ERROR_THRESHOLD 3277    /* ~10% of electrical revolution */
#define FAULT_THRESHOLD          20      /* Consecutive errors before action */

/* Encoder error limits */
#define ENCODER_MAX_ACCEL        Q15(0.05)   /* Maximum plausible acceleration */
#define ERROR_CONFIDENCE_FACTOR  328     /* Confidence reduction per error */

/* Complementary filter coefficient (Q15 format) */
/* Higher = more encoder influence, lower = more estimator influence */
#define COMP_FILTER_COEF_LOW     26214   /* ~0.8 in Q15 */
#define COMP_FILTER_COEF_HIGH    6554    /* ~0.2 in Q15 */
```

## Testing and Validation

### 1. Basic Fusion Algorithm Validation
- Verify proper position fusion at different speeds
- Confirm smooth weighting transitions
- Test angle normalization and wraparound handling

### 2. Dynamic Response Testing
- Evaluate performance during speed changes
- Test response to load transients
- Verify robustness during acceleration/deceleration

### 3. Confidence Metric Validation
- Artificially introduce encoder errors to verify confidence reduction
- Test estimator confidence variation with speed
- Verify proper weight adjustment based on confidence

### 4. Mode Transition Testing
- Test switching between sensorless, encoder, and hybrid modes
- Verify smooth transitions without torque disturbances
- Confirm proper angle interpolation during transitions

### 5. Fault Handling
- Simulate encoder disconnection
- Test estimator disturbances
- Verify automatic recovery from fault conditions

### 6. Performance Measurement
- Compare position error against single-source implementations
- Measure speed estimation accuracy
- Evaluate torque ripple at different operating points

## Implementation Considerations

### Computational Efficiency
- Optimize fusion algorithm for minimal execution time
- Consider fixed-point optimizations for trigonometric operations
- Profile execution time to ensure it fits within control loop timing

### Memory Usage
- Minimize additional RAM requirements
- Use shared buffers where possible
- Consider flash usage for lookup tables if needed

### Numerical Stability
- Ensure proper angle normalization
- Prevent overflow in calculations
- Validate filter stability at all operating points

## Deliverables for Step 2

1. Complete `fusion.c/h` module with configurable algorithms
2. Updated `estim.c/h` with fusion integration
3. Modified `pmsm.c` to use fused position/speed values
4. Extended configuration parameters in `userparms.h`
5. X2CScope configuration for monitoring fusion performance
6. Test results documenting hybrid performance improvement
7. Documentation of tuning parameters and adjustment guides

## Next Steps

After successful implementation of Step 2, we will proceed to:

1. Fine-tune the fusion algorithm parameters based on real-world testing
2. Implement advanced features such as adaptive notch filters
3. Optimize startup and initial position detection
4. Add enhanced fault detection and recovery strategies

## Conclusion

This second step builds upon the basic encoder integration to create a sophisticated hybrid FOC system. By implementing a robust sensor fusion algorithm with adaptive weighting, we can significantly improve control performance across the entire speed range. The complementary filter approach provides a good balance between computational efficiency and performance while laying the groundwork for more advanced fusion techniques in the future.
