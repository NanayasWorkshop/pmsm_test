# X2CScope Library for dsPIC33 DSC

## Overview
X2CScope is a virtual oscilloscope tool developed by Linz Center of Mechatronics which allows run-time debugging or monitoring of embedded applications in MPLAB X IDE. This tool enables developers to "Watch" or "Plot" any global variable in the embedded application at run-time without halting the CPU.

## Library Structure

### Core Files
- **X2CScope.h**: Main header file that provides the public API
- **libx2cscope-generic-dspic-elf.a**: Compiled library containing implementations

## Key API Functions

```c
void X2CScope_Initialise(uint8_t* buffer, size_t buffer_size);
```
Initializes the X2CScope library with a buffer for data storage.

```c
void X2CScope_Communicate();
```
Handles communication between the target device and the X2CScope client running on the host PC. This function should be called periodically in the main loop.

```c
void X2CScope_Update();
```
Updates the scope data. This function should be called at a fixed rate, typically in a timer or ADC interrupt service routine.

```c
void X2CScope_HookUARTFunctions(
    void (*sendSerialFcnPntr)(uint8_t), 
    uint8_t (*receiveSerialFcnPntr)(), 
    uint8_t (*isReceiveDataAvailableFcnPntr)(), 
    uint8_t (*isSendReadyFcnPntr)());
```
Hooks the X2CScope library to the device's UART functions for serial communication with the host PC.

## Implementation in Application

In the LVMC dsPIC33CK256MP508 application, X2CScope is implemented in the `diagnostics_x2cscope.c` file:

1. A buffer is allocated for X2CScope data:
```c
#define X2C_BUFFER_SIZE 4900
X2C_DATA static uint8_t X2C_BUFFER[X2C_BUFFER_SIZE];
```

2. UART is configured for X2CScope communication:
```c
#define X2C_BAUDRATE_DIVIDER 54  // For 115.7kbaud
```

3. Library is initialized in the `DiagnosticsInit()` function:
```c
void DiagnosticsInit(void)
{
    // Configure UART
    UART1_InterruptReceiveDisable();
    UART1_InterruptReceiveFlagClear();
    UART1_InterruptTransmitDisable();
    UART1_InterruptTransmitFlagClear();
    UART1_Initialize();
    UART1_BaudRateDividerSet(X2C_BAUDRATE_DIVIDER);
    UART1_SpeedModeStandard();
    UART1_ModuleEnable();  
    
    X2CScope_Init();
}
```

4. UART functions are hooked to X2CScope in the `X2CScope_Init()` function:
```c
void X2CScope_Init(void)
{
    X2CScope_HookUARTFunctions(
        X2CScope_sendSerial,
        X2CScope_receiveSerial,
        X2CScope_isReceiveDataAvailable,
        X2CScope_isSendReady);
    X2CScope_Initialise(X2C_BUFFER, sizeof(X2C_BUFFER));
}
```

5. Communication primitives are implemented for X2CScope to use:
```c
static void X2CScope_sendSerial(uint8_t data)
{
    UART1_DataWrite(data);
}

static uint8_t X2CScope_receiveSerial()
{
    return UART1_DataRead();
}

static uint8_t X2CScope_isReceiveDataAvailable()
{
    return UART1_IsReceiveBufferDataReady();
}

static uint8_t X2CScope_isSendReady()
{
    return !UART1_StatusBufferFullTransmitGet();
}
```

6. The library's periodic functions are called in the appropriate places:
```c
void DiagnosticsStepMain(void)
{
    X2CScope_Communicate();  // Called in main loop
}

void DiagnosticsStepIsr(void)
{
    X2CScope_Update();  // Called in interrupt service routine
}
```

## Usage Considerations

- The baud rate is configured using divider values. The application includes a helpful comment table showing the relationship between divider values and resulting baud rates.
- Buffer size affects the amount of data that can be captured and the performance impact. The example uses a 4900-byte buffer.
- X2CScope must be integrated with UART or other communication interfaces to function properly.
- For optimal performance, X2CScope_Update() should be called at a fixed rate in an interrupt routine, while X2CScope_Communicate() should be called in the main loop.
