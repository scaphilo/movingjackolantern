# Input/Output (I/O) Component Documentation for STM32F429 Discovery Kit

## Introduction

### Scope
This documentation covers the input/output components for the Machine Control PCB, specifically focusing on button and LED interfaces for the STM32F429 Discovery Kit.

### Definitions and Acronyms
- GPIO: General Purpose Input/Output
- PCB: Printed Circuit Board
- HAL: Hardware Abstraction Layer
- PP: Push-Pull (GPIO output mode)
- STM32: STMicroelectronics 32-bit microcontroller series

## Hardware Dependencies

### STM32F429 Discovery Kit Specifics
- Microcontroller: STM32F429ZIT6
- GPIO Ports Used: 
  - GPIOH: Button interface
  - GPIOC, GPIOI, GPIOB: LED interface
- Clock Frequency: Dependent on system configuration
- Power Supply: 3.3V logic level

## Detailed Component Description

### Button Interface Structure

#### Physical Configuration
- Total Buttons: 2
- GPIO Pins:
  - Pin PH4: Downward button (Index 0)
  - Pin PH8: Upward button (Index 1)

#### GPIO Pin Structure
```c
typedef struct _GPIO_PIN {
  GPIO_TypeDef *port;  // GPIO port
  uint16_t      pin;   // Specific pin number
} GPIO_PIN;
```

#### Button Indexing
- Index 0: Downward button (PH4)
- Index 1: Upward button (PH8)

### Button Functions

#### 1. Buttons_Initialize()
- **Purpose**: Initialize button GPIO pins
- **Parameters**: None
- **Return Values**:
  - `0`: Successful initialization
  - `-1`: Initialization failed (not implemented in current version)
- **Implementation Details**:
  - Enables GPIOH clock using `__GPIOH_CLK_ENABLE()`
  - Configures PH4 and PH8 as input pins
  - Sets pull-down resistor configuration
  - Configures low-speed GPIO mode
- **Error Handling**:
  - No explicit error handling implemented
  - Returns 0 in all cases

#### 2. Buttons_Uninitialize()
- **Purpose**: De-initialize button GPIO pins
- **Parameters**: None
- **Return Values**:
  - `0`: Successful de-initialization
  - `-1`: De-initialization failed (not implemented in current version)
- **Implementation Details**:
  - Resets GPIO configuration for PH4 and PH8 using `HAL_GPIO_DeInit()`
- **Error Handling**:
  - No explicit error handling implemented
  - Returns 0 in all cases

#### 3. Button_Downward_Reached()
- **Purpose**: Check the state of the downward button
- **Parameters**: None
- **Return Value**: 
  - `GPIO_PinState`: Current state of the downward button
    - `GPIO_PIN_SET`: Button pressed
    - `GPIO_PIN_RESET`: Button not pressed
- **Implementation**: 
  - Uses `HAL_GPIO_ReadPin()` to read GPIO pin state
- **Thread Safety**: 
  - Not guaranteed, potential race conditions if accessed concurrently

#### 4. Button_Upward_Reached()
- **Purpose**: Check the state of the upward button
- **Parameters**: None
- **Return Value**: 
  - `GPIO_PinState`: Current state of the upward button
    - `GPIO_PIN_SET`: Button pressed
    - `GPIO_PIN_RESET`: Button not pressed
- **Implementation**: 
  - Uses `HAL_GPIO_ReadPin()` to read GPIO pin state
- **Thread Safety**: 
  - Not guaranteed, potential race conditions if accessed concurrently

### LED Interface Structure

#### Physical Configuration
- Total LEDs: 3
- GPIO Ports and Pins:
  - GPIOC, Pin 4: First LED (Index 0)
  - GPIOI, Pin 10: Second LED (Index 1)
  - GPIOB, Pin 8: Third LED (Index 2)

#### GPIO Pin Structure
```c
typedef struct _GPIO_PIN {
  GPIO_TypeDef *port;  // GPIO port
  uint16_t      pin;   // Specific pin number
} GPIO_PIN;
```

#### LED Indexing
- Index 0: PC4 LED
- Index 1: PI10 LED
- Index 2: PB8 LED

#### LED Configuration
- GPIO Mode: Output Push-Pull (`GPIO_MODE_OUTPUT_PP`)
- Pull Configuration: Pull-down
- Speed: Low-speed GPIO
- Active State: High (GPIO_PIN_SET turns LED on)

### LED Functions

#### 1. LED_Initialize()
- **Purpose**: Initialize LED GPIO pins
- **Parameters**: None
- **Return Values**:
  - `0`: Successful initialization
  - `-1`: Initialization failed (not implemented)
- **Implementation Details**:
  - Enables GPIO clocks for GPIOC, GPIOI, and GPIOB using:
    - `__GPIOC_CLK_ENABLE()`
    - `__GPIOI_CLK_ENABLE()`
    - `__GPIOB_CLK_ENABLE()`
  - Configures each LED pin with:
    - Push-pull output mode
    - Pull-down resistor
    - Low-speed GPIO configuration
- **Error Handling**:
  - No explicit error handling
  - Always returns 0

#### 2. LED_Uninitialize()
- **Purpose**: De-initialize LED GPIO pins
- **Parameters**: None
- **Return Values**:
  - `0`: Successful de-initialization
  - `-1`: De-initialization failed (not implemented)
- **Implementation Details**:
  - Resets GPIO configuration for:
    - GPIOC, Pin 4
    - GPIOI, Pin 10
    - GPIOB, Pin 8
- **Error Handling**:
  - No explicit error handling
  - Always returns 0

#### 3. LED_On(uint32_t num)
- **Purpose**: Turn on a specific LED
- **Parameters**: 
  - `num`: LED index (0-2)
- **Return Values**:
  - `0`: Successful LED activation
  - `-1`: LED activation failed (not implemented)
- **Implementation**: 
  - Uses `HAL_GPIO_WritePin()` with `GPIO_PIN_SET`
- **Error Handling**:
  - No bounds checking
  - No error reporting for invalid indices

#### 4. LED_Off(uint32_t num)
- **Purpose**: Turn off a specific LED
- **Parameters**: 
  - `num`: LED index (0-2)
- **Return Values**:
  - `0`: Successful LED deactivation
  - `-1`: LED deactivation failed (not implemented)
- **Implementation**: 
  - Uses `HAL_GPIO_WritePin()` with `GPIO_PIN_RESET`
- **Error Handling**:
  - No bounds checking
  - No error reporting for invalid indices

#### 5. LED_SetOut(uint32_t val)
- **Purpose**: Set multiple LEDs using a bit vector
- **Parameters**: 
  - `val`: Bit vector representing LED states
- **Return Values**:
  - `0`: Successful LED state update
  - `-1`: LED state update failed (not implemented)
- **Bit Manipulation Logic**:
  ```c
  for (n = 0; n < LED_COUNT; n++) {
    if (val & (1 << n)) 
      LED_On(n);
    else                
      LED_Off(n);
  }
  ```
- **Error Handling**:
  - Implicitly handles out-of-range bits by ignoring them
  - No explicit error reporting

#### 6. LED_GetCount()
- **Purpose**: Get the total number of available LEDs
- **Return Value**: 
  - Number of LEDs (3)
- **Implementation**:
  - Returns `LED_COUNT` macro
  - Statically defined based on `LED_PIN[]` array size

## PlantUML Diagrams

### Button State Diagram
```plantuml
@startuml
[*] --> NotPressed
NotPressed --> Pressed : GPIO_PIN_SET
Pressed --> NotPressed : GPIO_PIN_RESET
@enduml
```

### Button Initialization Sequence
```plantuml
@startuml
actor System
participant "GPIO Module" as GPIO
participant "Button Interface" as Button

System -> Button: Buttons_Initialize()
Button -> GPIO: __GPIOH_CLK_ENABLE()
Button -> GPIO: HAL_GPIO_Init(GPIOH)
Button --> System: Return 0
@enduml
```

### LED Initialization Sequence
```plantuml
@startuml
actor System
participant "GPIO Module" as GPIO
participant "LED Interface" as LED

System -> LED: LED_Initialize()
LED -> GPIO: __GPIOC_CLK_ENABLE()
LED -> GPIO: __GPIOI_CLK_ENABLE()
LED -> GPIO: __GPIOB_CLK_ENABLE()
LED -> GPIO: HAL_GPIO_Init(GPIOC)
LED -> GPIO: HAL_GPIO_Init(GPIOI)
LED -> GPIO: HAL_GPIO_Init(GPIOB)
LED --> System: Return 0
@enduml
```

## Design Patterns

### Hardware Abstraction
- Uses STM32 HAL for low-level GPIO interactions
- Provides a consistent, platform-independent interface
- Separates hardware-specific configuration from application logic

### State Management
- Simple binary state management for buttons and LEDs
- Direct GPIO pin state reading and writing
- Minimal overhead design

## Limitations and Recommendations

### Button Interface Limitations
- No debounce mechanism
- No interrupt-based button reading
- Synchronous, blocking button state reading

### LED Interface Limitations
- No thread-safe LED control
- No error checking for LED indices
- Blocking LED state changes
- No PWM or dimming capabilities

### Recommended Improvements
- Implement software debounce for buttons
- Add interrupt-based button event handling
- Create thread-safe button and LED control mechanisms
- Support PWM for LED brightness control
- Implement comprehensive error checking and reporting

## Appendix

### References
- [STM32F4 HAL Documentation](https://www.st.com/resource/en/user_manual/dm00105879-description-of-stm32f4-hal-and-ll-drivers-stmicroelectronics.pdf)
- STM32F429 Discovery Kit Datasheet
- ARM Limited Copyright Notice

### List of Illustrations
1. Button State Diagram
2. Button Initialization Sequence Diagram
3. LED Initialization Sequence Diagram

### Additional Resources
- STM32CubeMX Documentation
- ARM Embedded Development Guidelines
- GPIO Configuration Best Practices