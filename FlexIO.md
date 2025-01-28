# Teensy 4.1 FlexIO 

## How to use the guide
FlexIO is a very complex subject. The basics are simple, but the details are complex. This guide will help you understand how FlexIO works and how to use it effectively. The first section Basic Concepts and Overview will provide you with a basic understanding of FlexIO and how it works. The second section is composed of a series of examples that will slowly build your knowledge of FlexIO. The third section is a detailed look at FlexIO's architecture, components, and capabilities. 
# Basic Concepts and Overview


## What is FlexIO?
FlexIO is a highly versatile peripheral on the Teensy 4.1 that allows you to create custom communication interfaces or emulate standard protocols. Think of it as a configurable digital interface builder that can adapt to almost any digital communication need.

### Block Diagram
![FlexIO Block Diagram](images/FlexIO_Block_Diagram.png)

## Core Components

FlexIO consists of four main building blocks:

1. Shifters: These 32-bit registers handle data movement, converting between parallel and serial data formats. They're essential for both sending and receiving data.

2. Timers: 16-bit precision timers that generate clock signals and control the shifters. They determine when and how data moves through the system.

3. Pin Multiplexer: Allows flexible routing of up to 32 digital pins as inputs or outputs. You can assign pins to different functions based on your needs.

4. DMA Support: Enables direct memory access for efficient data transfer without CPU intervention.

## Common Uses

FlexIO excels at tasks like:

- Driving LED strips (like WS2812B)
- Creating additional UART, SPI, or I2C interfaces
- Implementing camera and display interfaces
- Building custom parallel bus protocols
- Handling specialized timing-sensitive protocols

## Operating Modes

FlexIO can function as either:
- Master: Controlling the communication timing
- Slave: Responding to external timing signals

The peripheral can also synchronize with other Teensy systems through its trigger system, allowing coordinated operations with timers, pins, or other peripherals.

## FlexIO PINs
The Teensy 4.1 provides three FlexIO modules with different pin capacities:
- FlexIO1: 16 pins available
- FlexIO2: 32 pins available
- FlexIO3: 32 pins available

Each pin can be individually configured as:
- Input with optional pull-up/pull-down
- Push-pull output
- Open-drain output
- Bidirectional with configurable direction control

Pin assignments are flexible within each module, allowing you to map shifter inputs/outputs to any available FlexIO pin.



## FlexIO Shifters
Each FlexIO Module has 8 32-bit shifters that can:
- Buffer and shift data in either direction (MSB or LSB first)
- Operate in transmit or receive mode
- Support variable width data (1-32 bits)
- Generate or check parity
- Provide status flags for buffer states
- Trigger DMA requests

The shifter's timing is controlled by an associated timer, which determines when data moves in or out.

## FlexIO Timers
Timers control the timing and synchronization of FlexIO operations. Each 16-bit timer provides:
- Configurable clock source selection
- Programmable divider for precise frequency control
- Multiple trigger inputs and outputs
- Support for different operating modes:
  - Pulse generation (for PWM)
  - Shift clock generation
  - 8-16-32 bit counter modes
  
Timers can be chained together for more complex timing sequences and can trigger on external events or other FlexIO components.


# Code Example (WS2812B LED Driver)
WS2812B LEDs are popular RGB LEDs that use a single wire for control. Each LED requires precisely timed signals to set its color. While this is typically challenging for most microcontrollers, FlexIO makes it straightforward by handling the precise timing requirements in hardware.

Think of FlexIO as a mini-factory: the shifter is the assembly line moving your color data, the timer is the foreman ensuring perfect timing, and the pin is the delivery truck sending signals to the LED.

### Goal
We need to generate a precise timing signal for the WS2812B protocol:
- A '1' bit needs 800ns high followed by 450ns low (1250ns total)
- A '0' bit needs 400ns high followed by 850ns low (1250ns total)
- Each LED requires 24 bits (8 bits each for Green, Red, Blue)
- After each LED's data, we need 50+ microseconds of low signal (reset)

### Components Needed
- PIN: One FlexIO output pin to connect to the LED strip's data line
  - Must be capable of fast switching (>800kHz)
  - Will be configured as push-pull output

- SHIFTER: Handles the 24-bit color data per LED
  - Configured for 24-bit operation
  - Shifts MSB first (WS2812B protocol requirement)
  - Transmit mode with timer-controlled shifting

- TIMER: Creates the precise timing for bits
  - Generates 800kHz base frequency (1250ns period)
  - Controls shifter to create correct high/low timing
  - Triggers new bit transmission

### Setup Sequence
1. Configure shifter:
   - Set 24-bit mode for full RGB color
   - Enable MSB-first operation
   - Configure transmit mode
   - Set up timer trigger for shifting

2. Set timer parameters:
   - Configure 800kHz operation (1250ns period)
   - Set up compare values for 1/0 bit timing
   - Enable shifter triggering
   - Configure output enable control

3. Set up pin routing:
   - Select appropriate FlexIO pin
   - Configure as push-pull output
   - Map shifter output to pin
   - Enable pin output

### How They Work Together
1. Timer starts its cycle:
   - Begins counting at configured frequency
   - Generates shifter trigger at proper intervals

2. Shifter responds to timer:
   - Reads next bit from its buffer
   - Based on 1/0 value, outputs appropriate timing
   - Continues until all 24 bits are sent

3. Pin delivers the signal:
   - Converts shifter output to voltage levels
   - Maintains precise timing from timer
   - Provides reset period between LED updates

This coordination creates the exact timing the WS2812B needs:
- Timer ensures overall bit timing of 1250ns
- Shifter controls whether it's a 1 (800ns high) or 0 (400ns high)
- Pin delivers clean, properly timed signals to the LED

### Code Example
```cpp
#include <FlexIO_t4.h>

class WS2812B_FlexIO {
private:
    FlexIO_t4 *flexIO;
    const uint8_t pin;
    const uint8_t shifterIndex;
    const uint8_t timerIndex;
    
public:
    WS2812B_FlexIO(FlexIO_t4 *flex, uint8_t outputPin, uint8_t shifter = 0, uint8_t timer = 0)
        : flexIO(flex), pin(outputPin), shifterIndex(shifter), timerIndex(timer) {}
    
    void begin() {
        // 1. Configure the FlexIO pin
        flexIO->setPin(pin, OUTPUT);
        
        // 2. Configure the shifter
        flexIO->setShifterConfig(shifterIndex,
            FLEXIO_SHIFTER_TIMER_SEL(timerIndex) |     // Use our timer
            FLEXIO_SHIFTER_TIMER_POLARITY(1) |         // Shift on timer enable
            FLEXIO_SHIFTER_PIN_CONFIG(1) |             // Pin output
            FLEXIO_SHIFTER_PIN_POLARITY(0) |           // Active high
            FLEXIO_SHIFTER_PIN_SEL(pin) |             // Which pin to use
            FLEXIO_SHIFTER_MODE(2));                   // Transmit mode
        
        // 3. Configure the timer for 800kHz operation
        // Timer compare = bus_clock / (2 * 800kHz) - 1
        uint32_t compare = (F_BUS_ACTUAL / 1600000) - 1;
        
        flexIO->setTimerConfig(timerIndex,
            FLEXIO_TIMER_TRIGGER_SEL(1) |              // Trigger on shifter status flag
            FLEXIO_TIMER_TRIGGER_POLARITY(0) |         // Trigger active high
            FLEXIO_TIMER_TRIGGER_SOURCE(1) |           // Internal trigger
            FLEXIO_TIMER_PIN_CONFIG(3) |               // Timer pin output enable
            FLEXIO_TIMER_MODE(3) |                     // Dual 8-bit counters
            FLEXIO_TIMER_COMPARE(compare));            // Set frequency
        
        // 4. Enable FlexIO
        flexIO->begin();
    }
    
    void sendPixel(uint8_t r, uint8_t g, uint8_t b) {
        // WS2812B expects GRB order
        uint32_t color = (g << 16) | (r << 8) | b;
        
        // Wait until shifter buffer is empty
        while (!(flexIO->getShifterStatusFlags() & (1 << shifterIndex)));
        
        // Send 24-bit color data
        flexIO->setShifterBuffer(shifterIndex, color, 24);
    }
    
    void show() {
        // Send reset pulse (>50us low)
        delayMicroseconds(50);
    }
};

// Example usage
void setup() {
    FlexIO_t4 flexIO1;
    WS2812B_FlexIO leds(&flexIO1, 2);  // Using FlexIO pin 2
    
    leds.begin();
    
    // Set first LED to red
    leds.sendPixel(255, 0, 0);
    leds.show();
}
```

### Understanding the Configuration

#### Shifter Configuration (setShifterConfig)
The shifter configuration sets up how data moves through FlexIO:
```cpp
FLEXIO_SHIFTER_TIMER_SEL(timerIndex)     // Which timer controls this shifter
FLEXIO_SHIFTER_TIMER_POLARITY(1)         // Shift on timer's rising edge
FLEXIO_SHIFTER_PIN_CONFIG(1)             // Configure pin as output
FLEXIO_SHIFTER_PIN_POLARITY(0)           // Output is active high
FLEXIO_SHIFTER_PIN_SEL(pin)              // Which pin to use
FLEXIO_SHIFTER_MODE(2)                   // Mode 2 = transmit
```

#### Timer Configuration (setTimerConfig)
The timer creates the precise timing needed for WS2812B:
```cpp
FLEXIO_TIMER_TRIGGER_SEL(1)              // Use shifter status as trigger
FLEXIO_TIMER_TRIGGER_POLARITY(0)         // Trigger on rising edge
FLEXIO_TIMER_TRIGGER_SOURCE(1)           // Use internal triggering
FLEXIO_TIMER_PIN_CONFIG(3)               // Enable output on timer
FLEXIO_TIMER_MODE(3)                     // Dual 8-bit counter mode
FLEXIO_TIMER_COMPARE(compare)            // Set 800kHz frequency
```

#### Shifter Buffer (setShifterBuffer)
Controls the actual data transmission:
- Takes 24-bit color value (GRB format for WS2812B)
- Waits for shifter to be ready using status flags
- Automatically shifts out bits at timer-controlled rate

#### Pin Configuration (setPin)
Simple but critical setup:
- Configures physical pin for output
- Sets up proper drive strength for high-speed operation
- Connects pin to FlexIO shifter output

This configuration creates a system where:
1. Timer generates steady 800kHz pulses
2. Shifter converts 24-bit color data into WS2812B protocol
3. Pin outputs the precisely timed signal
4. Status flags manage data flow

This example demonstrates:
1. Setting up FlexIO pin, shifter, and timer configurations
2. Creating proper timing for WS2812B protocol
3. Sending RGB color data to the LED
4. Handling the reset timing between updates

The code uses hardware features to maintain precise timing:
- Timer handles the 800kHz base frequency
- Shifter manages the 24-bit color transmission
- Pin configuration ensures clean signal output


# Deep Dive into FlexIO
The following sections detail FlexIO's components and their capabilities. While the information may seem extensive, don't feel overwhelmed - you don't need to memorize everything. These details will become clearer through practical examples, and you can always refer back here when implementing specific features. Think of this as a reference to consult as you explore FlexIO's capabilities.

## Pins
The Teensy 4.1 features multiple FlexIO modules, each providing a set of highly configurable digital pins. FlexIO1 offers 16 pins, while FlexIO2 and FlexIO3 each provide 32 pins. These pins can be configured independently or grouped together for parallel operations.

### Pin Capabilities
- Digital Output: Drive signals with configurable strength and slew rate control
- Digital Input: Sample external signals with programmable thresholds
- Parallel Bus: Multiple pins can be grouped for 8/16/32-bit parallel interfaces
- Trigger Input: Pins can trigger FlexIO operations from external events

### Pin Allocation
- FlexIO1: 16 pins, ideal for simple protocols and single-channel operations
- FlexIO2/3: 32 pins each, suitable for wide parallel buses or multiple simultaneous protocols
- Pin Sharing: Some pins overlap with other peripherals, requiring careful allocation

### Signal Routing
- Internal Connection: Pins can connect to any shifter within their FlexIO module
- Cross-triggering: Pins from one FlexIO can trigger operations in another. For example a sampling pin can also trigger a timer.
- Peripheral Integration: Some pins can interact directly with other Teensy peripherals


## Shifters
Shifters form the core data handling components of FlexIO, each providing a versatile 32-bit register that can be configured for various serial and parallel protocols. They handle data movement in both directions, can operate multiple bits in parallel across different pins, and support simultaneous operation of multiple shifters. Through timer and pin configurations, they can be precisely controlled to implement both standard and custom communication protocols.

### Pins
- Output: Direct digital output mode for clean, fast signals with configurable drive strength
- Open-drain: Pulls pin low when active, floating when inactive. Essential for shared buses like I2C where multiple devices drive the same line
- Bidirectional: Dynamically switchable between input and output during operation, useful for half-duplex protocols
- Input: High impedance input mode for sampling external signals with optional pull-up/down
- Active high: Traditional logic where HIGH (3.3V) represents 1 and LOW (0V) represents 0
- Active low: Inverted logic where LOW represents 1 and HIGH represents 0, common in protocols like I2C clock stretching

### Shifter Modes
- Transmit: Outputs serial data with precise timing control. Data shifts out MSB or LSB first with programmable width and rate
- Receive: Samples incoming data on configurable clock edges with optional start/stop bit detection and error checking
- Match: Compares incoming data against a predefined pattern, triggering when matched. Useful for address detection
- Match continuous: Like Match mode but automatically resets after each match, enabling continuous monitoring of data stream
- Logic: Performs real-time boolean operations (AND, OR, XOR) between multiple inputs for custom protocol handling
- State: Implements basic state machines by changing behavior based on input conditions and internal state
- Match store: Captures and stores data when a specific pattern is detected, useful for packet filtering

### Timer Control
- Timer select: Associates a specific timer with the shifter for synchronization. Each shifter can be driven by any timer in the same FlexIO module
- Timer polarity: Controls when the shifter operates relative to the timer. Can shift on rising edge for standard protocols, falling edge for inverted timing, or both edges for double data rate

### Shifter Buffers
Shifter buffers are the data interface for FlexIO shifters - they're essentially where you put data you want to transmit or where you read data that's been received. Think of them as the "mailbox" for each shifter.

Each shifter has several buffer registers that handle data in different ways:

The standard buffer (SHIFTBUF) is your basic interface - data goes in and out normally
The bit-swapped buffer (SHIFTBUFBIS) lets you reverse the bit order (useful when you need MSB first instead of LSB first)
The byte-swapped buffer (SHIFTBUFBYS) swaps the order of bytes in your data
The bit-byte-swapped buffer (SHIFTBUFBBS) does both bit and byte swapping
When transmitting data, you write to these buffers and the shifter takes care of sending it out according to how it's configured. When receiving data, the shifter collects the incoming bits and places them in the buffer for you to read.

The shifter has a status flag that tells you when it's ready for more data (buffer empty) or when new data has arrived (buffer full), which helps you manage the flow of data.

### DMA Support
FlexIO1 and FlexIO2 support DMA operations, while FlexIO3 does not have DMA capability. The DMA functionality is structured as follows:

- Only shifters 0-3 can independently trigger DMA transfers
- Additional shifters (4-7) can be chained with shifters 0-3 to participate in DMA operations
- When chained, the primary shifter (0-3) controls the DMA trigger for the chain

This architecture allows for flexible data movement while maintaining efficient DMA usage. For example, you could:
- Use shifter 0 alone with DMA for simple transfers
- Chain shifter 4 with shifter 0 for wider data transfers using DMA
- Create multiple independent DMA channels using different primary shifters (0-3)

## Flex IO Timers
Each FlexIO Modulue has 8 16-bit timers that control the timing and synchronization of FlexIO operations. Each timer provides:
- Configurable clock source selection
- Programmable divider for precise frequency control
- Multiple trigger inputs and outputs
- Support for different operating modes:
  - Pulse generation (for PWM)
  - Shift clock generation
  - 8-16-32 bit counter modes
  
Timers can be chained together for more complex timing sequences and can trigger on external events or other FlexIO components.

## Clock Sources
- Internal Clock: System clock with programmable divider for precise frequency control
- External Pin: Use external signal as clock source for synchronization with other devices
- Other Timer: Chain from another timer's output for complex timing sequences
- Shifter Status: Trigger from shifter events for data-dependent timing

## Operating Modes
- Dual 8-bit: Two independent 8-bit counters for separate timing control
- Single 16-bit: Higher precision timing for standard protocols
- Concatenated 32-bit: Extended range for very slow or precise timing needs
- PWM Generation: Variable duty cycle output for motor control or dimming

## Trigger Control
- Input Triggers: Start, stop, or reset timer from pins, shifters, or other timers
- Output Triggers: Generate events to control shifters or cascade to other timers
- Edge Control: Select rising, falling, or both edges for trigger detection
- Disable Condition: Programmable conditions to stop timer operation

## Timer Chaining
- Series Operation: Connect timers for multi-stage timing sequences
- Parallel Timing: Multiple independent timers for complex protocols
- Trigger Sharing: Timers can share triggers for synchronized operation

# Details

## Pins
There are both physical pins and FlexIO pins.FXIO_D (or FXIO_Di where i is a number) refers to the FlexIO Data pins. These are the pins that can be used by the FlexIO module for data input or output. Each FlexIO module has a set of these data pins that can be configured for various purposes:

When used as inputs, they can receive data from external devices. When used as outputs, they can send data to external devices. The direction (input or output) is determined by the configuration of the shifter and timer settings.

The "i" in FXIO_Di represents the pin number (0-31 for FlexIO2/3, or 0-15 for FlexIO1). For example:
```
FXIO_D0 is FlexIO Data pin 0
FXIO_D1 is FlexIO Data pin 1
```
And so on...

At the hardware level, each physical pin must be set to FlexIO mode through the IOMUXC (Input/Output MUX Controller) which connects the GPIO pins to actual micrcontroller pins. Teensy 4.1, 4, and Micromod each have different physical pins connected to the FLexIO pins (FXIO) as show by the tables below.
### Pin Maps

#### Teensy 4.0 FlexIO Pin Mapping

The Teensy 4.0 has three FlexIO modules (1-3) that can be used for various communication protocols. Each FlexIO pin is mapped to a specific Teensy pin. The mapping below shows which Teensy pins correspond to which FlexIO module and pin:

| FlexIO Module | FXIO Pin | Teensy Pin | Additional Functions |
|--------------|------------|------------|-------------------|
| 1 | 4 | 2 | PWM, CAN1 TX |
| 1 | 5 | 3 | PWM, CAN1 RX |
| 1 | 6 | 4 | PWM |
| 1 | 7 | 5 | PWM |
| 1 | 8 | 6 | PWM |
| 1 | 17 | 7 | PWM, RX3 |
| 2 | 16 | 8 | PWM, TX3 |
| 2 | 17 | 9 | PWM |
| 2 | 18 | 10 | PWM, CS |
| 2 | 19 | 11 | PWM, MOSI |
| 2 | 20 | 12 | PWM, MISO |
| 2 | 21 | 13 | PWM, SCK |
| 3 | 4 | 14 | PWM, A0 |
| 3 | 5 | 15 | PWM, A1 |
| 3 | 6 | 16 | PWM, A2 |
| 3 | 7 | 17 | PWM, A3 |

#### Teensy 4.1 FlexIO Pin Mapping

The Teensy 4.1 expands on the 4.0's capabilities with additional pins and enhanced flexibility. Here's the mapping of FlexIO pins to Teensy 4.1 pins:

| FlexIO Module | FXIO Pin | Teensy Pin | Additional Functions |
|--------------|------------|------------|-------------------|
| 1 | 4 | 2 | PWM, CAN1 TX |
| 1 | 5 | 3 | PWM, CAN1 RX |
| 1 | 6 | 4 | PWM |
| 1 | 7 | 5 | PWM |
| 1 | 8 | 6 | PWM |
| 1 | 17 | 7 | PWM, RX3 |
| 2 | 16 | 8 | PWM, TX3 |
| 2 | 17 | 9 | PWM |
| 2 | 18 | 10 | PWM, CS |
| 2 | 19 | 11 | PWM, MOSI |
| 2 | 20 | 12 | PWM, MISO |
| 2 | 21 | 13 | PWM, SCK |
| 3 | 4 | 14 | PWM, A0 |
| 3 | 5 | 15 | PWM, A1 |
| 3 | 6 | 16 | PWM, A2 |
| 3 | 7 | 17 | PWM, A3 |
| 3 | 8 | 18 | PWM, A4/SDA |
| 3 | 9 | 19 | PWM, A5/SCL |
| 3 | 10 | 20 | PWM, A6 |
| 3 | 11 | 21 | PWM, A7 |
| 3 | 12 | 22 | PWM, A8 |
| 3 | 13 | 23 | PWM, A9 |
| 2 | 22 | 24 | PWM, A10 |
| 2 | 23 | 25 | PWM, A11 |
| 2 | 24 | 26 | PWM, A12 |
| 2 | 25 | 27 | PWM, A13 |
| 2 | 26 | 28 | PWM |
| 2 | 27 | 29 | PWM |
| 1 | 18 | 30 | PWM |
| 1 | 19 | 31 | PWM |
| 1 | 20 | 32 | PWM |
| 1 | 21 | 33 | PWM |

#### Teensy MicroMod FlexIO Pin Mapping

The Teensy MicroMod has a different pin mapping due to its form factor and design. Here's the mapping of FlexIO pins to MicroMod pins:

| FlexIO Module | FXIO Pin | MicroMod Pin | Additional Functions |
|--------------|------------|------------|-------------------|
| 1 | 4 | D0 | PWM |
| 1 | 5 | D1 | PWM |
| 1 | 6 | D2 | PWM |
| 1 | 7 | D3 | PWM |
| 2 | 16 | D4 | PWM |
| 2 | 17 | D5 | PWM |
| 2 | 18 | D6 | PWM |
| 2 | 19 | D7 | PWM |
| 2 | 20 | D8 | PWM |
| 2 | 21 | D9 | PWM |
| 3 | 4 | A0 | PWM |
| 3 | 5 | A1 | PWM |
| 3 | 6 | A2 | PWM |
| 3 | 7 | A3 | PWM |
| 3 | 8 | A4 | PWM |
| 3 | 9 | A5 | PWM |

Note: This mapping is specific to the Teensy MicroMod form factor. The pin names (D0-D9, A0-A5) reflect the MicroMod pin naming convention rather than the traditional Teensy pin numbers.

Notes:
- Each FlexIO pin can be configured for various functions including PWM, UART, SPI, or I2C
- Some pins share functionality with other peripherals as noted in the Additional Functions column
- When using FlexIO, ensure the pin is not being used by another peripheral

## Shifters
Each shifter is controlled by two registers:
- SHIFTCTL (Shifter Control Register): Controls basic operation mode, pin selection, and timer selection
- SHIFTCFG (Shifter Configuration Register): Sets up additional parameters like stop bits, input source, and data width
### SHIFTCTL (Shifter Control Register)
Controls the basic operation of the shifter:
| Field | Bits | Description |
|-------|------|-------------|
| **SMOD** | [2:0] | Shifter Mode:<br>0: Disabled<br>1: Receive mode<br>2: Transmit mode<br>3: Reserved<br>4: Match Store mode<br>5: Match Continuous mode<br>6: State mode<br>7: Logic mode |
| **Reserved** | [6:3] | Reserved |
| **PINPOL** | [7] | Pin Polarity:<br>0: Pin is active high<br>1: Pin is active low |
| **PINSEL** | [12:8] | Pin Select:<br>• Selects which FlexIO pin to use (0-31)<br>• FlexIO1: Only bits [11:8] supported (4 bits), bit 12 not usable. Can only select FXIO_D0 through FXIO_D15<br>• FlexIO2: All bits [12:8] supported (5 bits). Can select FXIO_D0 through FXIO_D31<br>• FlexIO3: All bits [12:8] supported (5 bits). Can select FXIO_D0 through FXIO_D31<br>Note: For output pins (PINCFG=1), selection takes effect immediately when register is written |
| **Reserved** | [15:13] | Reserved |
| **PINCFG** | [17:16] | Pin Configuration:<br>0: Pin disable<br>1: Pin open drain<br>2: Pin bidirectional<br>3: Pin output |
| **Reserved** | [21:18] | Reserved |
| **TIMPOL** | [23] | Timer Polarity:<br>0: Shift on posedge of shift clock<br>1: Shift on negedge of shift clock |
| **TIMSEL** | [26:24] | Timer Select:<br>Selects which timer to use for controlling the logic/shift register and generating the Shift clock |
| **Reserved** | [31:27] | Reserved |

### SHIFTCFG (Shifter Configuration Register)
Configures additional shifter parameters:
| Field | Bits | Description |
|-------|------|-------------|
| **START** | [1:0] | Shifter Start bit:<br>• For SMOD = Transmit: Allows automatic start bit insertion if timer enabled<br>• For SMOD = Receive/Match Store: Allows automatic start bit checking if timer enabled<br>• For SMOD = State: Used to disable state outputs<br>• For SMOD = Logic: Used to mask logic pin inputs<br><br>Values:<br>0: Start bit disabled, loads data on enable<br>1: Start bit disabled, loads data on first shift<br>2: Outputs start bit 0, error if not 0<br>3: Outputs start bit 1, error if not 1 |
| **Reserved** | [3:2] | Reserved |
| **STOP** | [5:4] | Shifter Stop bit:<br>• For SMOD = Transmit: Allows automatic stop bit insertion if timer enabled<br>• For SMOD = Receive/Match Store: Allows automatic stop bit checking if timer enabled<br>• For SMOD = State: Used to disable state outputs<br>• For SMOD = Logic: Used to mask logic pin inputs<br><br>Values:<br>0: Stop bit disabled<br>1: Reserved<br>2: Outputs stop bit 0, error if not 0<br>3: Outputs stop bit 1, error if not 1 |
| **Reserved** | [7:6] | Reserved |
| **INSRC** | [8] | Input Source:<br>0: Pin<br>1: Shifter N+1 Output (not supported for last shifter) |
| **Reserved** | [15:9] | Reserved |
| **PWIDTH** | [20:16] | Parallel Width:<br>Bits shifted per clock:<br>• 0: 1-bit shift<br>• 1-3: 4-bit shift<br>• 4-7: 8-bit shift<br>• 8-15: 16-bit shift<br><br>For parallel-capable shifters:<br>• Transmit: SHIFTER0, SHIFTER4, etc.<br>• Receive: SHIFTER3, SHIFTER7, etc.<br>• Pin selection: FXIO_D[PINSEL+PWIDTH]:FXIO_D[PINSEL]<br>• Non-parallel shifters only support parallel when INSRC=1<br>• For State mode: Used to disable state outputs<br><br>Note: Not supported in all instances |
| **Reserved** | [31:21] | Reserved |

### SHIFTBUF
The Shifter Buffer N Register (SHIFTBUF) is a 32-bit register that serves as the primary data interface for the shifter.

| Field | Bits | Description |
|-------|------|-------------|
| **SHIFTBUF** | [31:0] | Shifter Buffer:<br>• Read: Returns current value in shifter buffer<br>• Write: Loads new data for transmission<br>• Transmit: Writing clears shifter status flag<br>• Receive: Reading clears shifter status flag<br>• Data alignment depends on configured shift direction and width |

### SHIFTBUFBIS
The Shifter Buffer N Bit Swapped Register (SHIFTBUFBIS) provides the same 32-bit data interface as SHIFTBUF but with reversed bit ordering.

| Field | Bits | Description |
|-------|------|-------------|
| **SHIFTBUFBIS** | [31:0] | Shifter Buffer Bit Swapped:<br>• Same as SHIFTBUF but bits swapped within each byte<br>• For 8-bit data: bit 7 becomes 0, 6 becomes 1, etc.<br>• Useful for changing between MSB-first and LSB-first transmission<br>• Reading/writing this register has the same status flag effects as SHIFTBUF |

### SHIFTBUFBYS
The Shifter Buffer N Byte Swapped Register (SHIFTBUFBYS) provides byte-swapped access to the shifter buffer.

| Field | Bits | Description |
|-------|------|-------------|
| **SHIFTBUFBYS** | [31:0] | Shifter Buffer Byte Swapped:<br>• Same as SHIFTBUF but bytes swapped within the 32-bit word<br>• Byte 0 becomes 3, 1 becomes 2, etc.<br>• Useful for endianness conversion<br>• Reading/writing this register has the same status flag effects as SHIFTBUF |

### SHIFTBUFBBS
The Shifter Buffer N Bit Byte Swapped Register (SHIFTBUFBBS) combines both bit and byte swapping.

| Field | Bits | Description |
|-------|------|-------------|
| **SHIFTBUFBBS** | [31:0] | Shifter Buffer Bit and Byte Swapped:<br>• Combines the effects of both SHIFTBUFBIS and SHIFTBUFBYS<br>• Bits are swapped within each byte AND bytes are swapped within the word<br>• Useful when both bit and byte order need to be reversed<br>• Reading/writing this register has the same status flag effects as SHIFTBUF |

## Timer Registers
There are 8 timers in each FlexIO module. Each timer has two main configuration registers: TIMCTL (Timer Control Register) and TIMCFG (Timer Configuration Register).

### Timer Control Register (TIMCTL)
Controls the basic operation of the timer:
| Field | Bits | Description |
|-------|------|-------------|
| **TIMOD** | [1:0] | Timer Mode:<br>0: Timer Disabled<br>1: Dual 8-bit counters baud mode. The lower 8-bits of the counter and compare register are used to configure the baud rate of the timer shift clock, and the upper 8-bits are used to configure the shifter bit count.<br>2: Dual 8-bit counters PWM high mode. The lower 8-bits of the counter and compare register are used to configure the high period of the timer shift clock, and the upper 8-bits are used to configure the low period of the timer shift clock. The shifter bit count is configured using another timer or external signal.<br>3: Single 16-bit counter mode. The full 16-bits of the counter and compare register are used to configure either the baud rate of the shift clock or the shifter bit count. |
| **Reserved** | [6:2] | Reserved |
| **PINPOL** | [7] | Pin Polarity:<br>For pins configured as an output (PINCFG=11), this field takes effect when the register is written.<br>0: Pin is active high<br>1: Pin is active low |
| **PINSEL** | [12:8] | Pin Select:<br>Selects which FlexIO pin to use (0-31)<br>FlexIO1: Only bits [11:8] are supported (4 bits), bit 12 is not usable. Can only select FXIO_D0 through FXIO_D15<br>FlexIO2: All bits [12:8] are supported (5 bits). Can select FXIO_D0 through FXIO_D31<br>FlexIO3: All bits [12:8] are supported (5 bits). Can select FXIO_D0 through FXIO_D31 |
| **Reserved** | [15:13] | Reserved |
| **PINCFG** | [17:16] | Pin Configuration:<br>0: Pin disable<br>1: Pin open drain<br>2: Pin bidirectional<br>3: Pin output |
| **Reserved** | [21:18] | Reserved |
| **TRGSRC** | [22] | Trigger Source:<br>0: External trigger<br>1: Internal trigger |
| **TRGPOL** | [23] | Trigger Polarity:<br>0: Active high<br>1: Active low |
| **TRGSEL** | [29:24] | Trigger Select:<br>Selects which trigger input the timer uses<br>When TRGSRC = 0 (External trigger):<br>N = External trigger N input<br>When TRGSRC = 1 (Internal trigger), value represents:<br>Pin trigger: 2×N (N = pin number 0-15)<br>Shifter trigger: (4×N)+1 (N = shifter number 0-7)<br>Timer trigger: (4×N)+3 (N = timer number 0-7)<br>Example values for internal triggers (TRGSRC = 1):<br>0: Pin 0 input<br>1: Shifter 0 status flag<br>2: Pin 1 input<br>3: Timer 0 trigger<br>4: Pin 2 input<br>5: Shifter 1 status flag<br>(Pattern continues up to Pin 15, Shifter 7, Timer 7) |
| **Reserved** | [31:30] | Reserved |

### Timer Configuration Register (TIMCFG)
Controls the detailed timing behavior:
| Field | Bits | Description |
|-------|------|-------------|
|**Reserved** | [0] | Reserved |
|**TSTART** | [1] | Timer Start Bit:<br>Controls automatic start bit generation. When start bit is enabled, configured shifters outputs the contents of the start bit when the timer is enabled and the timer counter reloads from the compare register on the first rising edge of the shift clock. |
|**Reserved** | [3:2] | Reserved |
|**TSTOP** | [5:4] | Timer Stop Bit:<br>Controls automatic stop bit generation. The stop bit can be added on a timer compare (between each word) or on a timer disable. When stop bit is enabled, configured shifters output the contents of the stop bit when the timer is disabled. When stop bit is enabled on timer disable, the timer remains disabled until the next rising edge of the shift clock. If configured for both timer compare and timer disable, only one stop bit is inserted on timer disable.<br>00b - 0 Stop bit disabled<br>01b - 1 Stop bit is enabled on timer compare<br>10b - 2 Stop bit is enabled on timer disable<br>11b - 3 Stop bit is enabled on timer compare and timer disable |
|**Reserved** | [7:6] | Reserved |
|**TIMENA** | [10:8] | Timer Enable:<br>Configures the condition that causes the Timer to be enabled and start decrementing.<br><br>0 Timer always enabled<br>1 Timer enabled on Timer N-1 enable<br>2 Timer enabled on Trigger high<br>3 Timer enabled on Trigger high and Pin high<br>4 Timer enabled on Pin rising edge<br>5 Timer enabled on Pin rising edge and Trigger high<br>6 Timer enabled on Trigger rising edge<br>7 Timer enabled on Trigger rising or falling edge |
|**Reserved** | [11] | Reserved |
|**TIMDIS** | [14:12] | Timer Disable:<br>Configures the condition that causes the Timer to be disabled and stop decrementing. <br> 0 Timer never disabled <br>1 Timer disabled on Timer N-1 disable <br>2  Timer disabled on Timer compare (upper 8-bits match and decrement) <br> 3 Timer disabled on Timer compare (upper 8-bits match and decrement) and Trigger Low<br> 4 Timer disabled on Pin rising or falling edge<br> 5 Timer disabled on Pin rising or falling edge provided Trigger is high<br> 6  Timer disabled on Trigger falling edge <br> 7 Reserved |
|**Reserved** | [15] | Reserved |
|**TIMRST** | [18:16] | Timer Reset:<br>Configures the condition that causes the timer counter (and optionally the timer output) to be reset. In 8bit counter mode, the timer reset only resets the lower 8-bits that configure the baud rate. In all other modes, the timer reset resets the full 16-bits of the counter.<br> <br>0 Timer never reset<br>1 Reserved<br>2 Timer reset on Timer Pin equal to Timer Output<br>3 Timer reset on Timer Trigger equal to Timer Output<br>4 Timer reset on Timer Pin rising edge<br>5 Reserved<br>6 Timer reset on Trigger rising edge<br>7 Timer reset on Trigger rising or falling edge |
|**Reserved** | [19] | Reserved |
|**TIMDEC** | [12:20] | Timer Decrement:<br>Configures the source of the Timer decrement and the source of the Shift clock.<br><br>0 Decrement counter on FlexIO clock, Shift clock equals Timer output.<br>1 Decrement counter on Trigger input (both edges), Shift clock equals Timer output.<br>2 Decrement counter on Pin input (both edges), Shift clock equals Pin input.<br>3 Decrement counter on Trigger input (both edges), Shift clock equals Trigger input. |
|**Reserved** | [23:22] | Reserved |
|**TIMOUT** | [25:24] | Timer Output:<br>Configures the initial state of the Timer Output and whether it is affected by the Timer reset.<br><br>0 Timer output is logic one when enabled and is not affected by timer reset<br>1 Timer output is logic zero when enabled and is not affected by timer reset<br>2 Timer output is logic one when enabled and on timer reset<br>3 Timer output is logic zero when enabled and on timer reset |
|**Reserved** | [31:26] | Reserved |


### TIMCMP (Timer Compare Register)
|Field | Bits | Description |
|-------|------|-------------|
|**TIMCMP** | [15:0] | Timer Compare Value:<br>The timer compare value is loaded into the timer counter when the timer is first enabled, when the timer is reset and when the timer decrements down to zero.<br><br>In 8-bit baud counter mode, the lower 8-bits configure the baud rate divider equal to (CMP[7:0] + 1) * 2.The upper 8-bits configure the number of bits in each word equal to (CMP[15:8] + 1) / 2.<br><br>In 8-bit PWM high mode, the lower 8-bits configure the high period of the output to (CMP[7:0] + 1) and the upper 8-bits configure the low period of the output to (CMP[15:8] + 1).<br><br>In 16-bit counter mode, the compare value can be used to generate the baud rate divider (if shift clock source is timer output) to equal (CMP[15:0] + 1) * 2. When the shift clock source is a pin or trigger input, the compare register is used to set the number of bits in each word equal to (CMP[15:0] + 1) / 2. |
|**Reserved** | [31:16] | Reserved |




- logic mode
- state mode
- double buffer mode
- chaining
- cross triggering
- spi, uart, i2c, i2s, camera if, mtoral 68k/intel 8080 bus
- pwm wafverm gen
- shifter concatenation
- clock divider
- low level configuration and operation
- automatic start/stop generation
Interrupt, dma or polled rx/tx operatoin