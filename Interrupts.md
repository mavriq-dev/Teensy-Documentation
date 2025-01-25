# Teensy 4.1 Interrupts




## Introduction

Interrupts are like a doorbell for your microcontroller - no matter what your program is doing, an interrupt can temporarily pause it to handle something important right away. This ability to respond immediately to events makes interrupts a powerful feature of the Teensy 4.1.

The key components of the interrupt system include:
***The ISR (Interrupt Service Routine)*** - which is the function that runs when an interrupt occurs; 
***The Vector Table*** - which maps out where to find each ISR; 
***Priority levels*** - which determine the order of handling multiple interrupts. 
***An Interrupt Flag*** - serves as an indicator that an interrupt has occurred.

The Teensy 4.1's processor is constantly on the lookout for interrupt conditions while running your main program. When an interrupt triggers, the processor smoothly pauses your main program, saves its current state, runs the interrupt handler, restores the state, and finally returns to the main program.

Interrupts prove invaluable in many scenarios. They excel at reading sensor data at precise intervals, responding to button presses immediately, managing hardware communication (like Serial, I2C, or SPI), handling precise timing applications, and implementing effective power management strategies.

## Common Use Cases

Let's explore the most common ways interrupts are used in Teensy projects:

### 1. Timer Interrupts

Timer interrupts let you perform tasks at precise intervals. They're perfect for:

*   Periodic Tasks: Run code at exact intervals, like reading sensors every millisecond
*   Timeouts: Detect when something takes too long, like a communication response
*   PWM Generation: Create precise pulse-width signals for motors or LEDs
*   Time-based Sampling: Collect data at specific intervals for consistent measurements

```cpp
IntervalTimer myTimer;
volatile uint32_t sampleCount = 0;

void timerISR() {
    sampleCount++;                    // Count samples
    analogRead(A0);                   // Read sensor
}

void setup() {
    myTimer.begin(timerISR, 1000);   // Run ISR every 1000 microseconds (1kHz)
}
```

### 2. Communication Interrupts

These handle data transfer with other devices efficiently:

*   UART/Serial: Receive data without missing characters, even at high speeds
*   SPI Transfers: Handle high-speed data exchange with sensors or displays
*   I2C Events: Respond to slave device communications
*   USB Packets: Process USB data reliably

```cpp
void serialEvent1() {
    while (Serial1.available()) {
        char inChar = Serial1.read();
        // Process incoming character
    }
}
```

### 3. External Interrupts

React instantly to external events:

*   Button Presses: Respond immediately to user input
*   Sensor Triggers: Catch important sensor events (motion detection, limit switches)
*   External Events: Monitor state changes from other devices
*   Emergency Stops: Handle critical safety conditions

```cpp
const int buttonPin = 2;
volatile bool buttonPressed = false;

void buttonISR() {
    buttonPressed = true;             // Flag the event
}

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, FALLING);
}
```

### 4. System Interrupts

Handle critical system events:

*   Memory Management: Catch memory access violations
*   Bus Faults: Handle errors in peripheral access
*   Usage Faults: Detect programming errors like divide-by-zero
*   System Tick: Keep track of time and RTOS scheduling

```cpp
void HardFault_Handler(void) {
    // Log the fault condition
    Serial.println("Hard Fault Detected!");
    // Take recovery action or reset
    systemReset();
}
```

Each of these interrupt types serves a specific purpose, and you'll often use multiple types in a single project. For example, you might use timer interrupts to sample a sensor, external interrupts to detect user input, and communication interrupts to send the data to a computer.

## Interrupt Details

The interrupt process follows a precise sequence. When an interrupt occurs, the processor first completes its current instruction. It then saves the program counter and status registers, switches to interrupt handler mode, executes the ISR, restores the previous state, and finally returns to the main program.

The Teensy 4.1 organizes its interrupts through a vector table, which maps various interrupt sources to their handlers. This includes hardware interrupts from pins and timers, system exceptions, and software interrupts. Each entry in the vector table points to its corresponding ISR.

The priority system in the Teensy 4.1 is implemented through Teensyduino's core library, which configures the ARM Cortex-M7's NVIC to use 16 priority levels (0-15). While the processor hardware itself supports up to 256 priority levels, Teensyduino simplifies this to 16 levels for practical use. Lower numbers still indicate higher priority, with 0 being the highest priority and 15 being the lowest.

## Interrupt States

An interrupt can exist in several states throughout its lifecycle. When an interrupt first triggers but hasn't been handled yet, it enters a Pending state. Multiple pending interrupts form a queue based on their priority levels, and you can check their status through the NVIC registers.

During execution of its ISR, an interrupt is considered Active. While active, it can still be interrupted by higher priority interrupts, and the processor operates in handler mode to manage this special state.

The Inactive state represents the normal condition when an interrupt isn't triggered. In this state, the interrupt is ready to respond to new events but isn't using any system resources.

Sometimes you'll need to temporarily disable interrupts, putting them in a Masked state. You can do this using `noInterrupts()` or `NVIC_DISABLE_IRQ()`. This is particularly useful for protecting critical code sections from interruption. Just remember to re-enable interrupts using `interrupts()` or `NVIC_ENABLE_IRQ()` when you're done.

## Interrupt Masking

Sometimes you need to temporarily prevent interrupts from occurring. The Teensy 4.1 provides several ways to do this, from blocking all interrupts to selectively controlling specific ones. Here's how each method works:

### Global Masking

The simplest form of interrupt control, global masking affects all interrupts at once:

*   Complete Interrupt Control: Using `noInterrupts()` and `interrupts()`, you can quickly disable or enable all maskable interrupts
*   Atomic Operations: Perfect for when you need to ensure a sequence of instructions runs without interruption
*   Critical Section Protection: Protect important code that shouldn't be interrupted
*   System-wide Management: Useful for system initialization or when absolute timing is crucial

Example: Protecting a shared resource:
```cpp
noInterrupts();           // Block all interrupts
sharedVariable++;         // Modify shared resource
interrupts();            // Re-enable interrupts
```

### Individual Interrupt Control

For more precise control, you can enable or disable specific interrupts:

*   Selective Control: Enable or disable exactly the interrupts you need
*   Per-peripheral Management: Control interrupts for specific hardware like timers or communication interfaces
*   Fine-grained Timing: Manage interrupt sources individually without affecting others
*   Resource Protection: Prevent specific interrupts during sensitive operations

Example: Managing a timer interrupt:
```cpp
NVIC_DISABLE_IRQ(IRQ_PIT);    // Disable only the timer interrupt
// Perform timer-sensitive operations
NVIC_ENABLE_IRQ(IRQ_PIT);     // Re-enable the timer interrupt
```

### Priority-Based Masking

A more sophisticated approach that works with interrupt priority levels:

*   Priority Levels: Block interrupts below a certain priority while allowing higher priority ones
*   BASEPRI Register: Use the processor's built-in priority masking register
*   Selective Blocking: Keep critical interrupts working while blocking less important ones
*   Flexible Thresholds: Dynamically adjust which priority levels can interrupt

Example: Block low-priority interrupts:
```cpp
// Only allow interrupts with priority 0-3 (highest priorities)
set_priority_mask(4 << 4);  // Shift by 4 for priority format
```

### Fault Masking

Special handling for system faults and error conditions:

*   Fault Handlers: Configure how the system responds to different types of faults
*   Debug Features: Special modes for debugging fault conditions
*   Protection Mechanisms: Prevent critical system failures
*   Recovery Options: Define how to handle and recover from faults

Example: Setting up a fault handler:
```cpp
void HardFault_Handler(void) {
    // Log the fault
    // Attempt recovery or reset
    systemReset();
}
```

## Teensy 4.1's Interrupt System

At the heart of the Teensy 4.1's interrupt handling is the NVIC (Nested Vectored Interrupt Controller). This sophisticated system efficiently manages multiple types of interrupts, ensuring your code can respond quickly to important events while maintaining system stability.

### Interrupt Types

The Teensy 4.1 supports several types of interrupts, each serving a specific purpose:

*   Hardware Interrupts: These come from physical devices like pins, timers, or communication interfaces (SPI, I2C, etc.). They let your code know when external events need attention, like a button press or when data arrives.

*   Software Interrupts: Generated by your program itself, these are useful for scheduling tasks or handling software events. They work just like hardware interrupts but are triggered by code instead of physical events.

*   Exception Interrupts: These are system-level events that need immediate attention, such as memory access violations or undefined instructions. They help prevent system crashes by letting you handle errors gracefully.

*   Non-maskable Interrupts (NMI): These are special, highest-priority interrupts that can't be disabled. They're reserved for critical system events that must never be ignored, like power failure warnings.

### Priority System

The Teensy 4.1's interrupt priority system helps you manage multiple interrupts effectively:

*   16 Priority Levels: Ranging from 0 (highest) to 15 (lowest), letting you decide which interrupts are most important
*   Flexible Configuration: Each interrupt source can have its own priority level
*   Automatic Nesting: Higher priority interrupts can temporarily pause lower priority ones
*   Priority Groups: You can organize interrupts into groups for easier management

### Response Time

The Teensy 4.1 responds to interrupts remarkably quickly. Here's what happens when an interrupt occurs:

1. The processor detects the interrupt request
2. Current instruction completes (typically 1-2 cycles)
3. Important registers are automatically saved (about 8 cycles)
4. Your interrupt handler begins (around cycle 12)

This entire process takes about 12 cycles in ideal conditions (with zero-wait-state memory). Real-world timing might be slightly longer depending on what the processor was doing and memory access speeds, but it's still impressively fast - we're talking microseconds here!

### Interrupt Safety Guidelines

When working with interrupts, it's crucial to follow several key safety practices to ensure your code runs reliably. First and foremost, always keep your Interrupt Service Routines (ISRs) as brief as possible. Long ISRs can lead to system instability, so it's better to move complex processing to your main loop and use flags to trigger these actions when needed.

Any variables that are shared between ISRs and your main code must be declared as volatile. This crucial keyword prevents the compiler from optimizing these variables in ways that could cause unexpected behavior. For example, you might declare a shared counter as `volatile uint32_t shared_counter = 0;`.

ISRs require special handling in the Teensy 4.1. They should be marked with the `FASTRUN` attribute for optimal performance and kept as short as possible. Many Arduino functions aren't safe to use within an ISR. Here's a basic example of an ISR:

```cpp
void FASTRUN myISR() {
    // Quick interrupt handling here
}
```

Complex operations should be avoided within ISRs. This means steering clear of floating-point math unless absolutely necessary, avoiding any blocking operations or delays, and skipping Serial print statements. Instead, set flags that your main loop can check to handle these operations.

When dealing with nested interrupts, remember that higher priority interrupts can interrupt lower priority ones. For critical sections of code, you might need to temporarily disable interrupts. The NVIC_SET_PRIORITY() function helps you manage these interrupt priorities effectively.

Speaking of priorities, assign them thoughtfully. Time-critical operations should receive higher priority, while less time-sensitive tasks can work with lower priorities. The default priority sits in the middle range, giving you flexibility in both directions.

## Advanced Features

### Interrupt Chaining

The Teensy 4.1 allows you to set up multiple interrupt service routines (ISRs) for a single interrupt source. This powerful feature enables modular and flexible interrupt handling in complex applications. Here's how it works:

When you attach multiple ISRs to the same interrupt using `attachInterrupt()`, they form a chain where each handler is called in order of registration. The priority of execution follows the order in which they were attached - the first attached handler runs first. This is particularly useful when different parts of your code need to respond to the same event.

```cpp
void firstHandler() {
    // First response to the interrupt
}

void secondHandler() {
    // Secondary response to the interrupt
}

void setup() {
    // Both handlers will run when pin 2 goes HIGH
    attachInterrupt(2, firstHandler, RISING);
    attachInterrupt(2, secondHandler, RISING);
}
```

You can dynamically register and remove interrupt handlers during runtime using `attachInterrupt()` and `detachInterrupt()`. This flexibility allows you to adapt your interrupt handling based on your application's current state.

### FreeRTOS Integration

When using FreeRTOS with the Teensy 4.1, proper interrupt handling becomes crucial for system stability. The IMXRT1062 processor supports priority levels that work seamlessly with FreeRTOS, but there are some important considerations:

Interrupt priorities in FreeRTOS context must be carefully managed. FreeRTOS requires some interrupt priority levels for its own operation. On the Teensy 4.1, you should use `NVIC_SET_PRIORITY()` to set interrupt priorities, ensuring they don't interfere with FreeRTOS system interrupts.

For handling interrupts in an RTOS environment, consider using deferred processing:

```cpp
// Create a task handle for deferred processing
TaskHandle_t processingTask;

void FASTRUN irqHandler() {
    // Minimal processing in ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Notify task from ISR
    vTaskNotifyGiveFromISR(processingTask, &xHigherPriorityTaskWoken);
    
    // Request context switch if needed
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void processingTask(void* parameters) {
    while(1) {
        // Wait for notification from ISR
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // Handle the interrupt event
    }
}
```

### DMA Integration

The Teensy 4.1's DMA system can work hand-in-hand with interrupts for efficient data transfer. The DMA controller can trigger interrupts at various points during a transfer, letting you know when operations complete or if errors occur.

Here's a typical pattern for DMA with interrupt coordination:

```cpp
void dmaCallback() {
    // Transfer complete or error occurred
    if (DMA_ERR_STATUS) {
        // Handle error condition
    } else {
        // Process completed transfer
        // Maybe start next transfer
    }
}

void setupDMATransfer() {
    // Configure DMA channel
    DMA_CHANNEL.TCD->DLASTSGA = (uint32_t)dmaCallback;
    // Enable interrupt on major loop completion
    DMA_CHANNEL.TCD->CSR |= DMA_CSR_INTMAJOR;
}
```

Buffer management with DMA typically involves double-buffering or circular buffer techniques. This allows continuous data processing while maintaining interrupt responsiveness.

### Power Management

The Teensy 4.1 supports various power modes, and interrupts play a key role in power management. Interrupts can wake the processor from low-power states, making them essential for energy-efficient designs.

When configuring wake-up sources:

```cpp
void configureLowPowerWakeup() {
    // Configure pin 2 as wake source
    attachInterrupt(2, wakeHandler, FALLING);
    // Enter low power mode
    // Note: Available modes depend on which peripherals 
    // need to remain active
    SNVS_LPCR |= SNVS_LPCR_TOP_MASK;
}
```

Remember that different power modes affect which interrupts can wake the system. Some considerations:

- Light sleep mode allows most interrupts to function
- Deep sleep requires specific wake sources to be configured
- Clock domains may need reconfiguration after wake-up
- Some peripherals require re-initialization after deep sleep

When transitioning between power modes, ensure your interrupt handlers are prepared for the state change. Some peripherals may need reconfiguration after waking from deeper sleep states.

## Common Interrupt Functions

#### Pin Interrupts

```cpp
attachInterrupt(pin, ISR, mode);   // Attach interrupt to pin
detachInterrupt(pin);              // Detach interrupt from pin
```

#### GPIO Interrupts

*   Used for GPIO pin interrupts
*   Modes: RISING, FALLING, CHANGE, LOW, HIGH

```cpp
void pinCallback() {
    // Handle pin change
}
void setup() {
    pinMode(pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin), pinCallback, RISING);
}
```

#### DMA Interrupts

*   Specific to DMA transfers
*   Triggers when DMA transfer completes

```cpp
DMAChannel dma;
dma.attachInterrupt(callback);
```

#### Interval Timer

*   For precise timing interrupts
*   Up to 4 simultaneous timers
*   Not tied to any specific pin

```cpp
IntervalTimer myTimer;
myTimer.begin(callback, microseconds);
```

#### Port Interrupts

*   For handling multiple pins in a port
*   More efficient than individual pin interrupts

```cpp
attachInterruptVector(IRQ_GPIO6789, callback);
```

### Basic Interrupt Control

The Teensy 4.1 uses two simple functions to control interrupts globally. These work by managing the processor's PRIMASK register.

#### Disabling Interrupts

```cpp
noInterrupts();                    // Disable all interrupts
```

When you call `noInterrupts()`:

*   Blocks all regular interrupts
*   Only NMI and HardFault remain active
*   Perfect for protecting critical code sections
*   Use for very short operations only

#### Enabling Interrupts

```cpp
interrupts();                      // Enable all interrupts
```

When you call `interrupts()`:

*   Allows interrupts to trigger again
*   Processes any waiting interrupts
*   Call right after your critical code

#### Quick Example

```cpp
volatile uint32_t sharedCounter = 0;

void updateCounter(uint32_t newValue) {
    noInterrupts();                // Start protected section
    sharedCounter = newValue;      // Safe to update
    interrupts();                  // Back to normal
}
```

#### Key Points

1.  Keep protected sections short
2.  Avoid delays while interrupts are off
3.  Remember timing functions might pause
4.  Some interrupts can't be blocked

#### Individual Interrupt Control

```cpp
// Enable specific interrupt
NVIC_ENABLE_IRQ(IRQ_FLEXIO1);    // Enable FlexIO1 interrupt
NVIC_ENABLE_IRQ(IRQ_DMA_CH0);    // Enable DMA channel 0

// Disable specific interrupt
NVIC_DISABLE_IRQ(IRQ_FLEXIO1);   // Disable FlexIO1 interrupt
NVIC_DISABLE_IRQ(IRQ_DMA_CH0);   // Disable DMA channel 0

// Clear pending interrupt
NVIC_CLEAR_PENDING(IRQ_FLEXIO1); // Clear pending FlexIO1 interrupt
```

#### Priority Masking (disable all below certain priority)

This is available in the processor, but is not part of the Teensyduino library. If needed you can set the register directly

```cpp
// Write to BASEPRI directly using inline assembly
static inline void set_priority_mask(uint8_t priority) {
    asm volatile("msr basepri, %0" : : "r" (priority) : "memory");
}

// Clear priority mask
static inline void clear_priority_mask() {
    asm volatile("msr basepri, %0" : : "r" (0) : "memory");
}
```

### Priority Control

The Teensy 4.1's interrupt system lets you decide which interrupts are more important than others. Think of it like a priority list for who gets to speak first:

*   256 priority levels (0-255)
*   Lower numbers mean higher priority
*   Default priority is 128
*   Multiple interrupts can share priorities
*   Higher priority interrupts can interrupt lower ones

Here's how priorities typically work:

*   0-64: Critical operations (timer accuracy, fast sensor readings)
*   65-128: Important but not critical (USB, Serial)
*   129-192: Background operations (LED updates, button checks)
*   193-255: Low-priority tasks (status updates, debugging)

#### Setting Priorities

```cpp
// Using NVIC directly
NVIC_SET_PRIORITY(IRQ_FLEXIO1, priority);  // priority 0-255

// For DMA
_dma.priority(priority);  // priority 0-3 for DMA channels

// For attachInterrupt
attachInterruptVector(IRQ_FLEXIO1, callback);
NVIC_SET_PRIORITY(IRQ_FLEXIO1, priority);
```

#### Nesting Behavior

The Teensy 4.1's interrupt system follows these nesting rules:

1.  Priority-based Nesting
    *   Higher priority interrupts can preempt (interrupt) lower priority ones
    *   The lower priority ISR pauses while the higher priority one runs
    *   After the high priority ISR finishes, the lower priority one resumes

2.  Same-Level Priority
    *   Interrupts with equal priority run in order of arrival
    *   They cannot interrupt each other
    *   First-in-first-out (FIFO) processing

3.  Global Control
    *   `noInterrupts()` blocks all maskable interrupts regardless of priority
    *   Only Non-Maskable Interrupts (NMI) can still trigger
    *   Use for very short critical sections only

>   ⚠️ **Note:** While interrupt nesting can improve responsiveness, it also makes debugging more complex. Use priority levels thoughtfully.

```cpp
// Examples of setting priorities
NVIC_SET_PRIORITY(IRQ_GPIO6789, 64);    // High priority for fast GPIO response
NVIC_SET_PRIORITY(IRQ_LPUART6, 128);    // Medium priority for Serial
NVIC_SET_PRIORITY(IRQ_LPSPI4, 192);     // Lower priority for SPI

// Check current priority
uint8_t currentPriority = NVIC_GET_PRIORITY(IRQ_GPIO6789);
```

## Interrupt Enable Register (IER)

The Interrupt Enable Register (IER) is a crucial component in hardware interrupt handling, particularly in peripherals like SPI, UART, and timers. It controls which specific hardware conditions can trigger interrupts.

### Understanding the IER

Each bit in the IER corresponds to a different type of interrupt. For example, in the LPSPI (Low Power SPI) module:

```cpp
// LPSPI Interrupt Enable Register bits
LPSPI_IER_DMIE  (bit 13) // Data Match Interrupt Enable
LPSPI_IER_REIE  (bit 12) // Receive Error Interrupt Enable
LPSPI_IER_TEIE  (bit 11) // Transmit Error Interrupt Enable
LPSPI_IER_TCIE  (bit 10) // Transfer Complete Interrupt Enable
LPSPI_IER_FCIE  (bit 9)  // Frame Complete Interrupt Enable
LPSPI_IER_WCIE  (bit 8)  // Word Complete Interrupt Enable
LPSPI_IER_RDIE  (bit 1)  // Receive Data Interrupt Enable
LPSPI_IER_TDIE  (bit 0)  // Transmit Data Interrupt Enable
```

### Common Bit Manipulation Operations

1.  **Setting a Bit (Enable an Interrupt)**
    ```cpp
    // Using OR operation (|=)
    LPSPI4_IER |= LPSPI_IER_TCIE;  // Enable Transfer Complete Interrupt
    // OR
    LPSPI4_IER |= (1 << 10);       // Same thing using bit shift
    ```

2.  **Clearing a Bit (Disable an Interrupt)**
    ```cpp
    // Using AND operation with inverted bit (&= ~)
    LPSPI4_IER &= ~LPSPI_IER_TCIE;  // Disable Transfer Complete Interrupt
    // OR
    LPSPI4_IER &= ~(1 << 10);       // Same thing using bit shift
    ```

3.  **Checking if a Bit is Set**
    ```cpp
    if (LPSPI4_IER & LPSPI_IER_TCIE) {
        // Transfer Complete Interrupt is enabled
    }
    ```

4.  **Setting Multiple Bits**
    ```cpp
    // Enable multiple interrupts at once
    LPSPI4_IER |= (LPSPI_IER_TCIE | LPSPI_IER_RDIE);
    ```

5.  **Clearing Multiple Bits**
    ```cpp
    // Disable multiple interrupts at once
    LPSPI4_IER &= ~(LPSPI_IER_TCIE | LPSPI_IER_RDIE);
    ```

### Best Practices

1.  **Atomic Operations**: When possible, use single operations to modify bits to avoid race conditions.
    ```cpp
    // Good - single atomic operation
    LPSPI4_IER |= (LPSPI_IER_TCIE | LPSPI_IER_RDIE);
    
    // Bad - multiple operations could be interrupted
    LPSPI4_IER |= LPSPI_IER_TCIE;
    LPSPI4_IER |= LPSPI_IER_RDIE;
    ```

2.  **Clear Pending Interrupts**: When enabling an interrupt, clear any pending interrupts first.
    ```cpp
    LPSPI4_SR = LPSPI_SR_TCF;     // Clear pending interrupt flag
    LPSPI4_IER |= LPSPI_IER_TCIE; // Then enable the interrupt
    ```

3.  **Disable Before Configuration**: Disable interrupts before changing critical configurations.
    ```cpp
    uint32_t temp = LPSPI4_IER;        // Save current state
    LPSPI4_IER &= ~LPSPI_IER_TCIE;     // Disable interrupt
    // ... perform configuration ...
    LPSPI4_IER = temp;                  // Restore previous state
    ```

## Important Interrupt Registers

The Teensy 4.1 uses several key registers for interrupt control:

1. **NVIC Control Registers**
   ```cpp
   NVIC_ISER0    // Interrupt Set Enable Register
   NVIC_ICER0    // Interrupt Clear Enable Register
   NVIC_ISPR0    // Set Pending Register
   NVIC_ICPR0    // Clear Pending Register
   NVIC_IPR0     // Priority Register
   ```

2. **Core Status/Control Registers**
   ```cpp
   // Used by noInterrupts() and interrupts()
   PRIMASK       // Global Interrupt Mask
   
   // For priority-based masking
   BASEPRI       // Base Priority Mask
   
   // For system faults
   FAULTMASK     // Fault Interrupt Mask
   ```

3. **Peripheral Status Registers**
   ```cpp
   // Example for LPSPI4
   LPSPI4_IMR    // Interrupt Mask Register
   LPSPI4_ISR    // Interrupt Status Register
   LPSPI4_ICR    // Interrupt Clear Register
   ```

These registers are typically accessed through Teensyduino's wrapper functions for safety and portability. These are provided in the case you need to ustilize low level code for features not exposed through teensy core. For example, instead of writing to NVIC_ISER0 directly, use:
```cpp
NVIC_ENABLE_IRQ(IRQ_WHATEVER);  // Safer than direct register access
```

## Example Code

##### Priority-Based Masking Example:
```cpp
class PriorityMask {
private:
    uint32_t _oldPriMask;
    static constexpr uint8_t NVIC_PRIO_BITS = 4;  // Teensy 4.1 uses 4 priority bits

public:
    // Mask all interrupts below specified priority
    void maskBelowPriority(uint8_t priority) {
        uint32_t basePri;
        asm volatile("MRS %0, BASEPRI" : "=r" (_oldPriMask));
        basePri = priority << (8 - NVIC_PRIO_BITS);
        asm volatile("MSR BASEPRI, %0" : : "r" (basePri));
    }
    
    // Restore previous mask
    void restore() {
        asm volatile("MSR BASEPRI, %0" : : "r" (_oldPriMask));
    }
};
```

##### Selective Interrupt Masking:
```cpp
class SelectiveInterruptMask {
private:
    uint32_t _savedMask;
    IRQ_NUMBER_t _irq;

public:
    void disableInterrupt() {
        _savedMask = NVIC_GET_ENABLE(_irq);
        NVIC_DISABLE_IRQ(_irq);
    }
    
    void enableInterrupt() {
        if (_savedMask) {
            NVIC_ENABLE_IRQ(_irq);
        }
    }
};
```

##### RAII-Style Critical Section:
```cpp
class SelectiveInterruptMask {
private:
    bool _wasEnabled;
    IRQ_NUMBER_t _irq;

public:
    SelectiveInterruptMask(IRQ_NUMBER_t irq) : _irq(irq) {
        _wasEnabled = NVIC_IS_ENABLED(_irq);
        NVIC_DISABLE_IRQ(_irq);
    }
    
    ~SelectiveInterruptMask() {
        if (_wasEnabled) {
            NVIC_ENABLE_IRQ(_irq);
        }
    }
};
```

##### Button Debouncing with Interrupts
```cpp
class DebouncedButton {
private:
    static const uint32_t _debounceTime = 50; // milliseconds
    const uint8_t _pin;
    volatile uint32_t _lastInterruptTime;
    volatile bool _state;

public:
    DebouncedButton(uint8_t pin) : _pin(pin), _lastInterruptTime(0), _state(false) {
        pinMode(_pin, INPUT_PULLUP);
    }
    
    void begin() {
        // Capture 'this' pointer for the lambda
        DebouncedButton* btn = this;
        // Teensy-specific pin interrupt attachment
        attachInterrupt(digitalPinToInterrupt(_pin), 
                       [btn]() {
                           uint32_t currentTime = millis();
                           if (currentTime - btn->_lastInterruptTime >= _debounceTime) {
                               btn->_state = digitalReadFast(btn->_pin);
                               btn->_lastInterruptTime = currentTime;
                           }
                       },
                       CHANGE);
    }
    
    void end() {
        detachInterrupt(digitalPinToInterrupt(_pin));
    }
    
    bool getState() const {
        return _state;
    }
};
```

##### Circular Buffer with Interrupt
```cpp
template<typename T, size_t Size>
class InterruptBuffer {
private:
    static_assert((Size & (Size - 1)) == 0, "Size must be power of 2");
    
    volatile T _buffer[Size];
    volatile size_t _writeIndex;
    volatile size_t _readIndex;
    static const size_t _mask = Size - 1;

public:
    InterruptBuffer() : _writeIndex(0), _readIndex(0) {}
    
    void push(T value) {
        noInterrupts();  // Critical section
        size_t nextWrite = (_writeIndex + 1) & _mask;
        if (nextWrite != _readIndex) {
            _buffer[_writeIndex] = value;
            _writeIndex = nextWrite;
        }
        interrupts();
    }

    bool pop(T& value) {
        noInterrupts();
        if (_readIndex == _writeIndex) {
            interrupts();
            return false;
        }
        value = _buffer[_readIndex];
        _readIndex = (_readIndex + 1) & _mask;
        interrupts();
        return true;
    }
    
    bool isEmpty() const {
        return _readIndex == _writeIndex;
    }
    
    void clear() {
        noInterrupts();
        _readIndex = _writeIndex = 0;
        interrupts();
    }
};

// Example 1: Using InterruptBuffer with ADC readings
void Example_ADC_Buffer() {
    // Create a buffer for 16 ADC readings
    InterruptBuffer<uint16_t, 16> adcBuffer;
    
    // In an interrupt handler (e.g., ADC completion ISR)
    void adc_isr() {
        uint16_t adcValue = analogRead(A0);
        adcBuffer.push(adcValue);
    }
    
    // In the main loop
    void processADCReadings() {
        uint16_t value;
        while (adcBuffer.pop(value)) {
            Serial.printf("ADC Reading: %d\n", value);
        }
    }
}

// Example 2: Using InterruptBuffer with a custom structure
struct SensorData {
    uint32_t timestamp;
    float temperature;
    float humidity;
    
    SensorData(float temp = 0.0f, float hum = 0.0f) 
        : timestamp(millis()), temperature(temp), humidity(hum) {}
};

void Example_Sensor_Buffer() {
    // Create a buffer for 32 sensor readings
    InterruptBuffer<SensorData, 32> sensorBuffer;
    
    // In a timer interrupt
    void sensor_timer_isr() {
        float temp = readTemperature();  // Your sensor reading function
        float humidity = readHumidity(); // Your sensor reading function
        sensorBuffer.push(SensorData(temp, humidity));
    }
    
    // In the main loop
    void processSensorData() {
        SensorData data;
        while (sensorBuffer.pop(data)) {
            Serial.printf("Time: %lu, Temp: %.2f, Humidity: %.2f%%\n",
                         data.timestamp, data.temperature, data.humidity);
        }
    }
}

// Example 3: Using InterruptBuffer for button events
enum class ButtonEvent {
    PRESSED,
    RELEASED,
    HELD
};

void Example_Button_Events() {
    InterruptBuffer<ButtonEvent, 8> buttonEvents;
    
    // In button interrupt handler
    void button_isr() {
        if (digitalRead(BUTTON_PIN) == LOW) {
            buttonEvents.push(ButtonEvent::PRESSED);
        } else {
            buttonEvents.push(ButtonEvent::RELEASED);
        }
    }
    
    // In main loop
    void processButtonEvents() {
        ButtonEvent event;
        while (buttonEvents.pop(event)) {
            switch (event) {
                case ButtonEvent::PRESSED:
                    Serial.println("Button Pressed");
                    break;
                case ButtonEvent::RELEASED:
                    Serial.println("Button Released");
                    break;
                case ButtonEvent::HELD:
                    Serial.println("Button Held");
                    break;
            }
        }
    }
}

// Example 4: Using InterruptBuffer for timer events
void Example_Timer_Events() {
    InterruptBuffer<uint32_t, 16> timerEvents;
    
    // In timer interrupt handler
    void timer_isr() {
        timerEvents.push(millis());
    }
    
    // In main loop
    void processTimerEvents() {
        uint32_t timestamp;
        while (timerEvents.pop(timestamp)) {
            Serial.printf("Timer Event at %lu\n", timestamp);
        }
    }
}
```

##### Timer Interrupt Manager
```cpp
class TimerManager {
private:
    IntervalTimer _timer;
    volatile uint32_t _counter;
    std::function<void(void)> _callback;

public:
    TimerManager() : _counter(0) {}
    
    void begin(float frequency, std::function<void(void)> callback) {
        _callback = callback;
        _counter = 0;
        _timer.begin([this]() { 
            _counter++;
            _callback();
        }, 1000000.0f / frequency);
    }

    void end() {
        _timer.end();
    }

    uint32_t getCount() {
        noInterrupts();
        uint32_t count = _counter;
        interrupts();
        return count;
    }
};

// Example usage of TimerManager
void Example_Timer_Blink() {
    TimerManager timer;
    volatile bool ledState = false;
    
    // Define the callback function for the timer
    auto blinkCallback = [&ledState]() {
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    };
    
    // Start blinking at 1Hz (once per second)
    timer.begin(1.0f, blinkCallback);
    
    // Main loop example
    for(int i = 0; i < 10; i++) {
        Serial.printf("Blink count: %lu\n", timer.getCount());
        delay(1000);
    }
    
    // Cleanup
    timer.end();
}
```

###### Priority-based Interrupt Handler
```cpp
class PriorityInterrupt {
private:
    const uint8_t _pin;
    const uint8_t _priority;

public:
    PriorityInterrupt(uint8_t pin, uint8_t priority) : 
        _pin(pin), _priority(priority) {
        pinMode(_pin, INPUT_PULLUP);
        
        // Get IRQ number for the pin
        IRQ_NUMBER_t irq = (IRQ_NUMBER_t)digitalPinToInterrupt(_pin);
        
        // Attach interrupt
        attachInterruptVector(irq, handleInterrupt);
        
        // Set priority (0-15, 0 is highest)
        NVIC_SET_PRIORITY(irq, _priority);
        
        // Enable the interrupt
        NVIC_ENABLE_IRQ(irq);
    }

private:
    static void handleInterrupt() {
        // Handle the interrupt
        // Note: Keep this as short as possible
    }
};
```

### Low Level Code Examples

##### Short and Fast ISR
```cpp
volatile uint32_t _adcValue = 0;

void setup() {
    // Direct port configuration for fastest response
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_31 = 5; // Configure pin 33 as GPIO
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_31 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    GPIO7_GDIR &= ~(1 << 31);  // Set as input
    attachInterruptVector(IRQ_GPIO6789, portaISR);
    GPIO7_IMR |= (1 << 31);    // Enable interrupt for pin
    NVIC_ENABLE_IRQ(IRQ_GPIO6789);
}

void __attribute__((section(".fastrun"))) portaISR() {
    uint32_t status = GPIO7_ISR;
    GPIO7_ISR = status;  // Clear interrupt flags
    
    // Direct ADC register access for speed
    ADC1_HC0 = 0x83;  // Start conversion on AD3
    while (!(ADC1_HS & ADC_HS_COCO0)) ; // Wait for completion
    _adcValue = ADC1_R0;
}
```

##### Priority Levels
```cpp
void setup() {
// Volatile flags for ISR communication
volatile bool criticalFlag = false;
volatile bool normalFlag = false;

// Fast ISR for critical events (GPIO6)
void __attribute__((section(".fastrun"))) criticalISR() {
    uint32_t status = GPIO6_ISR & (1 << 16);  // Check pin 4
    if (status) {
        GPIO6_ISR = status;  // Clear only this interrupt flag
        criticalFlag = true;
    }
}

// Normal priority ISR (GPIO2)
void __attribute__((section(".fastrun"))) normalISR() {
    uint32_t status = GPIO2_ISR & (1 << 4);  // Check pin 6
    if (status) {
        GPIO2_ISR = status;  // Clear only this interrupt flag
        normalFlag = true;
    }
}

void setup() {
    Serial.begin(115200);
    
    // Configure System Control Block for 8 priority levels (3 bits)
    SCB_AIRCR = 0x05FA0300;  // Key(0x05FA) | PRIGROUP(3)
    
    // Configure pin 4 (GPIO6_16) - Critical interrupt
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 5;  // Configure as GPIO
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_00 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
    GPIO6_GDIR &= ~(1 << 16);  // Set as input
    GPIO6_ICR1 |= (0x2 << 0);  // Rising edge
    GPIO6_IMR |= (1 << 16);    // Enable interrupt
    attachInterruptVector(IRQ_GPIO6789, criticalISR);
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 0);  // Highest priority (0-7)
    NVIC_ENABLE_IRQ(IRQ_GPIO6789);
    
    // Configure pin 6 (GPIO2_4) - Normal interrupt
    IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_04 = 5;  // Configure as GPIO
    IOMUXC_SW_PAD_CTL_PAD_GPIO_B0_04 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3);
    GPIO2_GDIR &= ~(1 << 4);   // Set as input
    GPIO2_ICR1 |= (0x2 << 8);  // Rising edge
    GPIO2_IMR |= (1 << 4);     // Enable interrupt
    attachInterruptVector(IRQ_GPIO2_0_15, normalISR);
    NVIC_SET_PRIORITY(IRQ_GPIO2_0_15, 2);  // Lower priority (0-7)
    NVIC_ENABLE_IRQ(IRQ_GPIO2_0_15);
}
}
```

##### Handle Errors Gracefully (Low-Level Only Features)

```cpp
// ARM Cortex-M7 Auxiliary Fault Status Register
#define SCB_AFSR           (*(volatile uint32_t *)(0xE000ED3C))  // Auxiliary Fault Status Register

// Advanced fault handling
void __attribute__((naked)) HardFault_Handler(void) {
    __asm__ volatile(
        "tst lr, #4\n"
        "ite eq\n"
        "mrseq r0, msp\n"
        "mrsne r0, psp\n"
        "b hard_fault_handler_c\n"
    );
}

void hard_fault_handler_c(uint32_t* hardfault_args) {
    volatile uint32_t stacked_r0 = hardfault_args[0];
    volatile uint32_t stacked_r1 = hardfault_args[1];
    volatile uint32_t stacked_r2 = hardfault_args[2];
    volatile uint32_t stacked_r3 = hardfault_args[3];
    volatile uint32_t stacked_r12 = hardfault_args[4];
    volatile uint32_t stacked_lr = hardfault_args[5];
    volatile uint32_t stacked_pc = hardfault_args[6];
    volatile uint32_t stacked_psr = hardfault_args[7];
    
    // Log fault details
    Serial.printf("Hard Fault:\n");
    Serial.printf("R0 = %x\n", stacked_r0);
    Serial.printf("R1 = %x\n", stacked_r1);
    Serial.printf("R2 = %x\n", stacked_r2);
    Serial.printf("R3 = %x\n", stacked_r3);
    Serial.printf("R12 = %x\n", stacked_r12);
    Serial.printf("LR = %x\n", stacked_lr);
    Serial.printf("PC = %x\n", stacked_pc);
    Serial.printf("PSR = %x\n", stacked_psr);
    Serial.printf("BFAR = %x\n", SCB_BFAR);
    Serial.printf("CFSR = %x\n", SCB_CFSR);
    Serial.printf("HFSR = %x\n", SCB_HFSR);
    Serial.printf("DFSR = %x\n", SCB_DFSR);
    Serial.printf("AFSR = %x\n", SCB_AFSR);
    
    // Optional: Attempt recovery or reset
    // SCB_AIRCR = 0x05FA0004; // System reset
}
```

## Common Interrupt Reference

Here's a quick reference for commonly used interrupt vectors, masks, and flags on the Teensy 4.1. These definitions come from various sources in the Teensy ecosystem:

* Teensyduino Core (`cores/teensy4/`): Basic interrupt vectors and NVIC functions
* IMXRT1062 Headers (`imxrt.h`): Hardware-specific registers and flags
* CMSIS Headers: ARM Cortex-M7 system-level definitions

### Timer Interrupts (from IMXRT1062 Headers)

| Vector Name | Description | Enable Mask | Status Flag | Default ISR | Priority |
|------------|-------------|-------------|-------------|-------------|-----------|
| `IRQ_PIT` | Periodic Interrupt Timer | `PIT_TCTRL0_TIE` | `PIT_TFLG0` | `pit0_isr()` | 48 |
| `IRQ_GPT1` | General Purpose Timer 1 | `GPT1_IR_OF1IE` | `GPT1_SR_OF1` | `gpt1_isr()` | 48 |
| `IRQ_GPT2` | General Purpose Timer 2 | `GPT2_IR_OF1IE` | `GPT2_SR_OF1` | `gpt2_isr()` | 48 |
| `IRQ_QTIMER1` | Quad Timer 1 | `TMR1_SCTRL_TCF1EN` | `TMR1_SCTRL_TCF` | `qtimer1_isr()` | 48 |

### Communication Interrupts (from IMXRT1062 Headers and Teensyduino Core)

| Vector Name | Description | Enable Mask | Status Flag | Default ISR | Priority |
|------------|-------------|-------------|-------------|-------------|-----------|
| `IRQ_LPUART6` | Serial1 | `LPUART6_CTRL_RIE` | `LPUART6_STAT_RDRF` | `lpuart6_isr()` | 64 |
| `IRQ_LPUART4` | Serial2 | `LPUART4_CTRL_RIE` | `LPUART4_STAT_RDRF` | `lpuart4_isr()` | 64 |
| `IRQ_LPUART2` | Serial3 | `LPUART2_CTRL_RIE` | `LPUART2_STAT_RDRF` | `lpuart2_isr()` | 64 |
| `IRQ_LPUART3` | Serial4 | `LPUART3_CTRL_RIE` | `LPUART3_STAT_RDRF` | `lpuart3_isr()` | 64 |
| `IRQ_LPUART8` | Serial5 | `LPUART8_CTRL_RIE` | `LPUART8_STAT_RDRF` | `lpuart8_isr()` | 64 |
| `IRQ_LPUART1` | Serial6 | `LPUART1_CTRL_RIE` | `LPUART1_STAT_RDRF` | `lpuart1_isr()` | 64 |
| `IRQ_LPUART7` | Serial7 | `LPUART7_CTRL_RIE` | `LPUART7_STAT_RDRF` | `lpuart7_isr()` | 64 |
| `IRQ_LPSPI4` | SPI | `LPSPI4_IER_TCIE` | `LPSPI4_SR_TCF` | `lpspi4_isr()` | 64 |
| `IRQ_LPI2C1` | I2C | `LPI2C1_MER_RDIE` | `LPI2C1_MSR_RDF` | `lpi2c1_isr()` | 64 |
| `IRQ_USB1` | USB | `USB1_INTEN` | `USB1_ISTAT` | `usb1_isr()` | 112 |

### External Pin Interrupts (from Teensyduino Core)

| Vector Name | Description | Enable Function | Status Register | Priority |
|------------|-------------|-----------------|-----------------|-----------|
| `IRQ_GPIO6789` | GPIO Ports 6-9 | `attachInterrupt()` | `GPIO6_DR` to `GPIO9_DR` | 16 |
| `IRQ_GPIO1` | GPIO Port 1 | `attachInterrupt()` | `GPIO1_DR` | 16 |
| `IRQ_GPIO2` | GPIO Port 2 | `attachInterrupt()` | `GPIO2_DR` | 16 |
| `IRQ_GPIO3` | GPIO Port 3 | `attachInterrupt()` | `GPIO3_DR` | 16 |

### System Interrupts (from CMSIS/Teensyduino Core)

| Vector Name | Description | Source | Priority | Notes |
|------------|-------------|--------|-----------|-------|
| `SysTick_IRQn` | System Tick | CMSIS | 128 | Used by Arduino timing functions |
| `HardFault_IRQn` | Hard Fault | CMSIS | -1 | System errors |
| `PendSV_IRQn` | Pendable Service | CMSIS | 255 | Used by RTOS |
| `SVC_IRQn` | Supervisor Call | CMSIS | 0 | System calls |
| `IRQ_SOFTWARE` | Software Interrupt | Core | 240 | User software interrupts |
| `IRQ_AUDIOCODEC` | Audio Shield | Core | 144 | CS42448 & SGTL5000 |
| `IRQ_AUDIOMEM` | Audio Memory | Core | 144 | Audio memory transfer |
| `IRQ_AUDIOUSB` | Audio USB | Core | 144 | USB audio transfer |

### NVIC Functions (from Teensyduino Core)

```cpp
// Enable/disable interrupts
NVIC_ENABLE_IRQ(IRQn_Type irq);   // Enable specific interrupt
NVIC_DISABLE_IRQ(IRQn_Type irq);  // Disable specific interrupt
NVIC_SET_PRIORITY(IRQn_Type irq, uint8_t priority);  // Set priority (0-255)
NVIC_GET_PRIORITY(IRQn_Type irq);  // Get current priority
```

Note: When using these interrupts, always refer to the official IMXRT1062 Reference Manual for the most up-to-date register definitions and proper usage.


## Additional Resources

1.  ARM Cortex-M7 Reference Manual
    *   Detailed interrupt architecture
    *   NVIC programming model
    *   Exception handling
    *   Priority configuration

2.  Teensy 4.1 Technical Reference
    *   Interrupt vector table
    *   Pin interrupt mapping
    *   Peripheral interrupt sources
    *   System configuration

3.  IMXRT1062 Reference Manual
    *   Interrupt controller details
    *   DMA interrupt integration
    *   Power management
    *   Clock configuration
