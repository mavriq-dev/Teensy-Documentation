# Teensy 4.1 Interrupts

Copyright &copy; 2025 Geoff Van Brunt

This documentation is licensed under the MIT License.

Permission is hereby granted, free of charge, to any person obtaining a copy of this documentation and associated files, to deal in the documentation without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the documentation, and to permit persons to whom the documentation is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the documentation.

THE DOCUMENTATION IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE DOCUMENTATION OR THE USE OR OTHER DEALINGS IN THE DOCUMENTATION.

## Introduction

Interrupts are essential mechanisms that allow the microcontroller to respond to events immediately, regardless of what the main program is doing. The Teensy 4.1, based on the ARM Cortex-M7 processor, provides a sophisticated interrupt system with multiple priority levels and advanced features.

## Basic Concepts

1. Interrupt Types
   - Hardware Interrupts: Triggered by peripheral devices
   - Software Interrupts: Triggered by software
   - Exception Interrupts: System-level events
   - Non-maskable Interrupts (NMI): Highest priority, can't be disabled

2. Priority Levels
   - 16 priority levels (0-15, 0 being highest)
   - Configurable for each interrupt source
   - Nested interrupt support
   - Priority grouping options

3. Interrupt States
   - Pending: Interrupt has occurred but not yet serviced
   - Active: Currently being serviced
   - Inactive: Not pending or active
   - Masked: Temporarily disabled

## Key Features

### 1. Vectored Interrupt Controller (VIC)

The VIC is a sophisticated hardware component that manages interrupt handling efficiently:

a) Direct ISR Branching
   - Automatically jumps to the correct Interrupt Service Routine (ISR)
   - No software overhead for determining which ISR to execute
   - Reduces interrupt latency significantly
   - Critical for real-time applications

b) Hardware State Management
   - Automatically saves processor state (registers)
   - Handles stack operations efficiently
   - Restores state after ISR completion
   - Ensures reliable context switching

c) Vector Table Management
   - Configurable vector table location
   - Flexible ISR assignment
   - Runtime vector table relocation
   - Support for dynamic ISR registration

d) Performance Benefits
   - Typical response in 12-14 clock cycles
   - Deterministic interrupt handling
   - Minimal jitter in response time
   - Efficient return-from-interrupt processing

### 2. Nested Vectored Interrupt Controller (NVIC)

The NVIC extends the basic VIC with advanced features:

a) Interrupt Nesting
   - Higher priority interrupts can preempt lower ones
   - Up to 16 priority levels (0-15)
   - Hardware-managed interrupt stack
   - Automatic priority arbitration
   
b) Dynamic Priority Management
   - Runtime priority adjustment
   - Priority grouping options
   - Priority inheritance support
   - Deadlock prevention mechanisms

c) Tail-Chaining Optimization
   - Efficient handling of pending interrupts
   - Eliminates unnecessary stack operations
   - Reduces latency between ISRs
   - Optimizes multiple interrupt scenarios

d) Late-Arrival Handling
   - Handles higher priority interrupts arriving during ISR entry
   - Prevents priority inversion
   - Maintains correct execution order
   - Reduces worst-case latency

e) Implementation Example:
```cpp
class NestedInterruptManager {
private:
    static const uint8_t MAX_PRIORITY = 15;
    IRQ_NUMBER_t _irqNumber;
    
    // AIRCR key (0x5FA) in upper 16 bits
    static const uint32_t AIRCR_VECTKEY = 0x05FA0000;
    // Priority group position in AIRCR
    static const uint32_t AIRCR_PRIGROUP_SHIFT = 8;
    static const uint32_t AIRCR_PRIGROUP_MASK = 7 << AIRCR_PRIGROUP_SHIFT;

public:
    // Priority group settings
    enum class PriorityGroup {
        GROUP_16_0 = 0x0, // 16 preemption priorities, 0 sub-priorities
        GROUP_8_2  = 0x4, // 8 preemption priorities, 2 sub-priorities
        GROUP_4_4  = 0x5, // 4 preemption priorities, 4 sub-priorities
        GROUP_2_8  = 0x6, // 2 preemption priorities, 8 sub-priorities
        GROUP_0_16 = 0x7  // 0 preemption priorities, 16 sub-priorities
    };

    void setPriorityGrouping(PriorityGroup group) {
        uint32_t aircr = SCB_AIRCR;
        aircr &= ~(AIRCR_PRIGROUP_MASK | 0xFFFF0000);
        aircr |= AIRCR_VECTKEY | ((uint32_t)group << AIRCR_PRIGROUP_SHIFT);
        SCB_AIRCR = aircr;
    }

    void configureNesting(uint8_t priority, PriorityGroup group = PriorityGroup::GROUP_16_0) {
        // Ensure valid priority
        if (priority > MAX_PRIORITY) priority = MAX_PRIORITY;
        
        // Set priority grouping first
        setPriorityGrouping(group);
        
        // Configure NVIC priority
        NVIC_SET_PRIORITY(_irqNumber, priority);
        
        // Enable nested interrupts
        __enable_irq();
    }
    
    void handleNestedInterrupt() {
        // Higher priority interrupts can occur here
        // Lower priority ones are automatically blocked
    }
};
```

### 3. Interrupt Masking

Sophisticated interrupt control mechanisms:

a) Global Masking
   - Complete interrupt disable/enable
   - Atomic operation support
   - Critical section protection
   - System-wide interrupt management

b) Individual Interrupt Control
   - Selective enabling/disabling
   - Per-peripheral interrupt management
   - Fine-grained control
   - Resource-specific protection

c) Priority-Based Masking
   - Mask by priority level
   - Basepri register usage
   - Selective priority blocking
   - Interrupt priority thresholding

d) Fault Masking
   - Configurable fault handling
   - Debug support
   - System protection features
   - Error recovery options

e) Implementation Patterns:

1. Priority-Based Masking Example:
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

2. Selective Interrupt Masking:
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

3. RAII-Style Critical Section:
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

## Implementation Guidelines

### 1. Keep ISRs Short and Fast

#### Teensyduino Approach:
```cpp
class SelectiveInterruptMask {
static volatile uint32_t _adcValue = 0;

private:
    void processADCValue(uint32_t value) {
        // Add your ADC value processing logic here
        // For example:
        Serial.printf("ADC Value: %lu\n", value);
    }

public:
    void setup() {
        attachInterrupt(digitalPinToInterrupt(pin), fastISR, RISING);
    }

    // Quick ISR - just capture data
    static void fastISR() {
        _adcValue = analogRead(A0);
    }

    // Process in main loop
    void loop() {
        if (_adcValue > 0) {
            processADCValue(_adcValue);
            _adcValue = 0;
        }
    }
};

volatile uint32_t SelectiveInterruptMask::_adcValue = 0;
```

#### Low-Level Approach:
```cpp
volatile uint32_t _adcValue = 0;

void setup() {
    // Direct port configuration for fastest response
    IOMUXC_SW_MUX_CTL_PAD_GPIO_EMC_31 = 5; // Configure pin 33 as GPIO
    IOMUXC_SW_PAD_CTL_PAD_GPIO_EMC_31 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    GPIO7_GDIR &= ~(1 << 31);  // Set as input
    attachInterruptVector(IRQ_GPIO6789, portaISR);
    GPIO7_IMR |= (1 << 31);    // Enable interrupt for pin
    GPIO7_ICR2 |= (0x2 << 30); // Rising edge interrupt
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

### 2. Use Appropriate Priority Levels

#### Teensyduino Approach:
```cpp
// Pin definitions
const int criticalPin = 2;    // Using pin 2 for critical interrupt
const int normalPin = 3;      // Using pin 3 for normal interrupt

volatile bool criticalFlag = false;
volatile bool normalFlag = false;

void criticalISR() {
    criticalFlag = true;
}

void normalISR() {
    normalFlag = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(criticalPin, INPUT_PULLUP);
    pinMode(normalPin, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(criticalPin), criticalISR, RISING);
    attachInterrupt(digitalPinToInterrupt(normalPin), normalISR, RISING);

    // Set interrupt priorities (0 = highest, 255 = lowest)
    NVIC_SET_PRIORITY(IRQ_GPIO6789, 64);  // Higher priority for critical pin
    NVIC_SET_PRIORITY(IRQ_GPIO1_0_15, 128);  // Lower priority for normal pin
}

```

#### Low-Level Approach:
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

### 3. Protect Shared Resources

#### Teensyduino Approach:
```cpp
// Using critical sections
volatile uint32_t _sharedData = 0;

void updateSharedData(uint32_t newValue) {
    noInterrupts();
    _sharedData = newValue;
    interrupts();
}

// Using mutex-like protection
class SharedResource {
private:
    volatile bool _locked = false;
    volatile uint32_t _data = 0;
    
public:
bool tryUpdate(uint32_t newValue) {
    noInterrupts();
    if (!_locked) {
        _locked = true;            // Lock the resource
        _data = newValue;
        _locked = false;           // Unlock the resource
        interrupts();
        return true;
    }
    interrupts();
    return false;
}
};
```


### 4. Handle Errors Gracefully (Low-Level Only Features)

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

## Common Use Cases

1. Timer Interrupts
   - Periodic tasks
   - Timeouts
   - PWM generation
   - Time-based sampling

2. Communication Interrupts
   - UART data reception
   - SPI transfer completion
   - I2C events
   - USB packet handling

3. External Interrupts
   - Button presses
   - Sensor triggers
   - External events
   - Emergency stops

4. System Interrupts
   - Memory management
   - Bus faults
   - Usage faults
   - System tick

## Advanced Features

1. Interrupt Chaining
   - Multiple ISRs for one interrupt
   - Priority-based execution
   - Dynamic ISR registration
   - Shared interrupt handling

2. FreeRTOS Integration
   - Interrupt priorities with RTOS
   - Critical sections in RTOS context
   - ISR-safe API functions
   - Deferred interrupt handling

3. DMA Integration
   - DMA interrupt coordination
   - Transfer completion handling
   - Error handling
   - Buffer management

4. Power Management
   - Wake-from-sleep handling
   - Low-power interrupt configuration
   - Clock domain considerations
   - Power mode transitions

## Example Patterns

### 1. Button Debouncing with Interrupts
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

### 2. Circular Buffer with Interrupt
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

### 3. Timer Interrupt Manager
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

### 4. Priority-based Interrupt Handler
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

## Command Reference

### Interrupt Control
```cpp
// Basic interrupt control
noInterrupts();                    // Disable all interrupts
interrupts();                      // Enable all interrupts

// Attach/detach interrupts
attachInterrupt(pin, ISR, mode);   // Attach interrupt to pin
detachInterrupt(pin);              // Detach interrupt from pin

// Priority control
NVIC_SET_PRIORITY(IRQ, priority);  // Set interrupt priority
NVIC_GET_PRIORITY(IRQ);            // Get interrupt priority

// Enable/disable specific interrupts
NVIC_ENABLE_IRQ(IRQ);             // Enable specific interrupt
NVIC_DISABLE_IRQ(IRQ);            // Disable specific interrupt

// Interrupt status
NVIC_IS_ENABLED(IRQ);             // Check if interrupt is enabled
NVIC_IS_PENDING(IRQ);             // Check if interrupt is pending
NVIC_IS_ACTIVE(IRQ);              // Check if interrupt is active
```

### Vector Table Operations
```cpp
// Attach/detach interrupt vectors
attachInterruptVector(IRQ_LPSPI4, myISR);      // Connect ISR to vector
detachInterruptVector(IRQ_LPSPI4);             // Disconnect ISR

// Common Teensy 4.1 vectors
IRQ_LPSPI4       // SPI4 interrupt vector
IRQ_LPUART6      // Serial6 interrupt vector
IRQ_GPIO6789     // GPIO6-9 pin interrupt vector
```

### Priority Management
The Teensy 4.1's Cortex-M7 supports priority levels 0-15 (0 = highest):

```cpp
// Setting priorities
NVIC_SET_PRIORITY(IRQ_LPSPI4, 2);   // High priority for SPI
NVIC_SET_PRIORITY(IRQ_GPIO6789, 8);  // Lower priority for GPIO

// Get current priority
uint8_t priority = NVIC_GET_PRIORITY(IRQ_LPSPI4);

// Priority-based interrupt nesting
void highPriorityISR() {
    // This can interrupt lowPriorityISR
    // Keep this handler fast!
}

void lowPriorityISR() {
    // This can be interrupted by highPriorityISR
    // Can do more work here
}
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

1. **Setting a Bit (Enable an Interrupt)**
   ```cpp
   // Using OR operation (|=)
   LPSPI4_IER |= LPSPI_IER_TCIE;  // Enable Transfer Complete Interrupt
   // OR
   LPSPI4_IER |= (1 << 10);       // Same thing using bit shift
   ```

2. **Clearing a Bit (Disable an Interrupt)**
   ```cpp
   // Using AND operation with inverted bit (&= ~)
   LPSPI4_IER &= ~LPSPI_IER_TCIE;  // Disable Transfer Complete Interrupt
   // OR
   LPSPI4_IER &= ~(1 << 10);       // Same thing using bit shift
   ```

3. **Checking if a Bit is Set**
   ```cpp
if (LPSPI4_IER & LPSPI_IER_TCIE) {
       // Transfer Complete Interrupt is enabled
   }
   ```

4. **Setting Multiple Bits**
   ```cpp
   // Enable multiple interrupts at once
   LPSPI4_IER |= (LPSPI_IER_TCIE | LPSPI_IER_RDIE);
   ```

5. **Clearing Multiple Bits**
   ```cpp
   // Disable multiple interrupts at once
   LPSPI4_IER &= ~(LPSPI_IER_TCIE | LPSPI_IER_RDIE);
   ```

### Best Practices

1. **Atomic Operations**: When possible, use single operations to modify bits to avoid race conditions.
   ```cpp
   // Good - single atomic operation
   LPSPI4_IER |= (LPSPI_IER_TCIE | LPSPI_IER_RDIE);
   
   // Bad - multiple operations could be interrupted
   LPSPI4_IER |= LPSPI_IER_TCIE;
   LPSPI4_IER |= LPSPI_IER_RDIE;
   ```

2. **Clear Pending Interrupts**: When enabling an interrupt, clear any pending interrupts first.
   ```cpp
   LPSPI4_SR = LPSPI_SR_TCF;     // Clear pending interrupt flag
   LPSPI4_IER |= LPSPI_IER_TCIE; // Then enable the interrupt
   ```

3. **Disable Before Configuration**: Disable interrupts before changing critical configurations.
   ```cpp
   uint32_t temp = LPSPI4_IER;        // Save current state
   LPSPI4_IER &= ~LPSPI_IER_TCIE;     // Disable interrupt
   // ... perform configuration ...
   LPSPI4_IER = temp;                  // Restore previous state
   ```

### Critical Section Management
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

### Interrupt Safety Guidelines
1. Keep ISRs as short as possible
2. Use volatile for shared variables
3. Avoid complex operations in ISRs
4. Be careful with nested interrupts
5. Use appropriate priorities

## Additional Resources

1. ARM Cortex-M7 Reference Manual
   - Detailed interrupt architecture
   - NVIC programming model
   - Exception handling
   - Priority configuration

2. Teensy 4.1 Technical Reference
   - Interrupt vector table
   - Pin interrupt mapping
   - Peripheral interrupt sources
   - System configuration

3. IMXRT1062 Reference Manual
   - Interrupt controller details
   - DMA interrupt integration
   - Power management
   - Clock configuration

These guidelines ensure:
- Reliable interrupt handling
- Minimal latency
- Safe resource sharing
- Proper error recovery
- Optimal performance
- System stability
- Maintainable code