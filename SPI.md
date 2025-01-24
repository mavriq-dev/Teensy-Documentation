# Teensy 4.1 SPI
Copyright &copy; 2025 Geoff Van Brunt

This documentation is licensed under the MIT License.

Permission is hereby granted, free of charge, to any person obtaining a copy of this documentation and associated files, to deal in the documentation without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the documentation, and to permit persons to whom the documentation is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the documentation.

THE DOCUMENTATION IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE DOCUMENTATION OR THE USE OR OTHER DEALINGS IN THE DOCUMENTATION.
This document provides an overview of the Serial Peripheral Interface (SPI) functionality for the Teensy 4.1, covering both Teensyduino high-level APIs and necessary low-level operations.

## Hardware Overview

The IMXRT1060 features multiple LPSPI (Low Power SPI) modules:
- LPSPI1 through LPSPI4
- Each module supports master and slave modes
- DMA support for transmit and receive
- Multiple chip select lines
- Configurable clock polarity and phase
- Hardware FIFO buffers

## Basic SPI Setup

### Teensyduino API
```cpp
// Basic SPI device communication example
class SPIDevice {
private:
    const uint8_t _cs;     // Chip select pin
    const uint32_t _speed; // Communication speed in Hz
    const uint8_t _mode;   // SPI mode (0-3)

public:
    SPIDevice(uint8_t cs, uint32_t speed = 4000000, uint8_t mode = SPI_MODE0) 
        : _cs(cs), _speed(speed), _mode(mode) {
        pinMode(_cs, OUTPUT);
        digitalWriteFast(_cs, HIGH); // CS is active LOW
    }

    void begin() {
        SPI.begin();
    }

    // Single byte transfer
    uint8_t transfer(uint8_t data) {
        SPI.beginTransaction(SPISettings(_speed, MSBFIRST, _mode));
        digitalWriteFast(_cs, LOW);
        uint8_t received = SPI.transfer(data);
        digitalWriteFast(_cs, HIGH);
        SPI.endTransaction();
        return received;
    }

    // Multiple byte transfer
    void transfer(uint8_t* tx_data, uint8_t* rx_data, size_t len) {
        SPI.beginTransaction(SPISettings(_speed, MSBFIRST, _mode));
        digitalWriteFast(_cs, LOW);
        for(size_t i = 0; i < len; i++) {
            rx_data[i] = SPI.transfer(tx_data[i]);
        }
        digitalWriteFast(_cs, HIGH);
        SPI.endTransaction();
    }

    void end() {
        SPI.end();
    }
};
```

### Low-Level Setup (When Needed)
```cpp

class LowLevelSPI {
private:
    IMXRT_LPSPI_t* _port;
    uint8_t _cs;

public:
    LowLevelSPI(IMXRT_LPSPI_t* port = &IMXRT_LPSPI4_S, uint8_t cs = 10) 
        : _port(port), _cs(cs) {
        pinMode(_cs, OUTPUT);
        digitalWriteFast(_cs, HIGH);
    }

    void begin() {
        // Enable clock to SPI module
        CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
        
        // Reset module
        _port->CR = LPSPI_CR_RST;
        _port->CR = 0;
        
        // Configure module
        _port->CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_SAMPLE;
        _port->TCR = LPSPI_TCR_FRAMESZ(7);  // 8 bits
        
        // Set clock divider for desired speed
        _port->CCR = LPSPI_CCR_SCKDIV(11);  // ~4MHz

        // Enable module
        _port->CR = LPSPI_CR_MEN | LPSPI_CR_RRF | LPSPI_CR_RTF;
    }

    // Wait for transmit FIFO to have space
    inline void waitForTxSpace() {
        while (!(_port->SR & LPSPI_SR_TDF)) { }
    }

    // Wait for receive FIFO to have data
    inline void waitForRxData() {
        while (!(_port->SR & LPSPI_SR_RDF)) { }
    }

    // Single byte transfer with direct register access
    uint8_t transfer(uint8_t data) {
        digitalWriteFast(_cs, LOW);
        
        // Clear any previous flags
        _port->SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
        
        // Wait for TX FIFO space and send
        waitForTxSpace();
        _port->TDR = data;
        
        // Wait for transfer to complete
        while (!(_port->SR & LPSPI_SR_TCF)) { }
        
        // Read received data
        waitForRxData();
        uint8_t received = _port->RDR;
        
        digitalWriteFast(_cs, HIGH);
        return received;
    }

    // Multiple byte transfer using FIFO
    void transfer(uint8_t* tx_data, uint8_t* rx_data, size_t len) {
        digitalWriteFast(_cs, LOW);
        
        // Clear flags
        _port->SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
        
        size_t tx_count = 0;
        size_t rx_count = 0;
        
        while (rx_count < len) {
            // Fill TX FIFO while there's space and data to send
            while ((tx_count < len) && (_port->FSR & 0x7)) {
                _port->TDR = tx_data[tx_count++];
            }
            
            // Read from RX FIFO while there's data
            while ((_port->FSR & (0x7 << 16)) && (rx_count < len)) {
                rx_data[rx_count++] = _port->RDR;
            }
        }
        
        // Wait for all transfers to complete
        while (!(_port->SR & LPSPI_SR_TCF)) { }
        
        digitalWriteFast(_cs, HIGH);
    }

    void end() {
        _port->CR = 0; // Disable module
    }
};
```

## Advanced Features

### 1. DMA Support

#### Teensyduino with DMA
```cpp
class TeensySPIDMA {
private:
    SPIClass& _spi;
    DMAChannel _dmaTX;
    DMAChannel _dmaRX;
    DMAMEM static uint8_t _txBuffer[256];
    DMAMEM static uint8_t _rxBuffer[256];
    IMXRT_LPSPI_t* _lpspi4 = (IMXRT_LPSPI_t*)&IMXRT_LPSPI4;

public:
    TeensySPIDMA(SPIClass& spi = SPI) : _spi(spi) {}

    void begin() {
        _spi.begin();
        _spi.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));

        _dmaTX.begin();
        _dmaTX.destination((volatile uint32_t&)_lpspi4->TDR);
        _dmaTX.sourceBuffer(_txBuffer, sizeof(_txBuffer));
        _dmaTX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_TX);
        
        _dmaRX.begin();
        _dmaRX.source((volatile uint32_t&)_lpspi4->RDR);
        _dmaRX.destinationBuffer(_rxBuffer, sizeof(_rxBuffer));
        _dmaRX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_RX);
        
        _lpspi4->DER |= (LPSPI_DER_TDDE | LPSPI_DER_RDDE);
    }
};

DMAMEM uint8_t TeensySPIDMA::_txBuffer[256];
DMAMEM uint8_t TeensySPIDMA::_rxBuffer[256];
```

#### Low-Level DMA (When Needed)
```cpp
class LowLevelSPIDMA {
private:
    IMXRT_LPSPI_t* _port;
    DMAChannel _dmaTX;
    DMAChannel _dmaRX;
    uint8_t _cs;
    static volatile bool _transferComplete;
    
    // DMA buffers must be in DMAMEM for Teensy 4.1
    DMAMEM static uint8_t _txBuffer[256];
    DMAMEM static uint8_t _rxBuffer[256];
    
public:
    LowLevelSPIDMA(IMXRT_LPSPI_t* port = &IMXRT_LPSPI4_S, uint8_t cs = 10) 
        : _port(port), _cs(cs) {
        pinMode(_cs, OUTPUT);
        digitalWriteFast(_cs, HIGH);
    }

    void begin() {
        // Enable clock to SPI module
        CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
        
        // Reset module
        _port->CR = LPSPI_CR_RST;
        _port->CR = 0;
        
        // Configure module
        _port->CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_SAMPLE;
        _port->TCR = LPSPI_TCR_FRAMESZ(7);  // 8 bits
        _port->CCR = LPSPI_CCR_SCKDIV(11);  // ~4MHz
        
        // Enable DMA requests in SPI module
        _port->DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE;
        
        // Initialize DMA channels
        _dmaTX.begin(true); // Allocate channel
        _dmaRX.begin(true);
        
        // Configure TX DMA
        _dmaTX.destination((volatile uint32_t&)_port->TDR);
        _dmaTX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_TX);
        _dmaTX.disableOnCompletion();
        _dmaTX.interruptAtCompletion();
        _dmaTX.attachInterrupt(dmaCallback);
        
        // Configure RX DMA
        _dmaRX.source((volatile uint32_t&)_port->RDR);
        _dmaRX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_RX);
        _dmaRX.disableOnCompletion();
        
        // Enable module
        _port->CR = LPSPI_CR_MEN;
    }
    
    // Start a DMA transfer
    bool startTransfer(const uint8_t* txData, size_t len) {
        if (len > sizeof(_txBuffer)) return false;
        
        // Copy data to DMA buffer
        memcpy(_txBuffer, txData, len);
        
        digitalWriteFast(_cs, LOW);
        _transferComplete = false;
        
        // Clear any previous flags
        _port->SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
        
        // Configure DMA transfers
        _dmaRX.destinationBuffer(_rxBuffer, len);
        _dmaTX.sourceBuffer(_txBuffer, len);
        
        // Start transfers - RX first, then TX
        _dmaRX.enable();
        _dmaTX.enable();
        
        return true;
    }
    
    // Wait for transfer to complete and get received data
    bool getReceivedData(uint8_t* rxData, size_t len) {
        if (!rxData || len > sizeof(_rxBuffer)) return false;
        
        // Wait for transfer to complete
        while (!_transferComplete) { yield(); }
        
        // Ensure DMA is really done
        while (!_dmaTX.complete() || !_dmaRX.complete()) { yield(); }
        
        // Wait for SPI module to be idle
        while (!(_port->SR & LPSPI_SR_TCF)) { yield(); }
        
        // Copy received data
        memcpy(rxData, _rxBuffer, len);
        
        digitalWriteFast(_cs, HIGH);
        return true;
    }
    
    // Combined transfer function
    bool transfer(const uint8_t* txData, uint8_t* rxData, size_t len) {
        if (!startTransfer(txData, len)) return false;
        return getReceivedData(rxData, len);
    }
    
private:
    static void dmaCallback() {
        _transferComplete = true;
    }
    
    void end() {
        _dmaTX.disable();
        _dmaRX.disable();
        _port->CR = 0; // Disable module
    }
};

// Define static DMA buffers
DMAMEM uint8_t LowLevelSPIDMA::_txBuffer[256];
DMAMEM uint8_t LowLevelSPIDMA::_rxBuffer[256];
volatile bool LowLevelSPIDMA::_transferComplete = false;
```

### 2. Interrupt Handling

#### Teensyduino Approach
```cpp
class TeensySPIInterrupt {
private:
    SPIClass& _spi;
    static volatile bool _transferComplete;
    uint8_t _cs;
    static constexpr uint32_t SPI_CLOCK = 4000000; // 4MHz

public:
    TeensySPIInterrupt(SPIClass& spi = SPI, uint8_t cs = 10) 
        : _spi(spi), _cs(cs) {
        pinMode(_cs, OUTPUT);
        digitalWriteFast(_cs, HIGH);
    }

    void begin() {
        _spi.begin();
        _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
        _spi.usingInterrupt(IRQ_LPSPI4);
        attachInterruptVector(IRQ_LPSPI4, spiISR);
        NVIC_ENABLE_IRQ(IRQ_LPSPI4);
    }

    bool transfer(const uint8_t* txData, uint8_t* rxData, size_t len) {
        if (!txData || !rxData) return false;
        
        _transferComplete = false;
        digitalWriteFast(_cs, LOW);
        
        // Enable TCF (Transfer Complete Flag) interrupt
        LPSPI4_IER |= LPSPI_IER_TCIE;
        
        for(size_t i = 0; i < len; i++) {
            // Start transfer
            LPSPI4_TDR = txData[i];
            
            // Wait for transfer to complete
            while(!_transferComplete) { yield(); }
            _transferComplete = false;
            
            // Read received data
            rxData[i] = LPSPI4_RDR;
        }
        
        // Disable TCF interrupt
        LPSPI4_IER &= ~LPSPI_IER_TCIE;
        digitalWriteFast(_cs, HIGH);
        return true;
    }

    void end() {
        _spi.endTransaction();
        NVIC_DISABLE_IRQ(IRQ_LPSPI4);
    }

private:
    static void spiISR() {
        uint32_t sr = LPSPI4_SR;
        if (sr & LPSPI_SR_TCF) {
            _transferComplete = true;
            LPSPI4_SR = LPSPI_SR_TCF;  // Clear the flag
        }
    }
};

// Define static member
volatile bool TeensySPIInterrupt::_transferComplete = false;
```

#### Low-Level Interrupts
```cpp
class LowLevelSPIInterrupt {
private:
    IMXRT_LPSPI_t* _port;
    
public:
    void begin() {
        // Enable transmit complete interrupt
        _port->IER = LPSPI_IER_TCIE;
        
        // Configure interrupt priority
        NVIC_SET_PRIORITY(IRQ_LPSPI4, 64);
        NVIC_ENABLE_IRQ(IRQ_LPSPI4);
    }
};
```

### 3. FIFO Management

#### Teensyduino FIFO Access
```cpp

class TeensySPIFIFO {
private:
    SPIClass& _spi;
    uint8_t _cs;
    static constexpr uint32_t SPI_CLOCK = 4000000; // 4MHz
    static constexpr uint8_t FIFO_SIZE = 4;        // LPSPI FIFO depth is 4

public:
    TeensySPIFIFO(SPIClass& spi = SPI, uint8_t cs = 10) 
        : _spi(spi), _cs(cs) {
        pinMode(_cs, OUTPUT);
        digitalWriteFast(_cs, HIGH);
    }

    void begin() {
        _spi.begin();
        _spi.beginTransaction(SPISettings(SPI_CLOCK, MSBFIRST, SPI_MODE0));
    }

    void end() {
        _spi.endTransaction();
    }

    // Wait for space in transmit FIFO
    void waitForTxSpace() {
        while (!(LPSPI4_SR & LPSPI_SR_TDF)) { }  // Wait for Transmit Data Flag
    }

    // Wait for data in receive FIFO
    void waitForRxData() {
        while (!(LPSPI4_SR & LPSPI_SR_RDF)) { }  // Wait for Receive Data Flag
    }

    // Transfer multiple bytes using FIFO
    bool transfer(const uint8_t* txData, uint8_t* rxData, size_t len) {
        if (!txData || !rxData) return false;
        
        digitalWriteFast(_cs, LOW);
        
        size_t txCount = 0;
        size_t rxCount = 0;
        
        // Initial FIFO fill
        while (txCount < len && txCount < FIFO_SIZE) {
            waitForTxSpace();
            LPSPI4_TDR = txData[txCount++];
        }
        
        // Process remaining data
        while (rxCount < len) {
            // Fill TX FIFO when space available and data remains
            if (txCount < len && (LPSPI4_SR & LPSPI_SR_TDF)) {
                LPSPI4_TDR = txData[txCount++];
            }
            
            // Read RX FIFO when data available
            if (LPSPI4_SR & LPSPI_SR_RDF) {
                rxData[rxCount++] = LPSPI4_RDR;
            }
        }
        
        digitalWriteFast(_cs, HIGH);
        return true;
    }

    // Simple single byte transfer
    uint8_t transfer(uint8_t data) {
        digitalWriteFast(_cs, LOW);
        
        waitForTxSpace();
        LPSPI4_TDR = data;
        waitForRxData();
        uint8_t received = LPSPI4_RDR;
        
        digitalWriteFast(_cs, HIGH);
        return received;
    }
};

// Example usage
void setup() {
    Serial.begin(115200);
    while (!Serial) ; // Wait for Serial
    
    TeensySPIFIFO spi;
    spi.begin();
    
    // Single byte transfer example
    uint8_t result = spi.transfer(0x42);
    Serial.printf("Single byte transfer result: 0x%02X\n", result);
    
    // Multiple byte transfer example
    uint8_t txBuffer[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint8_t rxBuffer[8];
    
    if (spi.transfer(txBuffer, rxBuffer, 8)) {
        Serial.println("Multiple byte transfer results:");
        for (int i = 0; i < 8; i++) {
            Serial.printf("0x%02X ", rxBuffer[i]);
        }
        Serial.println();
    }
    
    spi.end();
}

void loop() {
    // Empty loop
}
```

#### Low-Level FIFO Control
```cpp
class LowLevelSPIFIFO {
private:
    IMXRT_LPSPI_t* _port;

public:
    void configureFIFO() {
        // Set watermarks
        _port->FCR = LPSPI_FCR_TXWATER(0) | LPSPI_FCR_RXWATER(0);
        
        // Wait for FIFO empty
        while (!(_port->SR & LPSPI_SR_TDF)) {}
    }
};
```

## SPI Configuration Details

### Bit Order

The bit order determines whether data is transmitted MSB (Most Significant Bit) or LSB (Least Significant Bit) first.

1. Available Options:
   - `MSBFIRST`: Most Significant Bit First (default)
   - `LSBFIRST`: Least Significant Bit First

2. Usage in Teensyduino:
```cpp
// In SPISettings constructor
SPISettings(4000000, MSBFIRST, SPI_MODE0);

// Or directly
SPI.setBitOrder(MSBFIRST);
```

3. Low-Level Configuration:
```cpp
// Set MSB first
IMXRT_LPSPI4_S.CFGR1 &= ~LPSPI_CFGR1_LSBF;

// Set LSB first
IMXRT_LPSPI4_S.CFGR1 |= LPSPI_CFGR1_LSBF;
```

4. Common Uses:
   - MSBFIRST: Most common, used by most SPI devices
   - LSBFIRST: Some displays, shift registers, and specialized devices

### Data Modes

SPI Data Modes define the clock polarity (CPOL) and clock phase (CPHA) relationship.

1. Available Modes:
```cpp
SPI_MODE0: CPOL=0, CPHA=0  // Clock idle low,  data sampled on rising edge
SPI_MODE1: CPOL=0, CPHA=1  // Clock idle low,  data sampled on falling edge
SPI_MODE2: CPOL=1, CPHA=0  // Clock idle high, data sampled on falling edge
SPI_MODE3: CPOL=1, CPHA=1  // Clock idle high, data sampled on rising edge
```

2. Mode Characteristics:
   - CPOL=0: Clock idles low
   - CPOL=1: Clock idles high
   - CPHA=0: Data sampled on first clock edge
   - CPHA=1: Data sampled on second clock edge

3. Usage in Teensyduino:
```cpp
// In SPISettings constructor
SPISettings settings(4000000, MSBFIRST, SPI_MODE0);

// Or directly
SPI.setDataMode(SPI_MODE0);
```

4. Low-Level Configuration:
```cpp
// Mode 0 (CPOL=0, CPHA=0)
IMXRT_LPSPI4_S.TCR &= ~(LPSPI_TCR_CPOL | LPSPI_TCR_CPHA);

// Mode 1 (CPOL=0, CPHA=1)
IMXRT_LPSPI4_S.TCR = (IMXRT_LPSPI4_S.TCR & ~LPSPI_TCR_CPOL) | LPSPI_TCR_CPHA;

// Mode 2 (CPOL=1, CPHA=0)
IMXRT_LPSPI4_S.TCR = (IMXRT_LPSPI4_S.TCR & ~LPSPI_TCR_CPHA) | LPSPI_TCR_CPOL;

// Mode 3 (CPOL=1, CPHA=1)
IMXRT_LPSPI4_S.TCR |= (LPSPI_TCR_CPOL | LPSPI_TCR_CPHA);
```

5. Common Device Modes:
   - Mode 0: Most common, used by SD cards, many sensors
   - Mode 1: Some displays, ADCs
   - Mode 2: Less common
   - Mode 3: Some displays (ST7735, ILI9341)

6. Timing Diagrams:
```
Mode 0 (CPOL=0, CPHA=0):
Clock:   ___‾‾‾___ 
Data:    ↑_____↑___  (Sample on rising edge)

Mode 1 (CPOL=0, CPHA=1):
Clock:   ___‾‾‾___
Data:    ___↑___↑_  (Sample on falling edge)

Mode 2 (CPOL=1, CPHA=0):
Clock:   ‾‾‾___‾‾‾
Data:    ↑_____↑___  (Sample on falling edge)

Mode 3 (CPOL=1, CPHA=1):
Clock:   ‾‾‾___‾‾‾
Data:    ___↑___↑_  (Sample on rising edge)
```

7. Important Considerations:
   - Always check device datasheet for required mode
   - Some devices support multiple modes
   - Mode must match between master and slave
   - Mode affects timing of chip select signals
   - Changing modes requires ending current transaction

## Best Practices

1. Clock Configuration
   - Use appropriate clock dividers for desired speed
   - Consider peripheral timing requirements
   - Account for signal integrity at higher speeds

2. DMA Usage
   - Use DMA for large transfers
   - Properly align buffers in memory
   - Handle completion and error interrupts

3. Error Handling
   - Check status flags
   - Implement timeout mechanisms
   - Reset module if needed

4. Performance Optimization
   - Use appropriate FIFO watermarks
   - Minimize interrupt overhead
   - Consider double buffering for continuous transfers

## Common Use Cases

1. Basic Communication
   - Single byte transfers
   - Command/response protocols
   - Register access

2. High-Speed Data Transfer
   - DMA-based transfers
   - Continuous streaming
   - Double buffering

3. Multi-Slave Management
   - Chip select handling
   - Different settings per slave
   - Shared bus arbitration

## Hardware-Specific Notes

1. LPSPI Module Features
   - 4 independent modules
   - 64-bit TX and RX FIFOs
   - Programmable delays
   - Multiple chip selects

2. Clock Configuration
   - PLL3 PFD2 as clock source
   - Configurable prescalers
   - Module-specific dividers

3. Pin Configuration
   - Multiple pin mapping options
   - Configurable pin characteristics
   - Input/output impedance control

## Command Reference

### Teensyduino SPI Commands

1. Basic Operations
```cpp
SPI.begin();                    // Initialize SPI
SPI.end();                      // Disable SPI
SPI.beginTransaction(settings); // Start transaction with settings
SPI.endTransaction();          // End current transaction
```

2. Data Transfer
```cpp
uint8_t  SPI.transfer(uint8_t data);             // Transfer single byte
uint16_t SPI.transfer16(uint16_t data);          // Transfer 16-bit word
void     SPI.transfer(void* buf, size_t count);  // Transfer buffer
```

3. Configuration
```cpp
// Create settings (speed in Hz)
SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode);

// Example settings
SPISettings(4000000, MSBFIRST, SPI_MODE0);  // 4MHz, MSB first, Mode 0
SPISettings(1000000, LSBFIRST, SPI_MODE3);  // 1MHz, LSB first, Mode 3

// Set individual parameters
SPI.setBitOrder(MSBFIRST);     // Set bit order
SPI.setDataMode(SPI_MODE0);    // Set data mode
SPI.setClockDivider(SPI_CLOCK_DIV4);  // Set clock divider
```

4. Hardware Access
```cpp
// Access hardware registers
IMXRT_LPSPI_t& hw = SPI.hardware();
uint32_t fifoSize = hw.PARAM & 0xFF;
bool isMaster = hw.CFGR1 & LPSPI_CFGR1_MASTER;

// DMA support
uint8_t txChannel = SPI.hardware().tx_dma_channel;
uint8_t rxChannel = SPI.hardware().rx_dma_channel;
```

### Low-Level Register Commands

1. Control Register (CR)
```cpp
IMXRT_LPSPI4_S.CR = LPSPI_CR_RST;     // Reset module
IMXRT_LPSPI4_S.CR = LPSPI_CR_MEN;     // Enable module
IMXRT_LPSPI4_S.CR = LPSPI_CR_RRF;     // Reset receive FIFO
IMXRT_LPSPI4_S.CR = LPSPI_CR_RTF;     // Reset transmit FIFO
```

2. Configuration Register (CFGR1)
```cpp
// Master mode, sample on SCK rising edge
IMXRT_LPSPI4_S.CFGR1 = LPSPI_CFGR1_MASTER | LPSPI_CFGR1_SAMPLE;

// Slave mode with other options
IMXRT_LPSPI4_S.CFGR1 = LPSPI_CFGR1_PINCFG(3) | LPSPI_CFGR1_OUTCFG;
```

3. Clock Configuration (CCR)
```cpp
// Set clock divider for desired speed
IMXRT_LPSPI4_S.CCR = LPSPI_CCR_SCKDIV(11) | // SCK divider
                     LPSPI_CCR_DBT(8) |      // Delay between transfers
                     LPSPI_CCR_PCSSCK(6);    // PCS to SCK delay
```

4. FIFO Control Register (FCR)
```cpp
// Set FIFO watermarks
IMXRT_LPSPI4_S.FCR = LPSPI_FCR_TXWATER(0) | // TX FIFO watermark
                     LPSPI_FCR_RXWATER(0);   // RX FIFO watermark
```

5. DMA Enable Register (DER)
```cpp
// Enable DMA requests
IMXRT_LPSPI4_S.DER = LPSPI_DER_TDDE |  // TX DMA request
                     LPSPI_DER_RDDE;    // RX DMA request
```

6. Status Register (SR) Flags
```cpp
// Status checks
bool txFifoFull   = IMXRT_LPSPI4_S.SR & LPSPI_SR_TDF;
bool rxFifoEmpty  = IMXRT_LPSPI4_S.SR & LPSPI_SR_RDF;
bool dataComplete = IMXRT_LPSPI4_S.SR & LPSPI_SR_TCF;
bool frameError   = IMXRT_LPSPI4_S.SR & LPSPI_SR_FEF;
```

7. Interrupt Enable Register (IER)
```cpp
// Enable specific interrupts
IMXRT_LPSPI4_S.IER = LPSPI_IER_TCIE |  // Transfer complete
                     LPSPI_IER_RDIE |  // RX data ready
                     LPSPI_IER_TDIE;   // TX data needed
```

### Common Register Combinations

1. High-Speed Master Configuration
```cpp
// Configure for 24MHz operation
void setupMaxSpeed() {
    CCM_CBCMR = (CCM_CBCMR & ~CCM_CBCMR_LPSPI_PODF_MASK) |
                CCM_CBCMR_LPSPI_PODF(0);     // Divide by 1
    
    IMXRT_LPSPI4_S.CCR = LPSPI_CCR_SCKDIV(1) |  // Min clock div
                         LPSPI_CCR_DBT(0) |      // No delay
                         LPSPI_CCR_PCSSCK(0);    // No CS delay
    
    // Minimal TCR settings
    IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7);  // 8-bit
}
```

2. DMA with Interrupts
```cpp
// Setup for DMA with completion interrupt
IMXRT_LPSPI4_S.DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE;
IMXRT_LPSPI4_S.IER = LPSPI_IER_TCIE;  // Interrupt on complete
IMXRT_LPSPI4_S.FCR = LPSPI_FCR_TXWATER(0) |    // Empty TX triggers
                     LPSPI_FCR_RXWATER(3);      // Full RX triggers
```

3. Slave Mode with Error Detection
```cpp
// Configure as slave with error detection
IMXRT_LPSPI4_S.CR = LPSPI_CR_RST;
IMXRT_LPSPI4_S.CFGR1 = LPSPI_CFGR1_PINCFG(3);  // 4-pin mode
IMXRT_LPSPI4_S.IER = LPSPI_IER_FEIE |   // Frame error
                     LPSPI_IER_REIE;     // RX error
```

### Clock Configuration Commands

1. Enable SPI Clock
```cpp
// Enable LPSPI4 clock
CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);

// Configure clock source
CCM_CBCMR = (CCM_CBCMR & ~(CCM_CBCMR_LPSPI_PODF_MASK |
            CCM_CBCMR_LPSPI_CLK_SEL_MASK)) |
            CCM_CBCMR_LPSPI_PODF(5) |     // Divide by 6
            CCM_CBCMR_LPSPI_CLK_SEL(3);   // PLL2
```

### Transfer Control Register (TCR) Settings

1. Frame Size and Format
```cpp
// 8-bit transfers
IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7);  // 8-bit (n-1)

// 16-bit transfers
IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(15); // 16-bit (n-1)

// Advanced format
IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7) |     // 8-bit
                     LPSPI_TCR_RXMSK |          // Ignore RX data
                     LPSPI_TCR_CONT |           // Continuous CS
                     LPSPI_TCR_BYSW;           // Byte swap
```

2. Chip Select Control
```cpp
// CS configuration
IMXRT_LPSPI4_S.TCR = LPSPI_TCR_PCS(0);      // Use PCS0 (CS0)
IMXRT_LPSPI4_S.TCR = LPSPI_TCR_PCS(3);      // Use PCS3 (CS3)
IMXRT_LPSPI4_S.TCR = LPSPI_TCR_CONT;        // Keep CS asserted
```

### FIFO Status and Management

1. FIFO Status Checks
```cpp
// Check FIFO counts
uint32_t txCount = (IMXRT_LPSPI4_S.FSR >> 16) & 0xFF;  // TX FIFO count
uint32_t rxCount = IMXRT_LPSPI4_S.FSR & 0xFF;          // RX FIFO count

// Check FIFO states
bool txFull = !(IMXRT_LPSPI4_S.SR & LPSPI_SR_TDF);    // TX FIFO full
bool rxEmpty = !(IMXRT_LPSPI4_S.SR & LPSPI_SR_RDF);   // RX FIFO empty
```

2. FIFO Reset
```cpp
// Reset FIFOs
IMXRT_LPSPI4_S.CR = LPSPI_CR_RTF;  // Reset TX FIFO
IMXRT_LPSPI4_S.CR = LPSPI_CR_RRF;  // Reset RX FIFO

// Clear all flags
IMXRT_LPSPI4_S.SR = LPSPI_SR_TCF | LPSPI_SR_FCF | 
                    LPSPI_SR_TEF | LPSPI_SR_REF;
```

### Error Handling and Recovery

1. Error Detection
```cpp
// Check for errors
bool frameError = IMXRT_LPSPI4_S.SR & LPSPI_SR_FEF;    // Frame Error
bool txError = IMXRT_LPSPI4_S.SR & LPSPI_SR_TEF;       // Transmit Error
bool rxError = IMXRT_LPSPI4_S.SR & LPSPI_SR_REF;       // Receive Error
bool dataMatch = IMXRT_LPSPI4_S.SR & LPSPI_SR_DMF;     // Data Match

// Clear error flags
IMXRT_LPSPI4_S.SR = LPSPI_SR_FEF | LPSPI_SR_TEF | LPSPI_SR_REF;
```

2. Error Recovery
```cpp
void recoverFromError() {
    // Disable module
    IMXRT_LPSPI4_S.CR = 0;

    // Reset module
    IMXRT_LPSPI4_S.CR = LPSPI_CR_RST;
    
    // Clear all flags
    IMXRT_LPSPI4_S.SR = 0xFFFFFFFF;
    
    // Reset FIFOs
    IMXRT_LPSPI4_S.CR = LPSPI_CR_RRF | LPSPI_CR_RTF;
    
    // Re-enable with original configuration
    IMXRT_LPSPI4_S.CR = LPSPI_CR_MEN;
}
```

### Performance Optimization

1. Maximum Speed Configuration
```cpp
// Configure for maximum speed (up to 24MHz)
void setupMaxSpeed() {
    CCM_CBCMR = (CCM_CBCMR & ~CCM_CBCMR_LPSPI_PODF_MASK) |
                CCM_CBCMR_LPSPI_PODF(0);     // Divide by 1
    
    IMXRT_LPSPI4_S.CCR = LPSPI_CCR_SCKDIV(1) |  // Min clock div
                         LPSPI_CCR_DBT(0) |      // No delay
                         LPSPI_CCR_PCSSCK(0);    // No CS delay
    
    // Minimal TCR settings
    IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7);  // 8-bit
}
```

2. Optimized FIFO Usage
```cpp
// Configure for optimal FIFO performance
void optimizeFIFO() {
    // Set watermarks for maximum throughput
    IMXRT_LPSPI4_S.FCR = LPSPI_FCR_TXWATER(0) |    // Empty TX triggers
                         LPSPI_FCR_RXWATER(3);      // Full RX triggers
    
    // Enable DMA for large transfers
    IMXRT_LPSPI4_S.DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE;
}
```


### Timing Configurations

1. Delay Settings
```cpp
// Configure inter-transfer timing
IMXRT_LPSPI4_S.CCR = LPSPI_CCR_SCKDIV(1) |   // Clock divider
                     LPSPI_CCR_DBT(8) |       // Delay between transfers
                     LPSPI_CCR_PCSSCK(6) |    // PCS to SCK delay
                     LPSPI_CCR_SCKPCS(6);     // SCK to PCS delay

// Minimum timing for maximum speed
IMXRT_LPSPI4_S.CCR = LPSPI_CCR_SCKDIV(1) |   // Minimum divider
                     LPSPI_CCR_DBT(0) |       // No delays
                     LPSPI_CCR_PCSSCK(0) |
                     LPSPI_CCR_SCKPCS(0);
```

2. Clock Phase and Polarity
```cpp
// Mode 0 (CPOL=0, CPHA=0)
IMXRT_LPSPI4_S.CFGR1 &= ~(LPSPI_CFGR1_OUTCFG | LPSPI_CFGR1_PINCFG(3));
IMXRT_LPSPI4_S.TCR &= ~(LPSPI_TCR_CPOL | LPSPI_TCR_CPHA);

// Mode 3 (CPOL=1, CPHA=1)
IMXRT_LPSPI4_S.TCR |= LPSPI_TCR_CPOL | LPSPI_TCR_CPHA;
```

### Debug Features

1. Status Monitoring
```cpp
// Monitor all status flags
uint32_t getFullStatus() {
    uint32_t status = IMXRT_LPSPI4_S.SR;
    Serial.printf("Status: 0x%08X\n", status);
    Serial.printf("  Module Busy: %d\n", (status & LPSPI_SR_MBF) ? 1 : 0);
    Serial.printf("  TX Empty: %d\n", (status & LPSPI_SR_TDF) ? 1 : 0);
    Serial.printf("  RX Full: %d\n", (status & LPSPI_SR_RDF) ? 1 : 0);
    Serial.printf("  Transfer Complete: %d\n", (status & LPSPI_SR_TCF) ? 1 : 0);
    return status;
}
```

2. FIFO Monitoring
```cpp
// Monitor FIFO levels
void monitorFIFO() {
    uint32_t fsr = IMXRT_LPSPI4_S.FSR;
    uint32_t txCount = (fsr >> 16) & 0xFF;
    uint32_t rxCount = fsr & 0xFF;
    
    Serial.printf("TX FIFO: %d/4\n", txCount);
    Serial.printf("RX FIFO: %d/4\n", rxCount);
}
```

3. Debug Mode Configuration
```cpp
// Enable debug mode (halt on debug)
IMXRT_LPSPI4_S.CFGR1 |= LPSPI_CFGR1_DBGEN;

// Configure for easy debugging
void setupDebugMode() {
    // Slower clock for easier signal viewing
    IMXRT_LPSPI4_S.CCR = LPSPI_CCR_SCKDIV(19);  // Slower clock
    
    // Enable all error interrupts
    IMXRT_LPSPI4_S.IER = LPSPI_IER_TCIE |   // Transfer complete
                         LPSPI_IER_TEIE |   // TX error
                         LPSPI_IER_REIE |   // RX error
                         LPSPI_IER_FEIE;    // Frame error
}
```

### References

1. IMXRT1060 Reference Manual
   - Chapter 48: LPSPI
   - DMA chapter for peripheral triggers
   - Clock configuration chapter

2. Teensyduino SPI Library
   - SPIClass implementation
   - Hardware definitions
   - DMA support functions
