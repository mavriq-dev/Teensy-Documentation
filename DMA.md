# Teensy 4.1 DMA

Copyright &copy; 2025 Geoff Van Brunt

This documentation is licensed under the MIT License.

Permission is hereby granted, free of charge, to any person obtaining a copy of this documentation and associated files, to deal in the documentation without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the documentation, and to permit persons to whom the documentation is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the documentation.

THE DOCUMENTATION IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE DOCUMENTATION OR THE USE OR OTHER DEALINGS IN THE DOCUMENTATION.

## Introduction
This document provides an overview of the Direct Memory Access (DMA) functionality for the IMXRT1060 processor used in the Teensy 4.1.

## Basic DMA Components

- **Source Address Register (SAR)**: Holds the source address for data transfer
- **Destination Address Register (DAR)**: Holds the destination address for data transfer
- **Transfer Count (BCR)**: Byte Count Register that determines amount of data to transfer
- **Control Register (DCR)**: Controls DMA operation parameters

## Key Features

- 32 independent DMA channels
- Supports multiple transfer sizes:
  - 8-bit
  - 16-bit
  - 32-bit
- Automatic address increment capability
- Transfer completion interrupt support

## Basic Setup Process

1. Configure DMA channel
2. Set up source and destination addresses
3. Configure transfer size and count
4. Enable the channel
5. Initiate transfer (via software trigger or hardware request)

## Detailed Setup Process

### 1. Channel Configuration
- Select an available DMA channel (0-31)
- Configure the channel priority level
- Set the channel multiplexing (if required)
- Configure transfer attributes:
  - Transfer size (8/16/32-bit)
  - Address increment options
  - Burst size (if applicable)

### 2. Address Configuration
- Source Address (SAR):
  - Must be aligned according to transfer size
  - For peripherals: Use peripheral register address
  - For memory: Use buffer address
- Destination Address (DAR):
  - Similar alignment requirements as SAR
  - Can be fixed (for peripheral registers)
  - Can auto-increment for memory buffers

### 3. Transfer Configuration
- Set the Byte Count Register (BCR):
  - Number of bytes to transfer
  - Must be multiple of transfer size
- Configure transfer type:
  - Single transfer
  - Burst transfer
  - Peripheral-to-memory
  - Memory-to-peripheral
  - Memory-to-memory

### 4. Channel Enable
- Clear any pending error flags
- Enable error interrupts if needed
- Enable completion interrupts if needed
- Set channel enable bit
- Configure channel request source:
  - Software trigger
  - Hardware trigger from peripheral

### 5. Transfer Initiation
- For software trigger:
  - Set the start bit in control register
- For hardware trigger:
  - Configure peripheral to generate DMA requests
  - DMA will start automatically on peripheral request

### 6. Transfer Monitoring
- Check transfer status:
  - Done flag
  - Error flags
  - Bytes remaining
- Handle completion interrupt if enabled
- Clear status flags after handling

### 7. Error Handling
- Monitor error conditions:
  - Configuration errors
  - Bus errors
  - Transfer errors
- Implement error recovery:
  - Reset channel
  - Clear error flags
  - Reconfigure if necessary

## Implementation Considerations

### Memory Requirements
- Proper memory alignment
- Buffer boundary awareness
- Transfer size limitations
- Channel priority management

### Teensy 4.1 Implementation
When implementing DMA on Teensy 4.1:
1. Utilize the DMAChannel class
2. Configure source and destination
3. Set appropriate transfer size
4. Enable interrupts if needed
5. Initiate the transfer

## Notes
- This overview covers regular DMA mode only, not extended mode
- Always refer to the IMXRT1060 Reference Manual for detailed specifications
- Consider memory alignment requirements for optimal performance

## Basic DMA Examples

### 1. Simple Memory-to-Memory Transfer

#### Teensyduino Approach:
```cpp
class TeensyDMATransfer {
private:
    static DMAChannel _dma;
    static volatile bool _transferComplete;
    volatile bool _error;
    
    // Source and destination buffers in DMA-accessible memory
    DMAMEM static uint32_t _sourceBuffer[256];
    DMAMEM static uint32_t _destBuffer[256];

public:
    TeensyDMATransfer() : _error(false) {}

    void begin() {
        // Initialize source buffer with some test data
        for (int i = 0; i < 256; i++) {
            _sourceBuffer[i] = i;  // Fill with incrementing values
        }
        memset(_destBuffer, 0, sizeof(_destBuffer));  // Clear destination

        _dma.begin(true);  // Initialize DMA channel with force parameter
        
        // Configure the DMA transfer
        _dma.sourceBuffer(_sourceBuffer, sizeof(_sourceBuffer));
        _dma.destinationBuffer(_destBuffer, sizeof(_destBuffer));  // Set destination buffer
        _dma.transferSize(4);  // 32-bit transfers
        _dma.transferCount(sizeof(_sourceBuffer) / 4);  // Number of 32-bit transfers
        
        // Enable interrupt on completion
        _dma.attachInterrupt(transferComplete);
        _dma.interruptAtCompletion();  // Interrupt when done
        
        _transferComplete = false;
        _error = false;
    }

    // Start the DMA transfer
    void startTransfer() {
        if (!_dma.error()) {
            _dma.enable();
            Serial.println("DMA transfer started");
        } else {
            _error = true;
            Serial.println("DMA configuration error!");
        }
    }

    // Check if transfer is complete
    bool isComplete() const {
        return _transferComplete;
    }

    // Check if there was an error
    bool hasError() const {
        return _error;
    }

    // Get source data
    uint32_t getSourceValue(size_t index) const {
        if (index < 256) {
            return _sourceBuffer[index];
        }
        return 0;
    }

    // Get transferred data
    uint32_t getDestValue(size_t index) const {
        if (index < 256) {
            return _destBuffer[index];
        }
        return 0;
    }

private:
    static void transferComplete() {
        // Toggle LED to show completion
        digitalToggleFast(LED_BUILTIN);
        
        // Clear the interrupt flag
        _dma.clearInterrupt();
        
        _transferComplete = true;
        Serial.println("DMA transfer complete!");
        
        // Verify transfer
        bool verifyError = false;
        for (int i = 0; i < 256; i++) {
            if (_destBuffer[i] != _sourceBuffer[i]) {
                verifyError = true;
                Serial.printf("Verification failed at index %d: expected %lu, got %lu\n", 
                            i, _sourceBuffer[i], _destBuffer[i]);
                break;
            }
        }
        
        if (!verifyError) {
            Serial.println("Data verification successful!");
        }
    }
};

// Static member definitions
DMAChannel TeensyDMATransfer::_dma;
volatile bool TeensyDMATransfer::_transferComplete = false;
```

#### Low-Level Approach:
```cpp
// DMA Register definitions for IMXRT1062 (Teensy 4.1)
#define DMA_TCD_SADDR(n)             (*(volatile uint32_t*)(0x400E9000 + 0x1000 * n))
#define DMA_TCD_DADDR(n)             (*(volatile uint32_t*)(0x400E9008 + 0x1000 * n))
#define DMA_TCD_ATTR(n)              (*(volatile uint16_t*)(0x400E9004 + 0x1000 * n))
#define DMA_TCD_NBYTES_MLNO(n)       (*(volatile uint32_t*)(0x400E9010 + 0x1000 * n))
#define DMA_TCD_CITER_ELINKNO(n)     (*(volatile uint16_t*)(0x400E9016 + 0x1000 * n))
#define DMA_TCD_BITER_ELINKNO(n)     (*(volatile uint16_t*)(0x400E901E + 0x1000 * n))
#define DMA_TCD_CSR(n)               (*(volatile uint16_t*)(0x400E901C + 0x1000 * n))
#define DMAMUX_CHCFG(n)              (*(volatile uint32_t*)(0x400EC000 + 4 * n))
#define DMA_SERQ                     (*(volatile uint8_t *)0x400E100D)
#define DMA_CINT                     (*(volatile uint8_t *)0x400E100C)

// DMA Attribute register bit definitions
#define DMA_TCD_ATTR_SSIZE(n)        (((n) & 0x7) << 0)
#define DMA_TCD_ATTR_DSIZE(n)        (((n) & 0x7) << 3)

// DMA CSR bits
#define DMA_TCD_CSR_INTMAJOR        0x0002

// CCM register for clock gating
#define CCM_CCGR_ON                  0x03
#define CCM_CCGR5_DMA(n)            ((uint32_t)(n) << 6)


class LowLevelDMATransfer {
private:
    static constexpr uint8_t DMA_CHANNEL = 0;
    DMAMEM static uint32_t _source[256] __attribute__((aligned(32)));
    DMAMEM static uint32_t _dest[256] __attribute__((aligned(32)));

public:
    void begin() {
        // Enable DMA clock
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
        
        // Configure DMA MUX
        DMAMUX_CHCFG(DMA_CHANNEL) = 0;
        
        // Setup TCD (Transfer Control Descriptor)
        DMA_TCD_SADDR(DMA_CHANNEL) = (uint32_t)_source;
        DMA_TCD_DADDR(DMA_CHANNEL) = (uint32_t)_dest;
        DMA_TCD_ATTR(DMA_CHANNEL) = DMA_TCD_ATTR_SSIZE(2) | // 32-bit source
                                   DMA_TCD_ATTR_DSIZE(2);    // 32-bit dest
        DMA_TCD_NBYTES_MLNO(DMA_CHANNEL) = 4;  // 4 bytes per transfer
        DMA_TCD_CITER_ELINKNO(DMA_CHANNEL) = sizeof(_source) / 4;
        DMA_TCD_BITER_ELINKNO(DMA_CHANNEL) = sizeof(_source) / 4;
        
        // Enable interrupt
        DMA_TCD_CSR(DMA_CHANNEL) = DMA_TCD_CSR_INTMAJOR;
        attachInterruptVector(static_cast<IRQ_NUMBER_t>(IRQ_DMA_CH0 + DMA_CHANNEL), dmaISR);
        NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + DMA_CHANNEL);
        
        // Start transfer
        DMA_SERQ = DMA_CHANNEL;
    }

private:
    static void dmaISR() {
        // Clear interrupt flag
        DMA_CINT = DMA_CHANNEL;
        // Handle completion
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
    }
};
```

### 2. ADC with DMA

#### Teensyduino Approach:
```cpp
class TeensyADCDMA {
private:
    DMAChannel _dma;
    ADC* _adc;
    DMAMEM static uint16_t _adcBuffer[1024];
    static volatile bool _bufferFull;
    const uint8_t _adcPin;

public:
    TeensyADCDMA(uint8_t pin) : _adcPin(pin) {
        _adc = new ADC();
        _bufferFull = false;
    }

    void begin() {
        // Configure ADC
        _adc->setAveraging(4, ADC_0);
        _adc->setResolution(12, ADC_0);
        _adc->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED, ADC_0);
        _adc->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED, ADC_0);
        
        // Configure DMA
        _dma.begin(true); // Force hardware allocation
        _dma.source((volatile uint16_t&)ADC1_R0);  // ADC result register
        _dma.destinationBuffer(_adcBuffer, sizeof(_adcBuffer));
        _dma.transferSize(2);   // 16-bit transfers
        _dma.transferCount(sizeof(_adcBuffer) / 2);
        _dma.attachInterrupt(dmaCallback);
        
        // Setup hardware trigger from ADC
        _dma.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
        
        // Enable circular buffer mode
        _dma.enable();
        
        // Start ADC with DMA
        _adc->enableDMA(ADC_0);
        _adc->startContinuous(_adcPin, ADC_0);
    }

private:
    static void dmaCallback() {
        _bufferFull = true;
        // Process buffer here or set flag for main loop
    }
};

// Initialize static members
uint16_t TeensyADCDMA::_adcBuffer[1024];
volatile bool TeensyADCDMA::_bufferFull;
```

#### Low-Level Approach:
```cpp
// ADC Register definitions for IMXRT1062 (Teensy 4.1)
#define ADC1_HC0                (*(volatile uint32_t *)0x400C4000)
#define ADC1_CFG               (*(volatile uint32_t *)0x400C4014)
#define ADC1_GC                (*(volatile uint32_t *)0x400C4020)
#define ADC1_R0                (*(volatile uint32_t *)0x400C4048)

// DMA Register definitions for IMXRT1062
#define DMA_BASE               0x400E8000
#define DMAMUX_BASE           0x400EC000

// DMA Channel Transfer Control Descriptor (TCD)
#define DMA_TCD_SADDR(n)       (*(volatile uint32_t *)(DMA_BASE + 0x1000 * (n) + 0x000))
#define DMA_TCD_SOFF(n)        (*(volatile uint16_t *)(DMA_BASE + 0x1000 * (n) + 0x004))
#define DMA_TCD_ATTR(n)        (*(volatile uint16_t *)(DMA_BASE + 0x1000 * (n) + 0x006))
#define DMA_TCD_NBYTES_MLNO(n) (*(volatile uint32_t *)(DMA_BASE + 0x1000 * (n) + 0x008))
#define DMA_TCD_SLAST(n)       (*(volatile uint32_t *)(DMA_BASE + 0x1000 * (n) + 0x00C))
#define DMA_TCD_DADDR(n)       (*(volatile uint32_t *)(DMA_BASE + 0x1000 * (n) + 0x010))
#define DMA_TCD_DOFF(n)        (*(volatile uint16_t *)(DMA_BASE + 0x1000 * (n) + 0x014))
#define DMA_TCD_CITER_ELINKNO(n) (*(volatile uint16_t *)(DMA_BASE + 0x1000 * (n) + 0x016))
#define DMA_TCD_DLASTSGA(n)    (*(volatile uint32_t *)(DMA_BASE + 0x1000 * (n) + 0x018))
#define DMA_TCD_CSR(n)         (*(volatile uint16_t *)(DMA_BASE + 0x1000 * (n) + 0x01C))
#define DMA_TCD_BITER_ELINKNO(n) (*(volatile uint16_t *)(DMA_BASE + 0x1000 * (n) + 0x01E))

// DMA Multiplexer Registers
#define DMAMUX_CHCFG(n)        (*(volatile uint32_t *)(DMAMUX_BASE + 0x4 * (n)))

// DMA Control Registers
#define DMA_CR                 (*(volatile uint32_t *)(DMA_BASE + 0x000))
#define DMA_ES                 (*(volatile uint32_t *)(DMA_BASE + 0x004))
#define DMA_ERQ                (*(volatile uint32_t *)(DMA_BASE + 0x00C))
#define DMA_EEI                (*(volatile uint32_t *)(DMA_BASE + 0x014))
#define DMA_CEEI               (*(volatile uint8_t  *)(DMA_BASE + 0x018))
#define DMA_SEEI               (*(volatile uint8_t  *)(DMA_BASE + 0x019))
#define DMA_CERQ               (*(volatile uint8_t  *)(DMA_BASE + 0x01A))
#define DMA_SERQ               (*(volatile uint8_t  *)(DMA_BASE + 0x01B))
#define DMA_CDNE               (*(volatile uint8_t  *)(DMA_BASE + 0x01C))
#define DMA_SSRT               (*(volatile uint8_t  *)(DMA_BASE + 0x01D))
#define DMA_CERR               (*(volatile uint8_t  *)(DMA_BASE + 0x01E))
#define DMA_CINT               (*(volatile uint8_t  *)(DMA_BASE + 0x01F))

// ADC Configuration bits
#define ADC_CFG_ADICLK(n)      (((n) & 0x3) << 0)
#define ADC_CFG_MODE(n)        (((n) & 0x3) << 2)
#define ADC_CFG_ADLSMP         (1 << 4)
#define ADC_CFG_ADIV(n)        (((n) & 0x3) << 5)
#define ADC_CFG_ADLPC          (1 << 7)
#define ADC_CFG_AVGS(n)        (((n) & 0x3) << 14)
#define ADC_CFG_ADTRG          (1 << 13)
#define ADC_CFG_REFSEL(n)      (((n) & 0x3) << 20)

// ADC General Control bits
#define ADC_GC_ADCO            (1 << 6)
#define ADC_GC_DMAEN           (1 << 8)

// DMAMUX Configuration
#define DMAMUX_CHCFG_ENBL      (1 << 31)
#define DMAMUX_CHCFG_SOURCE(n) ((n) & 0xFF)
#define DMAMUX_SOURCE_ADC1     54

// DMA TCD bits
#define DMA_TCD_CSR_INTMAJOR   (1 << 1)
#define DMA_TCD_CSR_DREQ       (1 << 3)

class LowLevelADCDMA {
private:
    static constexpr uint8_t DMA_CHANNEL = 0;
    DMAMEM static uint16_t _adcBuffer[1024] __attribute__((aligned(32)));
    static constexpr uint8_t ADC_PIN = 16; // A2/Pin 16 on Teensy 4.1

public:
    void begin() {
        // Configure ADC
        ADC1_CFG = ADC_CFG_ADICLK(1) |    // Input clock select (IPG clock)
                   ADC_CFG_MODE(1) |       // 12-bit conversion
                   ADC_CFG_ADIV(0) |       // Divide ratio = 1
                   ADC_CFG_ADLSMP |        // Long sample time
                   ADC_CFG_AVGS(2) |       // 8 sample average
                   ADC_CFG_ADTRG;          // Hardware trigger
                   
        ADC1_GC = ADC_GC_DMAEN | ADC_GC_ADCO;  // Enable DMA and continuous conversion
        
        // Enable DMA clock
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
        
        // Configure DMAMUX for ADC1
        DMAMUX_CHCFG(DMA_CHANNEL) = 0; // Disable first
        DMAMUX_CHCFG(DMA_CHANNEL) = DMAMUX_CHCFG_SOURCE(DMAMUX_SOURCE_ADC1) | 
                                   DMAMUX_CHCFG_ENBL;
        
        // Setup TCD (Transfer Control Descriptor)
        DMA_TCD_SADDR(DMA_CHANNEL) = (uint32_t)&ADC1_R0;
        DMA_TCD_DADDR(DMA_CHANNEL) = (uint32_t)_adcBuffer;
        
        // Configure transfer attributes
        DMA_TCD_ATTR(DMA_CHANNEL) = DMA_TCD_ATTR_SSIZE(1) | // 16-bit source
                                   DMA_TCD_ATTR_DSIZE(1);    // 16-bit dest
        
        // Configure transfer size
        DMA_TCD_NBYTES_MLNO(DMA_CHANNEL) = 2; // 2 bytes per transfer
        
        // Configure source adjustment
        DMA_TCD_SLAST(DMA_CHANNEL) = 0;  // Don't adjust source after transfer
        
        // Configure destination adjustment and wrap
        DMA_TCD_DLASTSGA(DMA_CHANNEL) = -(int32_t)sizeof(_adcBuffer);
        
        // Configure transfer counts
        DMA_TCD_CITER_ELINKNO(DMA_CHANNEL) = sizeof(_adcBuffer) / 2;
        DMA_TCD_BITER_ELINKNO(DMA_CHANNEL) = sizeof(_adcBuffer) / 2;
        
        // Configure TCD control
        DMA_TCD_CSR(DMA_CHANNEL) = DMA_TCD_CSR_INTMAJOR;
        
        // Setup DMA interrupt
        attachInterruptVector(static_cast<IRQ_NUMBER_t>(IRQ_DMA_CH0 + DMA_CHANNEL), dmaISR);
        NVIC_ENABLE_IRQ(IRQ_DMA_CH0 + DMA_CHANNEL);
        
        // Configure ADC pin
        pinMode(ADC_PIN, INPUT);
        
        // Start DMA
        DMA_SERQ = DMA_CHANNEL;
        
        // Start ADC conversion on specified channel
        ADC1_HC0 = ADC_PIN & 0x1F;  // Select ADC channel
    }

private:
    static void dmaISR() {
        DMA_CINT = DMA_CHANNEL;  // Clear interrupt
        // Process the buffer here
        digitalToggleFast(LED_BUILTIN); // Toggle LED to show activity
    }
};

// Initialize static member
uint16_t LowLevelADCDMA::_adcBuffer[1024] __attribute__((aligned(32)));
```

### 3. SPI DMA Transfer

#### Teensyduino Approach:
```cpp
class TeensySPIDMA {
private:
    DMAChannel _dmaTX;
    DMAChannel _dmaRX;
    DMAMEM static uint8_t _txBuffer[256];
    DMAMEM static uint8_t _rxBuffer[256];
    static volatile bool _transferComplete;
    static uint8_t _staticCsPin;  // Add static pin storage
    const uint8_t _csPin;

public:
    TeensySPIDMA(uint8_t csPin) : _csPin(csPin) {
        _transferComplete = false;
        _staticCsPin = csPin;  // Store pin in static variable
    }

    void begin() {
        // Configure SPI and CS pin
        SPI.begin();
        pinMode(_csPin, OUTPUT);
        digitalWriteFast(_csPin, HIGH);
        
        // Configure DMA channels
        _dmaTX.begin(true);  // Force hardware allocation
        _dmaRX.begin(true);
        
        // Set up completion callback
        _dmaRX.attachInterrupt(dmaCallback);
    }

    // Function to perform a transfer
    bool transfer(const uint8_t* sendData, size_t length) {
        if (length > sizeof(_txBuffer)) return false;
        
        // Copy data to TX buffer
        memcpy(_txBuffer, sendData, length);
        
        // Reset completion flag
        _transferComplete = false;
        
        // Configure TX DMA
        _dmaTX.sourceBuffer(_txBuffer, length);
        _dmaTX.destination((volatile uint8_t&)IMXRT_LPSPI4_S.TDR);
        _dmaTX.transferSize(1);
        _dmaTX.transferCount(length);
        _dmaTX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_TX);
        
        // Configure RX DMA
        _dmaRX.source((volatile uint8_t&)IMXRT_LPSPI4_S.RDR);
        _dmaRX.destinationBuffer(_rxBuffer, length);
        _dmaRX.transferSize(1);
        _dmaRX.transferCount(length);
        _dmaRX.triggerAtHardwareEvent(DMAMUX_SOURCE_LPSPI4_RX);
        
        // Start transfer
        SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
        digitalWriteFast(_csPin, LOW);
        
        // Enable DMA requests for LPSPI4
        IMXRT_LPSPI4_S.DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE; // Enable TX and RX DMA requests
        
        // Start DMA transfers
        _dmaRX.enable();
        _dmaTX.enable();
        
        return true;
    }
    
    // Check if transfer is complete
    bool isTransferComplete() const {
        return _transferComplete;
    }
    
    // Get received data
    const uint8_t* getReceivedData() const {
        return _rxBuffer;
    }

private:
    static void dmaCallback() {
        // Disable DMA requests
        IMXRT_LPSPI4_S.DER = 0;
        
        // Complete the transfer
        digitalWriteFast(_staticCsPin, HIGH);  // Use static pin
        SPI.endTransaction();
        
        _transferComplete = true;
    }
};

// Initialize static members
uint8_t TeensySPIDMA::_txBuffer[256] __attribute__((aligned(32)));
uint8_t TeensySPIDMA::_rxBuffer[256] __attribute__((aligned(32)));
volatile bool TeensySPIDMA::_transferComplete;
uint8_t TeensySPIDMA::_staticCsPin;

// Example usage
void setup() {
    TeensySPIDMA spi(10);  // CS on pin 10
    spi.begin();
    
    // Example data to send
    uint8_t dataToSend[] = {0x01, 0x02, 0x03, 0x04};
    
    // Start transfer
    if (spi.transfer(dataToSend, sizeof(dataToSend))) {
        // Wait for transfer to complete
        while (!spi.isTransferComplete()) {
            // Could do other tasks here
        }
        
        // Access received data
        const uint8_t* receivedData = spi.getReceivedData();
        // Process receivedData here
    }
}
```

#### Low-Level Approach:
```cpp
// IMXRT Register definitions
#ifndef CCM_CCGR1_LPSPI4
#define CCM_CCGR1_LPSPI4(n)		((uint32_t)(((n) & 0x03) << 6))
#define CCM_CCGR5_DMA(n)		((uint32_t)(((n) & 0x03) << 0))
#define CCM_CCGR_ON			0x03
#endif

#ifndef DMAMUX_SOURCE_LPSPI4_TX
#define DMAMUX_SOURCE_LPSPI4_TX    11
#define DMAMUX_SOURCE_LPSPI4_RX    10
#endif

#ifndef DMAMUX_CHCFG_ENBL
#define DMAMUX_CHCFG_ENBL          (1<<31)
#endif

#ifndef DMAMUX_CHCFG
#define DMAMUX_CHCFG(n)           (*(volatile uint32_t *)(IMXRT_DMAMUX_OFFSET(n)))
#endif

#ifndef LPSPI_CR_MEN
#define LPSPI_CR_MEN               (1<<0)
#define LPSPI_CFGR1_MASTER         (1<<0)
#define LPSPI_DER_TDDE             (1<<0)
#define LPSPI_DER_RDDE             (1<<1)
#endif

// IMXRT DMA Register definitions for Teensy 4.1
#define IMXRT_DMA_OFFSET(n)     (0x400E8000 + ((n) * 0x20))
#define IMXRT_DMAMUX_OFFSET(n)  (0x400EC000 + ((n) * 0x4))

#define DMA_TCD_ATTR_SSIZE(n)      (((n) & 0x7) << 8)
#define DMA_TCD_ATTR_DSIZE(n)      (((n) & 0x7) << 0)

// DMA channel registers
#define DMA_TCD_SADDR(n)        (*(volatile uint32_t *)(IMXRT_DMA_OFFSET(n) + 0x000))
#define DMA_TCD_DADDR(n)        (*(volatile uint32_t *)(IMXRT_DMA_OFFSET(n) + 0x004))
#define DMA_TCD_ATTR(n)         (*(volatile uint16_t *)(IMXRT_DMA_OFFSET(n) + 0x008))
#define DMA_TCD_NBYTES_MLNO(n)  (*(volatile uint32_t *)(IMXRT_DMA_OFFSET(n) + 0x00C))
#define DMA_TCD_SLAST(n)        (*(volatile uint32_t *)(IMXRT_DMA_OFFSET(n) + 0x010))
#define DMA_TCD_DLASTSGA(n)     (*(volatile uint32_t *)(IMXRT_DMA_OFFSET(n) + 0x014))
#define DMA_TCD_CSR(n)          (*(volatile uint16_t *)(IMXRT_DMA_OFFSET(n) + 0x01C))
#define DMA_TCD_BITER_ELINKNO(n) (*(volatile uint16_t *)(IMXRT_DMA_OFFSET(n) + 0x01E))
#define DMA_TCD_CITER_ELINKNO(n) (*(volatile uint16_t *)(IMXRT_DMA_OFFSET(n) + 0x016))

// DMAMUX registers
#define DMAMUX_CHCFG(n)         (*(volatile uint32_t *)(IMXRT_DMAMUX_OFFSET(n)))

// DMA TCD CSR bits
#define DMA_TCD_CSR_INTMAJOR       (1<<1)

// DMA Status/Control
#define DMA_INT                     (*(volatile uint32_t*)0x400E8024)
#define DMA_SERQ                    (*(volatile uint8_t*)0x400E800D)
#define DMA_CINT                    (*(volatile uint8_t*)0x400E800E)

class LowLevelSPIDMA {
private:
    static constexpr uint8_t DMA_TX_CH = 0;
    static constexpr uint8_t DMA_RX_CH = 1;
    DMAMEM static uint8_t _txBuffer[256] __attribute__((aligned(32)));
    DMAMEM static uint8_t _rxBuffer[256] __attribute__((aligned(32)));

public:
    void begin() {
        // Configure SPI
        CCM_CCGR1 |= CCM_CCGR1_LPSPI4(CCM_CCGR_ON);
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_00 = 3; // SDI
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_01 = 3; // SDO
        IOMUXC_SW_MUX_CTL_PAD_GPIO_B0_02 = 3; // SCK
        
        IMXRT_LPSPI4_S.CR = 0;  // Disable module for config
        IMXRT_LPSPI4_S.CFGR1 = LPSPI_CFGR1_MASTER;
        IMXRT_LPSPI4_S.CR = LPSPI_CR_MEN;  // Enable module
        
        // Enable DMA clock and configure MUX
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
        DMAMUX_CHCFG(DMA_TX_CH) = DMAMUX_SOURCE_LPSPI4_TX | DMAMUX_CHCFG_ENBL;
        DMAMUX_CHCFG(DMA_RX_CH) = DMAMUX_SOURCE_LPSPI4_RX | DMAMUX_CHCFG_ENBL;
        
        // Setup TX TCD
        DMA_TCD_SADDR(DMA_TX_CH) = (uint32_t)_txBuffer;
        DMA_TCD_DADDR(DMA_TX_CH) = (uint32_t)&IMXRT_LPSPI4_S.TDR;
        DMA_TCD_ATTR(DMA_TX_CH) = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
        DMA_TCD_NBYTES_MLNO(DMA_TX_CH) = 1;
        DMA_TCD_SLAST(DMA_TX_CH) = 0;
        DMA_TCD_CITER_ELINKNO(DMA_TX_CH) = sizeof(_txBuffer);
        DMA_TCD_BITER_ELINKNO(DMA_TX_CH) = sizeof(_txBuffer);
        DMA_TCD_CSR(DMA_TX_CH) = DMA_TCD_CSR_INTMAJOR;
        
        // Setup RX TCD
        DMA_TCD_SADDR(DMA_RX_CH) = (uint32_t)&IMXRT_LPSPI4_S.RDR;
        DMA_TCD_DADDR(DMA_RX_CH) = (uint32_t)_rxBuffer;
        DMA_TCD_ATTR(DMA_RX_CH) = DMA_TCD_ATTR_SSIZE(0) | DMA_TCD_ATTR_DSIZE(0);
        DMA_TCD_NBYTES_MLNO(DMA_RX_CH) = 1;
        DMA_TCD_DLASTSGA(DMA_RX_CH) = 0;
        DMA_TCD_CITER_ELINKNO(DMA_RX_CH) = sizeof(_rxBuffer);
        DMA_TCD_BITER_ELINKNO(DMA_RX_CH) = sizeof(_rxBuffer);
        DMA_TCD_CSR(DMA_RX_CH) = DMA_TCD_CSR_INTMAJOR;
        
        // Enable DMA requests in SPI
        IMXRT_LPSPI4_S.DER = LPSPI_DER_TDDE | LPSPI_DER_RDDE;
        
        // Enable DMA channels
        DMA_SERQ = DMA_RX_CH;
        DMA_SERQ = DMA_TX_CH;
    }

private:
    static void dmaISR() {
        if (DMA_INT & (1 << DMA_TX_CH)) {
            DMA_CINT = DMA_TX_CH;
            // TX Complete
        }
        if (DMA_INT & (1 << DMA_RX_CH)) {
            DMA_CINT = DMA_RX_CH;
            // RX Complete
        }
    }
};
```

## Enhanced DMA (eDMA)

The IMXRT1060 features an enhanced DMA (eDMA) module that provides complex data movement capabilities beyond basic transfers. The eDMA supports sophisticated transfer patterns and advanced features for optimizing data movement.

### Key eDMA Features

1. Transfer Control Descriptor (TCD)
   - Programmable source/destination addresses
   - Programmable transfer sizes
   - Programmable address modulo
   - Support for scatter-gather operations

2. Channel Multiplexing
   - Multiple DMA channels (32 channels)
   - Programmable channel priorities
   - Channel linking capability
   - Channel chaining support

3. Transfer Types
   - Single block transfers
   - Multi-block transfers
   - Circular buffer operations
   - Scatter-gather operations
   - Channel-to-channel linking

### Enhanced DMA (eDMA) Examples

#### 1. Scatter-Gather Operations

Scatter-Gather DMA is an advanced DMA operation mode that allows for complex memory transfer patterns without CPU intervention. In this mode, the DMA controller can automatically chain multiple transfers together using Transfer Control Descriptors (TCDs).

Key features of scatter-gather:
- **Automatic Chaining**: Each TCD can point to the next one, creating a chain or ring of transfers
- **Zero CPU Overhead**: Once configured, transfers continue automatically without CPU intervention
- **Memory Efficiency**: Allows efficient handling of non-contiguous memory blocks
- **Flexible Buffer Management**: Perfect for circular buffer implementations or ping-pong buffering

In the following example, we implement a three-buffer circular chain where:
1. Buffer1 transfers to the peripheral
2. Automatically switches to Buffer2
3. Then to Buffer3
4. And finally loops back to Buffer1

##### Teensyduino Approach:
```cpp
class TeensyScatterGather {
private:
    static DMAChannel _dma;  // Make DMA channel static to access in callback
    DMAMEM static uint8_t _buffer1[256] __attribute__((aligned(32)));
    DMAMEM static uint8_t _buffer2[256] __attribute__((aligned(32)));
    DMAMEM static uint8_t _buffer3[256] __attribute__((aligned(32)));
    static volatile uint8_t _currentBuffer;

public:
    void begin() {
        _currentBuffer = 0;
        
        // Initialize buffers with test data
        for(int i = 0; i < 256; i++) {
            _buffer1[i] = i;
            _buffer2[i] = i + 0x40;
            _buffer3[i] = i + 0x80;
        }

        _dma.begin(true); // Force hardware allocation
        
        // Configure first transfer from buffer1
        _dma.sourceBuffer(_buffer1, sizeof(_buffer1));
        _dma.destination((volatile uint8_t &)IMXRT_LPSPI4_S.TDR);  // Example: SPI transmit
        _dma.transferSize(1);
        _dma.transferCount(sizeof(_buffer1));
        
        // Setup scatter-gather chain
        // Link buffer1 -> buffer2
        _dma.TCD->DLASTSGA = (uint32_t)&_buffer2;
        _dma.TCD->SLAST = -sizeof(_buffer1);  // Source modifier to reset position
        _dma.TCD->CSR = DMA_TCD_CSR_ESG | DMA_TCD_CSR_INTMAJOR;
        
        // Create a second TCD for buffer2 -> buffer3
        DMAChannel::TCD_t *tcd2 = (DMAChannel::TCD_t *)&_buffer2;
        tcd2->SADDR = (volatile void *)&_buffer2;
        tcd2->DADDR = (volatile void *)&IMXRT_LPSPI4_S.TDR;
        tcd2->ATTR = 0;  // 8-bit transfers
        tcd2->NBYTES = 1;
        tcd2->SLAST = -sizeof(_buffer2);
        tcd2->DLASTSGA = (int32_t)&_buffer3;
        tcd2->CITER = sizeof(_buffer2);
        tcd2->BITER = sizeof(_buffer2);
        tcd2->CSR = DMA_TCD_CSR_ESG | DMA_TCD_CSR_INTMAJOR;
        
        // Create a third TCD for buffer3 -> buffer1 (circular)
        DMAChannel::TCD_t *tcd3 = (DMAChannel::TCD_t *)&_buffer3;
        tcd3->SADDR = (volatile void *)&_buffer3;
        tcd3->DADDR = (volatile void *)&IMXRT_LPSPI4_S.TDR;
        tcd3->ATTR = 0;
        tcd3->NBYTES = 1;
        tcd3->SLAST = -sizeof(_buffer3);
        tcd3->DLASTSGA = (int32_t)&_buffer1;  // Back to buffer1
        tcd3->CITER = sizeof(_buffer3);
        tcd3->BITER = sizeof(_buffer3);
        tcd3->CSR = DMA_TCD_CSR_ESG | DMA_TCD_CSR_INTMAJOR;
        
        // Setup completion interrupt
        _dma.attachInterrupt(completeCallback);
        
        // Start transfer
        _dma.enable();
    }

private:
    static void completeCallback() {
        // Disable DMA requests
        IMXRT_LPSPI4_S.DER = 0;
        
        // Complete the transfer
        digitalWriteFast(LED_BUILTIN, !digitalReadFast(LED_BUILTIN));
        
        _currentBuffer = (_currentBuffer + 1) % 3;
        _dma.clearInterrupt();
        
        // You can add buffer handling code here
        // _currentBuffer tells you which buffer just completed
    }
};
```

##### Low-Level Approach:
```cpp
// DMA Control and Status Register bit definitions
#define DMA_CSR_ESG         (uint16_t)0x0010    // Enable Scatter/Gather Processing
#define DMA_CSR_INTMAJOR    (uint16_t)0x0002    // Enable interrupt when major iteration count completes

// Define your peripheral register address here - this needs to be set to your specific peripheral
#define PERIPHERAL_REG      (uint32_t)&IMXRT_LPSPI1_S.TDR  // SPI1 transmit data register

class LowLevelScatterGather {
private:
    static constexpr uint8_t DMA_CHANNEL = 0;
    DMAMEM static uint8_t _buffer1[256] __attribute__((aligned(32)));
    DMAMEM static uint8_t _buffer2[256] __attribute__((aligned(32)));
    
    struct TCD {
        volatile uint32_t SADDR;
        volatile uint16_t SOFF;
        volatile uint16_t ATTR;
        volatile uint32_t NBYTES;
        volatile uint32_t SLAST;
        volatile uint32_t DADDR;
        volatile uint16_t DOFF;
        volatile uint16_t CITER;
        volatile uint32_t DLASTSGA;
        volatile uint16_t CSR;
        volatile uint16_t BITER;
    };
    
    static TCD* const TCD_BASE;

public:
    void begin() {
        // Enable DMA clock
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
        
        // Configure first TCD
        TCD_BASE[DMA_CHANNEL].SADDR = (uint32_t)_buffer1;
        TCD_BASE[DMA_CHANNEL].SOFF = 1;
        TCD_BASE[DMA_CHANNEL].ATTR = (0 << 11) | (0 << 3) | // No modulo
                                    (0 << 8) | (0 << 0);     // 8-bit transfer
        TCD_BASE[DMA_CHANNEL].NBYTES = sizeof(_buffer1);
        TCD_BASE[DMA_CHANNEL].SLAST = -sizeof(_buffer1);  // Return to start of buffer
        TCD_BASE[DMA_CHANNEL].DADDR = PERIPHERAL_REG;
        TCD_BASE[DMA_CHANNEL].DOFF = 0;
        TCD_BASE[DMA_CHANNEL].CITER = 1;
        TCD_BASE[DMA_CHANNEL].DLASTSGA = (uint32_t)&TCD_BASE[DMA_CHANNEL + 1];  // Point to next TCD
        TCD_BASE[DMA_CHANNEL].CSR = DMA_CSR_ESG | DMA_CSR_INTMAJOR;
        TCD_BASE[DMA_CHANNEL].BITER = 1;
        
        // Configure second TCD
        TCD_BASE[DMA_CHANNEL + 1].SADDR = (uint32_t)_buffer2;
        TCD_BASE[DMA_CHANNEL + 1].SOFF = 1;
        TCD_BASE[DMA_CHANNEL + 1].ATTR = (0 << 11) | (0 << 3) | // No modulo
                                        (0 << 8) | (0 << 0);     // 8-bit transfer
        TCD_BASE[DMA_CHANNEL + 1].NBYTES = sizeof(_buffer2);
        TCD_BASE[DMA_CHANNEL + 1].SLAST = -sizeof(_buffer2);  // Return to start of buffer
        TCD_BASE[DMA_CHANNEL + 1].DADDR = PERIPHERAL_REG;
        TCD_BASE[DMA_CHANNEL + 1].DOFF = 0;
        TCD_BASE[DMA_CHANNEL + 1].CITER = 1;
        TCD_BASE[DMA_CHANNEL + 1].DLASTSGA = (uint32_t)&TCD_BASE[DMA_CHANNEL];  // Point back to first TCD
        TCD_BASE[DMA_CHANNEL + 1].CSR = DMA_CSR_ESG | DMA_CSR_INTMAJOR;
        TCD_BASE[DMA_CHANNEL + 1].BITER = 1;
        
        // Enable DMA interrupt
        attachInterruptVector((IRQ_NUMBER_t)(IRQ_DMA_CH0 + DMA_CHANNEL), dmaISR);
        NVIC_ENABLE_IRQ((IRQ_NUMBER_t)(IRQ_DMA_CH0 + DMA_CHANNEL));
        
        // Start DMA
        DMA_SERQ = DMA_CHANNEL;
    }

private:
    static void dmaISR() {
        DMA_CINT = DMA_CHANNEL;
        // Handle completion
    }
};

// Define static members
LowLevelScatterGather::TCD* const LowLevelScatterGather::TCD_BASE = (TCD*)&DMA_TCD0_SADDR;
```

#### 2. Channel Linking

Channel linking is a DMA feature that allows one DMA channel to automatically trigger another channel when it completes its transfer. This creates a chain of DMA operations where multiple channels can work in sequence without CPU intervention.

On the Teensy 4.1, channel linking is particularly useful for:
- Creating continuous data streams by alternating between buffers
- Coordinating multiple peripheral operations in sequence
- Implementing complex data movement patterns with minimal CPU overhead

The following example demonstrates how to link two DMA channels using Teensyduino's DMAChannel class. When the first channel completes its transfer from `_buffer1`, it automatically triggers the second channel to transfer from `_buffer2`:

##### Teensyduino Approach:
```cpp
class TeensyChannelLink {
private:
    DMAChannel _dma1;
    DMAChannel _dma2;
    DMAMEM static uint8_t _buffer1[256] __attribute__((aligned(32)));
    DMAMEM static uint8_t _buffer2[256] __attribute__((aligned(32)));

public:
    void begin() {
        // Configure first channel
        _dma1.begin();  // Initialize DMA channel
        _dma1.sourceBuffer(_buffer1, sizeof(_buffer1));
        _dma1.destination((volatile uint8_t&)IMXRT_LPSPI1_S.TDR);
        _dma1.transferSize(1);
        _dma1.transferCount(sizeof(_buffer1));
        _dma1.interruptAtCompletion();
        
        // Configure second channel
        _dma2.begin();  // Initialize DMA channel
        _dma2.sourceBuffer(_buffer2, sizeof(_buffer2));
        _dma2.destination((volatile uint8_t&)IMXRT_LPSPI1_S.TDR);
        _dma2.transferSize(1);
        _dma2.transferCount(sizeof(_buffer2));
        _dma2.interruptAtCompletion();
        
        // Link channels - when dma1 completes, it triggers dma2
        _dma1.triggerAtCompletionOf(_dma2);
        
        // Enable SPI DMA requests
        IMXRT_LPSPI1_S.DER = LPSPI_DER_TDDE;
        
        // Start first channel
        _dma1.enable();
    }
};
```

##### Low-Level Approach:
```cpp
// DMA Control and Status Register bit definitions
#define DMA_CSR_ESG         (uint16_t)0x0010    // Enable Scatter/Gather Processing
#define DMA_CSR_INTMAJOR    (uint16_t)0x0002    // Enable interrupt when major iteration count completes
#define DMA_CSR_MAJORELINK  (uint16_t)0x0020    // Enable channel-to-channel linking on major loop complete
#define DMA_CSR_DREQ        (uint16_t)0x0008    // Disable request at end of major loop

class LowLevelChannelLink {
private:
    static constexpr uint8_t DMA_CH1 = 0;
    static constexpr uint8_t DMA_CH2 = 1;
    DMAMEM static uint8_t _buffer1[256] __attribute__((aligned(32)));
    DMAMEM static uint8_t _buffer2[256] __attribute__((aligned(32)));

    struct TCD {
        volatile uint32_t SADDR;
        volatile uint16_t SOFF;
        volatile uint16_t ATTR;
        volatile uint32_t NBYTES;
        volatile uint32_t SLAST;
        volatile uint32_t DADDR;
        volatile uint16_t DOFF;
        volatile uint16_t CITER;
        volatile uint32_t DLASTSGA;
        volatile uint16_t CSR;
        volatile uint16_t BITER;
    };

    static TCD* const TCD_BASE;

public:
    void begin() {
        // Enable DMA clock
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
        
        // Configure first channel
        TCD_BASE[DMA_CH1].SADDR = (uint32_t)_buffer1;
        TCD_BASE[DMA_CH1].SOFF = 1;  // Source increment by 1
        TCD_BASE[DMA_CH1].ATTR = (0 << 11) | (0 << 3) | // No modulo
                                (0 << 8) | (0 << 0);     // 8-bit transfer
        TCD_BASE[DMA_CH1].NBYTES = sizeof(_buffer1);
        TCD_BASE[DMA_CH1].SLAST = -sizeof(_buffer1);  // Return to start
        TCD_BASE[DMA_CH1].DADDR = (uint32_t)&IMXRT_LPSPI1_S.TDR;
        TCD_BASE[DMA_CH1].DOFF = 0;  // Don't increment destination
        TCD_BASE[DMA_CH1].CITER = 1;
        TCD_BASE[DMA_CH1].DLASTSGA = (uint32_t)&TCD_BASE[DMA_CH2];  // Link to second channel
        TCD_BASE[DMA_CH1].CSR = DMA_CSR_MAJORELINK | DMA_CSR_INTMAJOR;  // Enable linking and interrupt
        TCD_BASE[DMA_CH1].BITER = 1;
        
        // Configure second channel
        TCD_BASE[DMA_CH2].SADDR = (uint32_t)_buffer2;
        TCD_BASE[DMA_CH2].SOFF = 1;
        TCD_BASE[DMA_CH2].ATTR = (0 << 11) | (0 << 3) | // No modulo
                                (0 << 8) | (0 << 0);     // 8-bit transfer
        TCD_BASE[DMA_CH2].NBYTES = sizeof(_buffer2);
        TCD_BASE[DMA_CH2].SLAST = -sizeof(_buffer2);  // Return to start
        TCD_BASE[DMA_CH2].DADDR = (uint32_t)&IMXRT_LPSPI1_S.TDR;
        TCD_BASE[DMA_CH2].DOFF = 0;
        TCD_BASE[DMA_CH2].CITER = 1;
        TCD_BASE[DMA_CH2].DLASTSGA = (uint32_t)&TCD_BASE[DMA_CH1];  // Link back to first channel
        TCD_BASE[DMA_CH2].CSR = DMA_CSR_MAJORELINK | DMA_CSR_INTMAJOR;  // Enable linking and interrupt
        TCD_BASE[DMA_CH2].BITER = 1;
        
        // Enable SPI DMA requests
        IMXRT_LPSPI1_S.DER = LPSPI_DER_TDDE;
        
        // Start first channel
        DMA_SERQ = DMA_CH1;
    }
};

// Define static members
LowLevelChannelLink::TCD* const LowLevelChannelLink::TCD_BASE = (TCD*)&DMA_TCD0_SADDR;
```
#### 3. Circular Buffer with Modulo Addressing

Circular buffers are a common use case for DMA transfers, especially when dealing with continuous data streams. Modulo addressing allows the DMA controller to automatically wrap around to the beginning of the buffer when it reaches the end.

The following example demonstrates how to configure a circular buffer with modulo addressing using Teensyduino's DMAChannel class:

##### Teensyduino Approach:
```cpp
class TeensyModuloBuffer {
private:
    DMAChannel _dma;
    // Buffer size must be power of 2 and aligned to its size for modulo addressing
    DMAMEM static volatile uint8_t  _buffer[256] __attribute__((aligned(256)));

public:
    void begin() {
        // Initialize DMA channel with forced allocation
        _dma.begin(true);
        
        // Configure the transfer
        _dma.sourceBuffer(_buffer, sizeof(_buffer));
        _dma.destination((volatile uint8_t&)IMXRT_LPSPI1_S.TDR);
        _dma.transferSize(1);  // 8-bit transfers
        _dma.transferCount(sizeof(_buffer));
        
        // Enable modulo addressing for source
        // For modulo addressing, buffer size must be 2^N and aligned to its size
        _dma.TCD->ATTR = (_dma.TCD->ATTR & 0xF0FF) | ((8 & 0xF) << 8); // Set SMOD to log2(256)=8
        
        // Enable interrupt at completion for buffer management
        _dma.interruptAtCompletion();
        
        // Enable SPI DMA requests
        IMXRT_LPSPI1_S.DER = LPSPI_DER_TDDE;
        
        // Start the transfer
        _dma.enable();
    }

    // Method to fill the buffer while DMA is running
    void fillBuffer(const uint8_t* data, size_t length, size_t offset = 0) {
        if (offset + length <= sizeof(_buffer)) {
            memcpy((void*)(_buffer + offset), data, length);
        }
    }
};
```

##### Low-Level Approach:
```cpp
class LowLevelModuloBuffer {
private:
    static constexpr uint8_t DMA_CHANNEL = 0;
    // Buffer must be volatile since it's accessed by DMA
    DMAMEM static volatile uint8_t _buffer[256] __attribute__((aligned(256)));

public:
    void begin() {
        // Enable DMA clock
        CCM_CCGR5 |= CCM_CCGR5_DMA(CCM_CCGR_ON);
        
        // Get pointer to TCD (Transfer Control Descriptor)
        volatile uint32_t* TCD = (volatile uint32_t*)(&(IMXRT_DMA_TCD[DMA_CHANNEL].SADDR));
        
        // Configure source and destination
        TCD[0] = (uint32_t)_buffer;                         // SADDR
        TCD[2] = (uint32_t)&IMXRT_LPSPI1_S.TDR;           // DADDR
        
        // Set modulo 256 (2^8) for source and transfer sizes
        TCD[1] = (8 << 11) |                               // ATTR: Source modulo
                (0 << 8) |                                  // ATTR: SSIZE = 8-bit
                (0 << 3);                                   // ATTR: DSIZE = 8-bit
        
        // Configure transfer size
        TCD[3] = 1;                                        // NBYTES: 1 byte per minor loop
        
        // Configure counts
        TCD[4] = sizeof(_buffer);                          // SLAST: Source adjust
        TCD[5] = 0;                                        // DADDR_LAST: No dest adjustment
        TCD[6] = (sizeof(_buffer) << 16) | sizeof(_buffer);// CITER & BITER
        
        // Configure control and status
        TCD[7] = (1 << 31) |                              // CSR: Enable interrupt
                 (1 << 3);                                 // CSR: DREQ - disable ERQ on complete
        
        // Enable DMA request from LPSPI1
        IMXRT_LPSPI1_S.DER = LPSPI_DER_TDDE;
        
        // Enable DMA channel
        DMA_SERQ = DMA_CHANNEL;
    }
    
    // Method to safely fill buffer while DMA is running
    void fillBuffer(const uint8_t* data, size_t length, size_t offset = 0) {
        if (offset + length <= sizeof(_buffer)) {
            memcpy((void*)(_buffer + offset), data, length);
        }
    }
};
```

These examples demonstrate:
1. Proper use of both Teensyduino's DMAChannel class and direct register access
2. Memory alignment requirements for DMA operations
3. Advanced features like scatter-gather, channel linking, and modulo addressing
4. Interrupt handling for transfer completion
5. Proper buffer management and circular operations

## DMA Command Reference

### Basic DMA Control Commands

```cpp
DMAChannel dma;  // Create DMA channel

// Initialization and Control
dma.begin();                    // Initialize DMA channel
dma.enable();                   // Enable DMA transfers
dma.disable();                  // Disable DMA transfers
dma.pause();                    // Pause DMA operations
dma.unpause();                  // Resume DMA operations
dma.clearComplete();           // Clear transfer complete flag
dma.clearError();              // Clear error flags
```

### Transfer Configuration Commands

```cpp
// Source Configuration
dma.source(volatile const void *src);           // Set source address
dma.sourceBuffer(volatile const void *src,      // Set source buffer
                uint32_t size);                 // with size
dma.sourceCircular(volatile const void *src,    // Set circular source buffer
                  uint32_t size);               // with size

// Destination Configuration
dma.destination(volatile void *dst);            // Set destination address
dma.destinationBuffer(volatile void *dst,       // Set destination buffer
                     uint32_t size);            // with size
dma.destinationCircular(volatile void *dst,     // Set circular destination buffer
                       uint32_t size);          // with size

// Transfer Parameters
dma.transferSize(uint32_t size);               // Set size of each transfer (1,2,4 bytes)
dma.transferCount(uint32_t count);             // Set number of transfers to perform
dma.transferSize(uint32_t size);               // Set transfer size in bytes

// Address Increment Configuration
dma.sourceIncrement(int16_t inc);              // Set source address increment
dma.destinationIncrement(int16_t inc);         // Set destination address increment
```

### Trigger and Interrupt Configuration

```cpp
// Trigger Configuration
dma.triggerAtHardwareEvent(uint8_t source);    // Set hardware trigger source
dma.triggerAtTransfersComplete(uint32_t count); // Trigger after N transfers
dma.triggerContinuously();                     // Enable continuous triggering
dma.disableTrigger();                          // Disable triggering

// Interrupt Configuration
dma.interruptAtCompletion();                   // Interrupt when complete
dma.interruptAtHalf();                         // Interrupt at halfway point
dma.attachInterrupt(void (*func)());           // Attach completion interrupt
dma.attachError(void (*func)());               // Attach error interrupt
dma.detachInterrupt();                         // Remove completion interrupt
dma.detachError();                             // Remove error interrupt
```

### Status and Error Checking

```cpp
// Status Checks
bool busy = dma.busy();                        // Check if transfer is active
bool complete = dma.complete();                // Check if transfer is complete
bool error = dma.error();                      // Check for errors

// Error Information
uint32_t errorCount = dma.errorCount;          // Get error count
uint32_t errorSource = dma.errorSource;        // Get error source
```

### Memory Requirements and Alignment
Teensy 4.1 DMA requires proper memory alignment
Use DMAMEM macro for DMA buffers to ensure they're in the correct memory region
Different transfer sizes require different alignments:
32-byte alignment for optimal performance
4-byte alignment minimum for 32-bit transfers
2-byte alignment minimum for 16-bit transfers
```cpp
// Memory must be properly aligned for DMA
DMAMEM static uint32_t buffer __attribute__((aligned(32)));  // 32-byte alignment
```


### Channel Selection and Limitations
Not all DMA channels can be used with all peripherals
Some peripherals have dedicated DMA channels
Check the IMXRT1062 reference manual for channel assignments
```cpp
// Teensy 4.1 has 32 DMA channels (0-31)
// Some peripherals require specific DMA channels
static constexpr uint8_t LPSPI4_TX_CHANNEL = 7;  // Example: LPSPI4 TX uses channel 7
static constexpr uint8_t LPSPI4_RX_CHANNEL = 6;  // Example: LPSPI4 RX uses channel 6
```


### Modulo Buffer Configuration

```cpp
// Setting up a modulo buffer (power of 2 sized circular buffer)
dma.TCD->ATTR = (DMA_TCD_ATTR_SMOD(8) |      // Source modulo 256 (2^8)
                 DMA_TCD_ATTR_SSIZE(2) |      // 32-bit source
                 DMA_TCD_ATTR_DSIZE(2));      // 32-bit destination

// Common modulo sizes:
// SMOD/DMOD value -> Buffer Size
// 7  -> 128 bytes
// 8  -> 256 bytes
// 9  -> 512 bytes
// 10 -> 1024 bytes
```


### Performance Considerations

- DMA transfers are most efficient when:
  - Buffers are 32-byte aligned
  - Transfer sizes match peripheral width
  - Burst transfers are used where possible
  - Minimize interrupt frequency for high-speed transfers
  
```cpp
// Example of optimized DMA setup
dma.TCD->NBYTES_MLNO = 32;          // Transfer 32 bytes per minor loop
dma.TCD->ATTR |= DMA_TCD_ATTR_DSIZE(2) | // 32-bit transfers
                 DMA_TCD_ATTR_SSIZE(2);   
```

### Error Recovery and Debug
Always implement error recovery
Monitor error conditions in production code
Consider implementing watchdog for critical DMA operations
```cpp
void recoveryFromDMAError() {
    dma.disable();
    dma.clearError();
    
    // Check specific error bits
    if (DMA_ES & DMA_ES_VLD) {  // Valid error exists
        if (DMA_ES & DMA_ES_ECX) {
            // Transfer was canceled
        }
        if (DMA_ES & DMA_ES_GPE) {
            // Group priority error
        }
        // ... handle other error conditions
    }
    
    // Reset DMA channel
    dma.begin();
    // Reconfigure as needed
}
```



### Advanced Configuration

```cpp
// Direct TCD (Transfer Control Descriptor) Access
dma.TCD->SADDR = sourceAddress;                // Set source address
dma.TCD->DADDR = destAddress;                  // Set destination address
dma.TCD->NBYTES = bytesPerTransfer;           // Set bytes per transfer
dma.TCD->SLAST = sourceLastAdjust;            // Source last adjustment
dma.TCD->DLASTSGA = destLastAdjust;           // Destination last adjustment
dma.TCD->BITER = beginningIterationCount;     // Beginning minor loop count
dma.TCD->CITER = currentIterationCount;       // Current iteration count
dma.TCD->CSR = controlAndStatus;              // Control and status

// Channel Priority
dma.priority(uint8_t level);                   // Set channel priority (0-15)

// Linking Channels
dma.chainTo(DMAChannel *next);                // Chain to another DMA channel

```

```cpp
// Enhanced DMA: This needs both Teensyduino and low-level features:
   - Keep the scatter-gather setup (low-level)
   - Keep error handling (low-level)
   - Use Teensy's DMA methods where possible
   - Use Teensy's hardware trigger constants

class EnhancedDMA {
private:
    DMAChannel dma;
    DMAMEM static uint32_t _buffer1[256];
    DMAMEM static uint32_t _buffer2[256];
    static volatile bool _usingBuffer1;
    static volatile bool _error;

public:
    void setupDoubleBuffer(uint32_t hardwareTrigger) {
        // Configure initial buffer
        dma.begin();  // Initialize DMA channel
        dma.sourceBuffer(_buffer1, sizeof(_buffer1));
        dma.destination(PERIPHERAL_REG);
        dma.transferSize(4);
        dma.transferCount(sizeof(_buffer1) / 4);
        
        // Setup hardware trigger if specified
        if (hardwareTrigger) {
            dma.triggerAtHardwareEvent(hardwareTrigger);
        }
        
        // Enable interrupts for half transfer, completion, and error
        dma.attachInterrupt(swapBuffers);
        dma.attachError(errorHandler);
        dma.enableInterruptAtCompletion();
        dma.interruptAtHalf();
        
        // Optional: Set channel priority
        dma.priority(DMAChannel::HighPriority);
        
        // Enable scatter-gather mode for complex transfers
        dma.TCD->DLASTSGA = (uint32_t)&_buffer2;  // Next buffer address
        dma.TCD->CSR |= DMA_CSR_ESG;              // Enable scatter-gather
        
        // Start DMA
        dma.enable();
    }

    bool hasError() const { return _error; }
    
    void reset() {
        dma.disable();
        _error = false;
        _usingBuffer1 = true;
        setupDoubleBuffer(0);  // Restart with same settings
    }

private:
    static void swapBuffers() {
        if (!_error) {
            if (_usingBuffer1) {
                dma.sourceBuffer(_buffer2, sizeof(_buffer2));
                // Optional: Process _buffer1 here while DMA uses _buffer2
            } else {
                dma.sourceBuffer(_buffer1, sizeof(_buffer1));
                // Optional: Process _buffer2 here while DMA uses _buffer1
            }
            _usingBuffer1 = !_usingBuffer1;
        }
        dma.clearInterrupt();
    }

    static void errorHandler() {
        _error = true;
        dma.clearError();
        
        // Optional: Log error details
        if (dma.error()) {
            uint32_t status = dma.errorStatus();
            // Handle specific error conditions
            if (status & DMA_ES_ECX) {
                // Transfer cancelled error
            }
            if (status & DMA_ES_GPE) {
                // Group priority error
            }
            // etc.
        }
    }
};
```

