Static Allocation Keywords
When the compiler builds your program, all global variables, static variables, and compiled code is assigned to dedicated locations in memory. This is called static allocation, because the memory addresses are fixed. By default, allocation tries to use the ultra-fast DTCM & ITCM memory. The following keywords allow control over where the compiler will place your variables and code within the memory.
DMAMEM - Variables defined with DMAMEM are placed at the beginning of RAM2. Normally buffers and large arrays are placed here. These variables can not be initialized, your program must write their initial values, if needed.
EXTMEM - Variables defined with EXTMEM are placed in the optional PSRAM memory chip soldered to the QSPI memory expansion area on bottom side of Teensy 4.1. These variables can not be initialized, your program must write their initial values, if needed.
PROGMEM & F() - Variables defined with PROGMEM, and strings surrounded by F() are placed only in the flash memory. They can be accessed normally, special functions normally used on 8 bit boards are not required to read PROGMEM variables.
FASTRUN - Functions defined with "FASTRUN" are allocated in the beginning of RAM1. A copy is also stored in Flash and copied to RAM1 at startup. These functions are accessed by the Cortex-M7 ITCM bus, for the fastest possible performance. By default, functions without any memory type defined are treated as FASTRUN. A small amount of memory is typically unused, because the ITCM bus must access a memory region which is a multiple of 32K.
FLASHMEM - Functions defined with "FLASHMEM" executed directly from Flash. If the Cortex-M7 cache is not already holding a copy of the function, a delay result

Dynamic Allocation
As your program runs, it may use all of the RAM which was not reserved by static allocation. Because the specific memory address for each variable is computed as your program runs, this is called dynamic memory allocation.
Local Variables - Local variables, and also return addresses from function calls and the saved state from interrupts are placed on a stack which starts from the top of RAM1 and grown downward. The amount of space for local variable is the portion of RAM1 not used by FASTRUN code and the initialized and zeroed variables.
Heap - Memory allocated by C++ "new" and C malloc(), and Arduino String variables are placed in RAM2, starting immediately after the DMAMEM variables.
External Heap - If PSRAM has been added, extmem_malloc() may be used to allocate this memory, started immediat

-sdfat
RTC RAM
16 bytes of memory are located within the RTC. If a coin cell is connected to VBAT, contents of this memory is preserved while power is off. This memory is accessed as 32 bit registers LPGPR0-LPGPR3.
SD Card
A built in SD socket allows you to use SD cards for large data storage. The Arduino SD library is used to access the card, using SD.begin(BUILTIN_SDCARD). This built in SD socket uses fast 4 bit native SDIO to access the card. SD cards may also be used via the SPI pins, with SD.begin(cspin), using the slower single bit SPI protocol to access the card.
SPI Flash
Flash memory chips may be added using the SPI pins. These are supported by the SerialFlash and LittleFS libraries.