Secure Firmware Update
When locked, Teensy 4.1 can be reprogrammed using encrypted EHEX files. EHEX files may be published to allow anyone to securely update firmware. Your private key is needed only to create the EHEX file, which then may be loaded onto the hardware by people without access to your key. Once locked, Teensy can only be programmed by EHEX files created using your key.
Program Memory Protection
Code security protects your program code from unauthorized access and coping. When compiling, your program is encrypted. When run, the IMXRT Bus Encryption Engine provides on-the-fly decryption as your program executes. If an attacker removes and reads the flash memory chip from Teensy 4.1, or attempt to capture the USB communication from Teensy Loader, or copies the EHEX file Teensy Loader opens, they get only an encrypted copy of your program.

Secure Firmware Update
Users can be given an EHEX file and Teensy Loader to securely update commercial products or secure applications which embed a Lockable Teensy, without gaining access to the original program code. Of course the key already in its fuse memory and secure mode locked when the product is shipped.
Permanent Secure Mode
Brand new Teensy 4.1 can only run unencrypted programs. Once a key is written into fuse memory, either encrypted or unencrypted programs can run. Secure mode permanently disables the ability to run unencrypted code, and activates hardware security features.

Lockable Teensy - White Lock Stamp
Lockable Teensy
Secure mode can only be activated on Lockable Teensy. While Standard and Lockable Teensy are identical hardware, the permanent fuse configuation differs. Standard Teensy does not allow changes to fuses affecting boot or other critical configuration. Standard Teensy is meant to safe from "bricking" by programs which could write to fuse memory, but this safety means secure mode can not be activated. Standard Teensy can have a key written and can run encrypted code, but encryption alone is not fully secure. Only Lockable Teensy provides proper code security, and only when a key is written and secure mode is locked.
Authentication
The encryption process includes digital signature authentication. In secure mode, this signature is checked before any code can be decrypted.
JTAG Disable
Secure mode permanently disables the JTAG port. To enter programming mode without JTAG, Teensy Loader and the EHEX file automatically utilize a loader utility which is authenticated by your key's digitial signature, and in turn uses secure hash checks to fully authenticate all components of the programming process.
EHEX File Format
Teensyduino packages your encrypted code, metadata, a startup shim, loader utility, digital signatures and other essential details into a single EHEX file. This EHEX may be given to customers or untrusted parties to perform code updates with the convenice of a single file. The EHEX format and encryption details are documented on the code security page
Key Management
To make creating and using your key simple, Teensyduino adds a "Teensy 4 Security" window to the Arduino Tools menu. These functions can also be accessed from a command line utility for use from non-Arduino tools or automated scripts.