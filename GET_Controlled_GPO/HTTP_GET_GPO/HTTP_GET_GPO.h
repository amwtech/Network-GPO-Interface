#ifndef _HTTP_GET_GPO_UNO
#define _HTTP_GET_GPO_UNO

// The Arduino UNO processor includes EEPROM memory. This appliation 
// uses some locations in the EEPROM to store operational parameters including
//    a) the ethernet MAC address,
//    b) the output polarity invert
//    c) four sets of IP addresses (IP, gateway, DNS, netmask per set). 
//
// The multiple address sets support fast switching of IP addresses when the 
// GPO interface has to be moved between commonly used networks. The selection
// of the address set uses 2 input analogue pins (A0 and A1) on the Arduino.
//
// The following definitions set the first storage EEPROM location for each
// block of information.
//
// The MAC address, output inverts and active IP addresses can be user edited
// using a USB serial connection to a terminal program on a host computer.
#define _MAC_ADDRESS_START 0x00
#define _MAC_ADDRESS_LENGTH 0x06

// Storage for the output state invert data. Inverting the output sense enables
// relays that are active when the control pin is low to be activated when the
// user set level is high.
#define _OUTPUT_INVERT 0x10

// Each IP address set has five sub-fields - IP address (4 bytes), Subnet-mask 
// (4 bytes), Router (4 bytes), Gateway address and UDP Port (2 bytes).
// The _IP_ADDRESS_XX values are the start address for each store in EEPROM.
#define IP_SETS 4
#define _IP_ADDRESS_01  0x20
#define _IP_ADDRESS_02  0x40
#define _IP_ADDRESS_03  0x60
#define _IP_ADDRESS_04  0x80

#define _MIN_LOGICAL_OUT 0

// Error Codes
#define _E_BAD_SET -1

// =================================
// Define local functions prototypes
void setPinModes(void);
int logical_to_physical(int logical_dest);
int read_IP_block_ID(void);
void readMACfromEeprom(byte* myMac);
int readIpEepromData(int whichSet);
void readInvertSettings(void);
void copyIPdataToEditable();
int writeIpEepromData(int whichSet);
void identApplication(void); 
String ipToString(IPAddress ip);
void printIPstack(byte* mac, IPAddress addr_ip, IPAddress addr_mask, IPAddress addr_gw, IPAddress addr_dns, unsigned int myPort);
void parseOscMessage(int br, char* pktBuf); // REPLACE
void reportSerialError(void);
void resetInputString(void);
bool checkIsDottedIP(String newIP);
byte ascii2hexval(byte up, byte lo);
bool parseMACaddress(String newmac);
bool parseInvert(String invert);
void parseInputString(void);

#endif

