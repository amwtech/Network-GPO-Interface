#ifndef _ETHERNET_GPO
#define _ETHERNET_GPO

/* ============================================================================
The Arduino microcontroller includes EEPROM memory. This appliation uses some
of that EEPROM to store operational parameters including the ethernet MAC 
address, the output polarity invert controls and four sets of IP addresses. 
The multiple address sets provide fast switching of IP configuration when 
the GPO interface is regularly used on different networks.

The following block of definitions set the first storage locations for each
block of information.

The MAC address, output inverts and active IP addresses can be user edited
by using a serial connection to a terminal program on a host computer.
============================================================================ */

#define _MAC_ADDRESS_START 0x00     // EEPROM address storing the MAC value
#define _MAC_ADDRESS_LENGTH 0x06

// Storage for the pin invert data
#define _OUTPUT_INVERT 0x10

// Each IP address set contains six sub-fields :-
// 1) IP address (4 bytes) 
// 2) Subnet-mask (4 bytes)
// 3) Router address (4 bytes)
// 4) Gateway address (4 bytes)
// 5) UDP listen Port (2 bytes)
// 6) Target UDP port address for status reporrts sent to the system that
//    issued the switch or status commands.
//
// The _IP_ADDRESS_XX values are the start address for each store in EEPROM.
#define IP_SETS   4
#define _IP_ADDRESS_01  0x20
#define _IP_ADDRESS_02  0x40
#define _IP_ADDRESS_03  0x60
#define _IP_ADDRESS_04  0x80

#define _MIN_LOGICAL_OUT 0

// Error Codes
#define _E_BAD_SET -1

// Define local function prototypes
void setPinModes(void);
int logical_to_physical(int logical_dest);
int read_IP_block(void);
int read_Status_Enable(void);
void fetchMACfromEeprom(byte* myMac);
int fetchIpEepromData(int whichSet);
void readInvertSettings(void);
void copyIPdataToEditable();
int writeIpEepromData(int whichSet);
void identApplication(void); 
String ipToString(IPAddress ip);
void printIPstack(byte* mac, IPAddress addr_ip, IPAddress addr_mask, IPAddress addr_gw, IPAddress addr_dns, unsigned int myPort, unsigned int statPort);
void parseOscMessage(int br, char* pktBuf);
bool parseToken(int startIndex, int *endIndex, int *sw_id, int *sw_val);
void parseStatusRequest(int br, char* pktBuf);
void sendStatusReport(void);
void reportSerialError(void);
void resetInputString(void);
bool checkIsDottedIP(String newIP);
byte ascii2hexval(byte up, byte lo);
bool parseMACaddress(String newmac);
bool parseInvert(String invert);
void parseInputString(void);

#endif
