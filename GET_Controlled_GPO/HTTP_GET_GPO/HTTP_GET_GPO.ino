// ======================================
// Program Versions:
// 1.0    Initial Release

/* ============================================================================
This sketch implements a GPO (General Purpose Output) interface where the state
of each output is set using an HTTP GET message sent to the inferface processor. 
The outputs of the interface module are typically via low operating-voltage 
relays that provide isolation barriers between controlling and controlled 
devices.

Target Hardware:
================
Arduino Model:    UNO R3
Ethernet Shield:  WizNet based
Ethernet Library: Ethernet

Relays: 5V coil with protection diodes. Select small signal or mains 
switching capable as needed for the deployed application.

NOTE:
An ENC28J60 network interface implements much of the IP stack in software, 
using a lot of both flash and RAM. Hence this sketch can NOT be adapted to 
run on a NANO with ENC28J60 shield.

The is limited code and RAM memory in the Arduino UNO requires the HTTP client
to send slightly terse commands to the interface. An interface with more 
complex parsing of the message can be acheived using an Arduino MEGA 
or one of the newer 32-bit processor Arduino devices.

The switch commands are sent as part of the GET command. For example:

  GET /gpiswitch/out?s1=1&s2=0&s3=1&s4=0

where s1 is switch/output/relay number 1 and it is switched to ON by s1=1 and OFF
by s1=0. Up to eight outputs are supported by this module, but only the elements
that require updated output states are included in the GET path line. Because all
8 switches can be state defined in a single message there should be no issues with 
the limited number of client sessions (four) supported by an UNO ethernet module.

When a pulsed output is required, the controlling client must send two messages, 
one switching the output on, the second switching the output off after the 
required pulse duration time.

The Arduino UNO pins used for outputs are:
Relay Number   1   2   3   4   5   6   7   8
Pin Number     2   3   5   6   7   8   9  17 
Designation   D2  D3  D5  D6  D7  D8  D9  A3

** Digital pin 4 is used by the WizNet Ethernet Shield (flash card).

Because multiple GPO interfaces may be present on the same IP network care 
must be taken to ensure the MAC adddress and the IP address are not duplicated. 
Some GPO interfaces may need to be deployed on multiple networks. The MAC 
address for the board and up to 4 sets of IP properties are stored in the 
on-board EEPROM memory. Analogue inputs A0 and A1 are designated as IP address 
set selectors. With no pin connections made to A0 and A1 the first of the four
IP address sets is used.

Analogue pin A2 is used as a digital input. If this returns a logic 1
the USB serial link is enabled. If the pin is open-circuit the signalled 
value is 0, and the USB is only used for power. This prevents a broken 
serial port causing a lock-up in the Arduino when the serial buffer fills.

The elements of the active IP address set can be updated via a USB serial 
connection. The serial link must be enabled before boot by taking pin A2 low. 
Each input command is terminated by a line feed <lf> character. The input 
processing ignores any carriage return characters on the link, so the 
terminal that sends the commands can use <lf> only, <cr><lf>, or <lf><cr> 
as the line terminate.

The implemented commands are (all lower case text only):

show
set ip <IP dotted string>
set mask <network mask dotted string>
set gw <network gateway ip dotted address>
set dns <dns ip dotted address>
set port <local udp listen port number>
set mac <6 hex value with colon seperators>
set invert <string of eight 0/1>
save

Only send the save command after the other values are set.

The address set stored is the one defined by the state of A0 and A1 at 
the last processor reboot.

Copyright 2023 Andy Woodhouse

Andy Woodhouse
andy@amwtech.co.uk

Acknowledgements
================
Thanks to those who have contributed to the various libraries such as the 
Ethernet libraries and released them for others to use.

Licence: ISC
=======
Permission to use, copy, modify, and/or distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright notice 
and this permission notice appear in all copies.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH 
REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, 
INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM 
LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR 
PERFORMANCE OF THIS SOFTWARE.
============================================================================== */

#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include "HTTP_GET_GPO_UNO.h"

// Define the maximum number of outputs - eight for Arduino UNO. 
#define MAXGPO 8

// Universally accessible constants and variables.
// ===============================================
// The values in l2p are the map to convert a logical output (0..7) to the 
// Arduino pin number.
//
// Some relays boards have active low drives, others have active high drive. 
// This program assumes active high relays. Array pinInvert is edited to 
// toggle the high/low polarity for each output pin. The bit invert 
// properties are read from EEPROM at reboot and apply to all address sets.

const int l2p[MAXGPO] = { 2, 3, 5, 6, 7, 8, 9, 17 };
bool pinInvert[MAXGPO] = { false, false, false, false, false, false, false, false };

// Define variables that hold the working MAC address and IP address.
// The working values are read from EEPROM sduring startup.
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress my_ip = { 0, 0, 0, 0 };
IPAddress my_dns = { 0, 0, 0, 0 };
IPAddress my_gw = { 0, 0, 0, 0 };
IPAddress my_mask = { 0, 0, 0, 0 };
unsigned int localPort = 0;
int ipBlock = 0;    // Holds ID that controls which set of IP addresses are used.

// Define variables used to input and save operating properties into EEPROM.
byte new_mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress new_ip(0, 0, 0, 0);
IPAddress new_dns(0, 0, 0, 0);
IPAddress new_gw(0, 0, 0, 0);
IPAddress new_mask(0, 0, 0, 0);
unsigned int newPort = 0;
bool newPinInvert[MAXGPO] = { false, false, false, false, false, false, false, false }; 

const int ip_Data_Starts[IP_SETS] = {_IP_ADDRESS_01, _IP_ADDRESS_02, _IP_ADDRESS_03, _IP_ADDRESS_04};

EthernetServer server = EthernetServer(0);  // Make a default server. Port address updated during boot.

bool printEnabled = false;    // True if serial interface enabled. Connect pin A2 to GND to enable serial.
#define MAXSTRINGINPUT 42
String inputString = "";      // String to hold incoming serial data
bool stringComplete = false;  // If the serial input string complete and ready to parse?

// Define variables for receiving and parsing the GET request line
#define GETCOMMANDMAX 100
char getRequestBuffer[GETCOMMANDMAX];
int  nextCharPosn = 0;
bool isFirstLine = true;

// Module wide available variables
int outState[MAXGPO] = {0, 0, 0, 0, 0, 0, 0, 0} ;
int nextState[MAXGPO] = {-1, -1, -1, -1, -1, -1, -1, -1};
char sysReport[] = "{\"s1\":0,\"s2\":0,\"s3\":0,\"s4\":0,\"s5\":0,\"s6\":0,\"s7\":0,\"s8\":0}";


// ========================================================================= //
// Service/support functions                                                 //
// ========================================================================= //

/*!
@brief void setPinModes(void) configures the input and output pins needed to
read control modes and output switch command states.
*/
void setPinModes(void)
{
  // Set our output pins
  for (int i = _MIN_LOGICAL_OUT; i < _MIN_LOGICAL_OUT + MAXGPO; i++)
  {
    pinMode(logical_to_physical(i) , OUTPUT);
    digitalWrite(logical_to_physical(i), LOW);  // Set relay control to 'off'
  }

  // Set A0, A1 and A2 lines as inputs with internal pullup.
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(4, OUTPUT);        // Disable micro SD card interface
  digitalWrite(4, HIGH); 
}


/*!
@brief Convert the logical_dest into an output pin number.

@param logical_dest Integer in range 0 to 7
*/
int logical_to_physical(int logical_dest)
{
  if ((logical_dest < _MIN_LOGICAL_OUT) || (logical_dest >= MAXGPO)) {
    return -1;
  }

  return l2p[logical_dest];
}

/*!
@brief Read the IP address set to use from input pins A1 and A0 (A0 is lsb)
Note active pullup is used, so unconnected pin reports high. Invert read state to get true value.

@return Integer in range 0 to 3
*/
int read_IP_block_ID(void)
{
  int select = 0;
  select += (digitalRead(A0) == 1) ? 0 : 1;
  select += (digitalRead(A1) == 1) ? 0 : 2;

  return select;
}


/*!
@brief Read the MAC address from EEPROM store

@param[in] myMac pointer to array of 6 bytes that store the MAC address value.
*/
void readMACfromEeprom(byte* myMac)
{
  for (int i = _MAC_ADDRESS_START; i < _MAC_ADDRESS_START + _MAC_ADDRESS_LENGTH; i++) {
    myMac[i - _MAC_ADDRESS_START] = EEPROM.read(i);
  }
}


/*!
@brief Reads an IP data set from EEPROM.

@param[in] whichSet The address set to read from EEPROM. Range 0 to 3
@return 0 for sucessful read or an _E_BADxxx code on error.
*/
int readIpEepromData(int whichSet)
{
  int addr1 = 0;
  int k;

  if ((whichSet < 0)  || (whichSet >= IP_SETS)) {
    return(_E_BAD_SET);
  }

  addr1 = ip_Data_Starts[whichSet]; // Get first address of data block
  for (k = 0; k < 4; k++) my_ip[k] = EEPROM.read(addr1++);
  for (k = 0; k < 4; k++) my_dns[k] = EEPROM.read(addr1++);
  for (k = 0; k < 4; k++) my_mask[k] = EEPROM.read(addr1++);
  for (k = 0; k < 4; k++) my_gw[k] = EEPROM.read(addr1++);
  localPort = ((unsigned int)EEPROM.read(addr1++) * 256) + (unsigned int)EEPROM.read(addr1++);

  return 0;
}


/*!
@brief Read output invert control settings from EEPROM memory. Values stored in global array pinInvert[].

*/
void readInvertSettings(void)
{
  int addr1 = _OUTPUT_INVERT; // Start of invert data in EEPROM

  for (int ix = 0; ix < MAXGPO; ix++) {
    pinInvert[ix] = EEPROM.read(addr1++) != 0 ? true : false;
  }
}


/*!
@brief Copy active IP address set properties into the information edit arrays.

*/
void copyIPdataToEditable(void) {
  int ix;
  for (ix = 0; ix < MAXGPO; ix++) newPinInvert[ix] = pinInvert[ix];
  for (ix = 0; ix < 6; ix++) new_mac[ix] = mac[ix];
  for (ix = 0; ix < 4; ix++) new_ip[ix] = my_ip[ix];
  for (ix = 0; ix < 4; ix++) new_dns[ix] = my_dns[ix];
  for (ix = 0; ix < 4; ix++) new_gw[ix] = my_gw[ix];
  for (ix = 0; ix < 4; ix++) new_mask[ix] = my_mask[ix];
  newPort = localPort;
}


/*!
@brief Write edited IP set values values into EEPROM.

@param[in] whichSet The set number where the IP address properties are stored.
@return 0 for success or a negative _E_BAD_xxx code.
*/
int writeIpEepromData(int whichSet) {
  int addr1 = 0;
  int k;

  if ((whichSet < 0)  || (whichSet >= IP_SETS)) {
    return(_E_BAD_SET);
  }

  // Start by copying the MAC. Once correctly set this rarely changes.
  for (int i = _MAC_ADDRESS_START; i < _MAC_ADDRESS_START + _MAC_ADDRESS_LENGTH; i++) {
      EEPROM.update(i, new_mac[i - _MAC_ADDRESS_START]);
  }

  // Save the output invert data
  addr1 = _OUTPUT_INVERT;
  for (int i = 0; i < MAXGPO ; i++) {
    EEPROM.update(addr1, newPinInvert[i] ? 1 : 0);
    addr1++;
  }

  addr1 = ip_Data_Starts[whichSet];  // Point at the start of the EEPROM store for the selected address set
  
  for (k = 0; k < 4; k++) {
    EEPROM.update(addr1, new_ip[k]);
    addr1++;
  }
  for (k = 0; k < 4; k++) {
    EEPROM.update(addr1, new_dns[k]);
    addr1++;
  }
  for (k = 0; k < 4; k++) {
    EEPROM.update(addr1, new_mask[k]);
    addr1++;
  }
  for (k = 0; k < 4; k++) {
    EEPROM.update(addr1, new_gw[k]);
    addr1++;
  }
  byte portHi, portLo;
  portHi = (byte)(newPort / 256);
  portLo = (byte)(newPort % 256);
  EEPROM.update(addr1, portHi);
  addr1++;
  EEPROM.update(addr1, portLo);

  return 0;
}


/*!
@brief identApplication is called to print the Name and key operating properties of the IP interface

*/
void identApplication(void)
{
  if (!printEnabled) return;
  Serial.println("HTTP GET GPO Interface");
  Serial.print("Using IP Address set ");
  Serial.println(ipBlock);
  printInvertSet();  // State of inverts array
  printIPstack(mac, my_ip, my_mask, my_gw, my_dns, localPort);
}


/*!
@brief Return the IPAddress as a dotted address string.

@param[in] ip Address of the byte array to convert to text.
@return string (Type defined in Arduino header files).
*/
String ipToString(IPAddress ip) {
  String ipa = "";
  ipa += String(ip[0]) + ".";
  ipa += String(ip[1]) + ".";
  ipa += String(ip[2]) + ".";
  ipa += String(ip[3]);

  return ipa;
}


/*!
@brief Display values stored in the working or edit set of address values on serial USB serial link. 
All values are passed to this function as parameters.

@param[in] my_mac Address of byte array that contains the MAC id
@param[in] addr_ip IPv4 address of the GPO
@param[in] addr_mask IPv4 network mask
@param[in] addr_gw IPv4 gateway address
@param[in] addr_dns IPv4 dns server address
@param[in] myPort Port number listening for http requests
*/
void printIPstack(byte* my_mac, IPAddress addr_ip, IPAddress addr_mask, IPAddress addr_gw, IPAddress addr_dns, unsigned int myPort)
{
  byte hexVal[3];
  byte hexChar[] ="0123456789ABCDEF";
  int addrLen = 0;

  if (!printEnabled) return;  // Only print when serial link is enabled

  // Show the mac address in use.
  hexVal[2] = '\0';
  Serial.print(" MAC ");
  for (int j = 0; j < 6; j++) {
    hexVal[0] = hexChar[(my_mac[j] >> 4) & 0x0f];
    hexVal[1] = hexChar[my_mac[j] & 0x0f];
    Serial.write(hexVal, 2);
    if (j != 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  // Show the IP address and port number
  Serial.print("  IP ");
  Serial.print(ipToString(addr_ip));
  Serial.print("  port ");
  Serial.println(myPort);

  // Show the netmask
  Serial.print("Mask ");
  Serial.println(ipToString(addr_mask));

  // Show the gateway
  Serial.print("  GW ");
  Serial.println(ipToString(addr_gw));
  // Show the dns
  Serial.print(" DNS ");
  Serial.println(ipToString(addr_dns));
}


/*!
@brief Report a problem with an input value, then reset serial input ready for new command

*/
void reportSerialError(void) {
  Serial.println("Error");  // Print message
  resetInputString();       // Flush input queue
}


/*!
@brief Flush the serial input string receive

*/
void resetInputString(void) {
  inputString = "";
  stringComplete = false;
}


/*!
@brief Check if the string passed as a parameter is a valid dotted IPv4 address

@param[in] myInput Pointer to string array that is tested
@return True for valid address, else false if an issue.
*/
bool checkIsDottedIP(String myInput) {
  IPAddress apip;   // Used in checking input is valid IP string

  return apip.fromString(myInput);
}


/*!
@brief Convert two upper case ASCII characters to an unsiged 8-bit unsigned integer (byte)

@param[in] up Character that is the value of the most significant nibble
@param[in] lo Character that is the value of the least significant nibble
@return the converted 8-bit value
*/
byte ascii2hexval(byte up, byte lo) {
  byte a, b;
  a = (up >= 'A') ? (up - 'A' + 10) << 4 : (up - '0') << 4 ;
  b = (lo >= 'A') ? (lo - 'A'+ 10): (lo - '0');
  return a+b; 
}


/*!
@brief Parse a character string passed as the parameter to check it is formatted as a 
6-byte MAC address that is 17 bytes long with colons at character positions 2, 5, 8, 11 and 14,
and that all other characters are hexadecimal digits.

@param[in] newmac Pointer to the array of bytes to test
@return True if the string array is a valid MAC address format.
*/
bool parseMACaddress(String newmac) {
  newmac.toUpperCase();
  if (newmac.length() != 17) return false;  // Invalid length
  const char* buf = newmac.c_str();  // Get access to character array
  // Check for the seperators
  bool haveSep = buf[2] == ':' && buf[5] == ':' && buf[8] == ':' && buf[11] == ':' && buf[14] == ':';
  if (!haveSep) return false;
  // Check for hexadecimal characters in other positions
  bool haveHexDigits = true;
  for (unsigned int ix = 0; ix < 17 ; ix += 3) {
    haveHexDigits = haveHexDigits && isHexadecimalDigit(buf[ix]);
    haveHexDigits = haveHexDigits && isHexadecimalDigit(buf[ix+1]);
  }
  if (!haveHexDigits) return false;

  Serial.println(newmac);
  Serial.write(buf,17); Serial.println();
  // Convert the values to bytes that are stored in variable new_mac
  int idx;
  for (int k = 0; k < 6 ; k++) {
    idx = k * 3;
    new_mac[k] = ascii2hexval(buf[idx], buf[idx+1]);
  }
  return true;
}


/*!
@brief Checks the provided parameter is a valid set of data for the relay output drive 
invert property. The string should be 8 characters long, and only have'0' or '1' in each slot.
The first character is control for relay 8, the last for relay 1

@param[in] invert String with 8 characters that is to be tested
@return True if the array has valid syntax
*/
bool parseInvert(String invert) {
  if (invert.length() != MAXGPO) return false;

  const char* buf = invert.c_str();
  bool just01 = true;
  for (int i = 0; i < MAXGPO ; i++) {
    just01 = just01 && ((buf[i] == '0') || (buf[i] == '1'));
  }
  if (!just01) return false;
  // Swap the data around so that the last digit of the string stores in invert array position 0
  for (int i = 0; i < MAXGPO; i++) {
    newPinInvert[i] = (buf[MAXGPO - 1 - i] == '1') ? true : false;
  }
  
  return true;
}


/*!
@brief Print the newPinInvert string global variable to the USB serial link.

*/
void printInvertSet(void)
{
  if (!printEnabled) return;

  Serial.print("Invert control ");
  for (int i = 0; i < MAXGPO; i++) {
    Serial.print( newPinInvert[MAXGPO - 1 - i] ? '1' : '0');
  }
  Serial.println();
}


/*!
@brief Process a string from the serial USB link stored in global variable inputString to check for a command that updates an operational property.

*/
void parseInputString(void)
{
  if (!printEnabled) return;

  inputString.trim(); // Remove leading and trailing whitespace
  // Check for a "save" request?
  if ((inputString.length() == 4) && (inputString == "save")) {
    // Save the address set to the EEPROM in currently active set
    if (writeIpEepromData(ipBlock) == 0) 
      Serial.println("Saved new values to flash");
    else
      Serial.println("ERROR - failed to save to flash");
    resetInputString();
    return;
  }

  if ((inputString.length() == 4) && (inputString == "show")) {
    Serial.println();
    Serial.println("Edited properties:");
    printInvertSet(); 
    printIPstack(new_mac, new_ip, new_mask, new_gw, new_dns, newPort);
    resetInputString();
    return;
  }

  if (inputString.startsWith("set ")) {
    // Remove the "set " string from the start of the inputString
    inputString.remove(0, 4);

    // We hope to have two parts left the "verb" and the "property_value" separated by a space. If not, we have an error.
    int verbLength = inputString.indexOf(' ');
    if (verbLength == -1) {
      reportSerialError();// Report error and exit
      return;
    }

    // Select the processing method for valid verbs
    String myVerb = inputString.substring(0, verbLength);
    String myProperty = inputString.substring(verbLength+1);

    if (myVerb == "mac") {
      if (!parseMACaddress(myProperty)) {
        reportSerialError();
        return;
      }
    }
    else if (myVerb == "invert") {
      if (!parseInvert(myProperty)) {
        reportSerialError();
        return;
      }
    }
    else if (myVerb == "ip") {
      if (!checkIsDottedIP(myProperty)) {
        reportSerialError();
        return;
      }
      new_ip.fromString(myProperty);
    }
    else if (myVerb == "dns") {
      if (!checkIsDottedIP(myProperty)) {
        reportSerialError();
        return;
      }
      new_dns.fromString(myProperty);
    }
    else if (myVerb == "mask") {
      if (!checkIsDottedIP(myProperty)) {
        reportSerialError();
        return;
      }
      new_mask.fromString(myProperty);
    }
    else if (myVerb == "gw") {
      if (!checkIsDottedIP(myProperty)) {
        reportSerialError();
        return;
      }
      new_gw.fromString(myProperty);
    }
    else if (myVerb == "port") {
      long pv = myProperty.toInt();
      if ((pv < 0) || (pv > 65535)) {
        reportSerialError();
        return;
      }
      else {
        newPort = (unsigned int)pv ;
      }
    }
    else {
      // Invalid input - Report as error
      reportSerialError();
      return;
    }

    resetInputString();

    // Print the current edited space
    Serial.println();
    Serial.print("Edited values for set "); Serial.println(ipBlock);
    printInvertSet();  // State of inverts array
    printIPstack(new_mac, new_ip, new_mask, new_gw, new_dns, newPort);
  }
  else {
    resetInputString();
  }
}


/*!
@brief Look for a match between provided http verb and content of the global variable getRequestBuffer

@param[in] verb pointer to start of string with an HTTP verb 
@param[in] checkLen Length of buffer (number of characters) to test.
@result True if the passed verb matches start of getRequestBuffer
*/
bool verbMatch(const char* verb, int checkLen) {
  bool match = true;
  for (int i = 0; i < checkLen; i++) {
    if (verb[i] != getRequestBuffer[i]) match = false;
  }
  return match;
}

// Define the process response codes
#define RQ_BAD_LEN 0
#define RQ_TYPE_HEAD 1
#define RQ_TYPE_GET 2
#define RQ_TYPE_OTHER 3


/*!
@brief Checks the getRequestBuffer for one of the two supported HTTP command verbs - "HEAD " and "GET ".

@return Index code for the type of HTTP request - RQ_BAD_LEN, RQ_TYPE_HEAD, RQ_TYPE_GET, RQ_TYPE_OTHER
*/
int parseGetRequest(void) {
  int rqLen = 0;
  while (getRequestBuffer[rqLen] != '\0') rqLen++;  // Check the request length

  if (rqLen < 14) return RQ_BAD_LEN ; // Insufficient characters in the string for valid get request

  if (verbMatch("HEAD ", 5)) return RQ_TYPE_HEAD;

  if (verbMatch("GET ", 4)) return RQ_TYPE_GET;

  return RQ_TYPE_OTHER;
}


/*!
@brief Removes the verb, spaces and "http/1.1" content from character array getRequestBuffer[].

@return The length of the address path at the start of getRequestBuffer
*/
int trimToPath(void) {
  int i=0;  // A general index/counter
  int ix=0; // Holds the position of 1st space character

  while (getRequestBuffer[ix++] != ' '); // Stops with ix pointing at character after 1st space
  while ((getRequestBuffer[i] = getRequestBuffer[i+ix]) != '\0') i++; // Copy path over the verb
  i = 0;
  while (getRequestBuffer[i] != ' ') i++; // Look for space character after the path
  getRequestBuffer[i] = '\0'; // i holds length of the string

  return i;
}


/*!
@brief Look for the first instance of a character passed as parameter 2 in buffer getRequestBuffer[] 
starting at character index defined in the first parameter

@param[in] start Index in buffer where search starts
@param[in] letter The character to find
@return The index of the character in getRequestBuffer, or -1 if character is not present.
*/
int findChar(int start, char letter) {
  int i = start;
  while ((getRequestBuffer[i] != '\0') && (getRequestBuffer[i] != letter)) i++;
  return getRequestBuffer[i] == '\0' ? -1 : i;
}


/*!
@brief Compare string passed as parameter against path address in getRequestBuffer[]

@param[in]  refstring Pointer to string to be tested
@return True if the path address matches the test string
*/
bool checkMatch(const char *refstring) {
  int chkLen = 0;
  int pathLen = 0;
  bool haveMatch = true;

  while (refstring[chkLen++] != '\0') ;         // Compute length of reference string
  while (getRequestBuffer[pathLen++] != '\0');  // Get length of path stored in getRequestBuffer
  
  if (chkLen != pathLen) return false;
  for (int i = 0; i < chkLen; i++) {
    if (refstring[i] != getRequestBuffer[i]) haveMatch = false;
  }
  return haveMatch;
}


/*!
@brief Check if the path string in getRequestBuffer starts with the provided parameter reference string?

@param[in] refstring The string to look for at the start of the getRequestBuffer.
@return True if the getRequestBuffer starts with the string passed as a parameter
*/
bool checkStartsWith(const char *refstring) {
  int chkLen = 0;
  int pathLen = 0;
  bool haveMatch = true;

  while (refstring[chkLen] != '\0') chkLen++;          // Compute length of reference string without '\0'
  while (getRequestBuffer[pathLen] != '\0') pathLen++;  // Get length of path stored in getRequestBuffer without '\0'
  if (pathLen < chkLen) return false;           // Path string is too short, so "not start with"
  for (int i = 0; i < chkLen; i++) {
    if (refstring[i] != getRequestBuffer[i]) haveMatch = false;
  }

  return haveMatch;
}


/*!
@brief Update the stringified JSON response with the current state of the output switches.

*/
void updateStatusMessage(void) {
  int j;

  for (int idx = 1; idx <= MAXGPO; idx++) {
    j = idx - 1;
    sysReport[(j * 7) + 6] = outState[j] == 0 ? '0' : '1';
  }
}


/*!
@brief Support function that extracts a switch control sub-string from the GET request. 
It updates the parameters used for the next token extract.

@param[in] startindex The start position of a token in the buffer, for example the index of the '?' or  '&' at the head of the token.
@param[out] *endindex The index of the next token in the request string, or -1 if no more tokens.
@param[out] *sw_id The switch number extracted from the token, or -1 if the switch number format is invalid.
@param[out] *sw_val The OFF or ON state defined by the token, or -1 for a badly formatted token. 
@return True if a valid token was found in the request, false if an error.
*/
bool parseToken(int startIndex, int *endIndex, int *sw_id, int *sw_val) {
  int idx1 = startIndex + 1;
  int idx2 = startIndex + 2;

  // Test 1 - does the token content start with an 's' or 'S'. If not return an error (false).
  if (!(getRequestBuffer[idx1] == 's') || (getRequestBuffer[idx1] == 'S')) {
    *endIndex = -1;
    *sw_id = -1;
    *sw_val = -1;
    return false;
  }

  // Test 2 - is there an '=' separator?
  while ((getRequestBuffer[idx2] != '=') && (getRequestBuffer[idx2] != '\0')) idx2++;
  if (getRequestBuffer[idx2] == '\0') {
    // No field seperation token identified. Return false
    *endIndex = -1;
    *sw_id = -1;
    *sw_val = -1;
    return false;
  }

  // Test 3 - are there one or more characters between the 's' and the '='? 
  if ((idx2 - idx1) < 2) {
    *endIndex = -1;
    *sw_id = -1;
    *sw_val = -1;
    return false;    
  }

  // Test 4 - are the characters between the 's' and '=' decimal characters?
  for (int i = idx1+1; i < idx2; i++) {
    if ((getRequestBuffer[i] < '0') || (getRequestBuffer[i] > '9')) {
      *endIndex = -1;
      *sw_id = -1;
      *sw_val = -1;
      return false;
    }
  }

  // Test 5 - Character after the '=' is either '0' or '1'
  if (!((getRequestBuffer[idx2+1] == '0') || (getRequestBuffer[idx2+1] == '1'))) {
    *endIndex = -1;
    *sw_id = -1;
    *sw_val = -1;
    return false;
  }

  // Test 6 - is the character after the value a '&' or a '\0'.
  if (!((getRequestBuffer[idx2+2] == '&') || (getRequestBuffer[idx2+2] == '\0'))) {
    *endIndex = -1;
    *sw_id = -1;
    *sw_val = -1;
    return false;
  }

  // All tests passed so compute values and return values
  int sw = 0;

  *endIndex = (getRequestBuffer[idx2+2] == '\0') ? -1 : idx2+2;
  for (int i = idx1+1; i < idx2; i++) {
    sw = sw * 10;
    sw = sw + (getRequestBuffer[i] -'0');
  }
  *sw_id = sw;
  *sw_val = getRequestBuffer[idx2+1] - '0';  
  return true;
}


// ========================================================================= //
// Arduino setup and loop functions                                          //
// ========================================================================= //

/*!
@brief The code run once during boot to initialise the Arduino application.

*/
void setup(void) {

  setPinModes(); // Setup the input and output pin modes used by this application
  ipBlock = read_IP_block_ID();  // Read the IP address set from A0 and A1 pins
  printEnabled = (digitalRead(A2) == 1) ? false : true; // Read the serial enable pin, low to enable.

 // Initialise the serial link if required for operational use. (When pin A2 is a low voltage/earthed)
  if (printEnabled) {
    Serial.begin(9600);
    while (!Serial) {
      ; // Wait for serial port to connect. Used by USB port only.
    }
  }

  readMACfromEeprom(mac);
  readIpEepromData(ipBlock);
  readInvertSettings();
  copyIPdataToEditable();
  inputString.reserve(MAXSTRINGINPUT); // Define maximum buffer for serial character input

  // Ident application and properties on console if serial port is enabled.
  identApplication(); 

  // Set the outputs to 'off' allowing for the pinInvert[] array setting
  for (int i = 0; i < MAXGPO; i++) {
    digitalWrite(logical_to_physical(i), pinInvert[i]);
  }

  // Start the Ethernet object using our properties read from EEPROM
  Ethernet.begin(mac, my_ip, my_dns, my_gw, my_mask);
  
  // Check for presence of Ethernet hardware
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    if (printEnabled) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    }
    while (true) {
      delay(10); // do nothing, no point running without Ethernet hardware
    }
  }

  if (Ethernet.linkStatus() == LinkOFF) {
    if (printEnabled) {
      Serial.println("Ethernet cable is not connected.");
    }
  }

  server = EthernetServer(localPort);   // Prepare our server on user set port number

  server.begin(); // Listen for TCP packets
  if (printEnabled) {
    Serial.print("Server has been started on port "); Serial.print(localPort);
    Serial.println();
  }
}


/*!
@brief The code that forms the Arduino Application loop.

*/
void loop(void) {
  char r1='\0', r2='\0', r3='\0', r4='\0'; // Used in detecting blank line at end of header
  bool endOfHeader = false;
  int repType = 0;

  EthernetClient client = server.available();

  if (client) {
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();

        if (isFirstLine) {
          if ((c != '\r') && (c != '\n')) {
            // Save the character into the string buffer, but use safety
            // length test on buffer content;
            if (nextCharPosn < GETCOMMANDMAX - 1) {
              getRequestBuffer[nextCharPosn++] = c;
            }
          }
          if (c == '\n') {
            getRequestBuffer[nextCharPosn] = '\0';  // Terminate the string 
            isFirstLine = false;
          }
        }

        // Move received characters along 4-character shift register.
        r1 = r2; r2 = r3; r3 = r4; r4 = c;

        endOfHeader = (r1 == '\r') && (r2 == '\n') && (r3 == '\r') && (r4 == '\n');

        if (endOfHeader) {
          // End of header detected - parse our input line and send a response.
          int myType = parseGetRequest(); // Test for the type of action requested
          int pathLen = 0;
          int paramStart = 0;

          if (myType == RQ_TYPE_GET) {
            // Need to examine the detail of the request and process accordingly. 
            // Remove the "GET " and " HTTP/1.1" from the process buffer
            pathLen = trimToPath();
            
            // Find the start of the parameters, indicated by a '?'
            paramStart = findChar(0, '?');
            if (paramStart < 0) {
              // No parameters present. Look to see if we have a string that is "/", or "/gpiswitch/out" or or "/favicon.ico"
              if (checkMatch("/") || checkMatch("/gpiswitch/out")) {
                repType = 1;
              }
              else if (checkMatch("/favicon.ico")) {
                repType = 3;
              }
              else {
                repType = 3;
              }
            }
            else {
              // There is a ? in the request. Does the request have the correct initial path element?
              if (checkStartsWith("/gpiswitch/out")) {
                // Parse the request
                int ts = paramStart;
                int tn = 0;
                int swID = 0;
                int swVal = 0;
                bool goodToken = false;
                bool swState = false;

                do {
                  goodToken = parseToken(ts, &tn, &swID, &swVal);
                  ts = tn;  // Prepare for next token parse
                  if (goodToken) {
                    // Update the switch update array if the switch ID is a valid value
                    if ((swID > 0) && (swID <= MAXGPO)) {
                      nextState[swID-1] = swVal;
                    }
                    else {
                      goodToken = false;  // Because used out of range switch id
                    }
                  }
                } 
                while (goodToken && (tn > 0));

                // goodToken is false if the parse had an error. Update if goodToken is TRUE
                if (goodToken) {

                  // Update the output pins
                  for (int j = 0; j < MAXGPO; j++) {
                    if (nextState[j] >= 0) {
                      // Update the pin
                      swState = (nextState[j] == 1) ;
                      swState = (pinInvert[j] == 1) ? !swState : swState;
                      digitalWrite(logical_to_physical(j) , swState ? 1 : 0);
                      // Update the output state record array
                      outState[j] = nextState[j];
                    }
                  }
                }
                // Clear the state changes array
                for (int j = 0; j < MAXGPO; j++) nextState[j] = -1;

                // Return a final response with status data
                repType = goodToken ? 1 : 3;
              }
              else {
                // Bad initial path
                repType = 3;
              }
            }
          }

          else if (myType == RQ_BAD_LEN) {
            repType = 3;
          }

          else if (myType == RQ_TYPE_HEAD) {
            // Send the header but no data and no parsing the request detail
            repType = 2;
          }
          else if (myType == RQ_TYPE_OTHER) {
            repType = 3;
          }

          // Make sure the JSON report data is up to date.
          updateStatusMessage();

          // Send a status report to the client. Type of report is defined by
          // value in variable 'RepType'
          switch (repType) {
            case 1:
            case 2:
              client.println("HTTP/1.1 200 OK");
              client.println("Content-Type: application/JSON");
              client.println("Access-Control-Allow-Origin: *");   // Support CORS loads
              client.println("Cache-Control: no-cache");
              client.println("Connection: close");
              client.println();
              if (repType == 1) {
                client.println(sysReport);
              }
              break;

            case 3:
              client.println("HTTP/1.1 404 Error");
              client.println("Content-Type: text/plain");
              client.println("Connection: close");
              client.println();
              client.println("Error");
              break;
          }

          // Prepare data markers for next header
          isFirstLine = true;
          nextCharPosn = 0;

          break;
        }
      }
    }
      // give the web browser time to receive the data
      delay(2);
      // close the connection:
      client.stop();
  }
  // ------------------
  if (stringComplete) parseInputString();   // Serial data from user - process string

  delay(5);
} // end of loop()


/*!
@brief Arduino serial event handler.
function serialEvent() is automatically called at the end of the main loop function 
if there is serial data in the input buffer.

*/
void serialEvent(void)
{
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if (!((inChar == '\r') || (inChar == '\n'))) {
      if (inputString.length() < MAXSTRINGINPUT - 2) {  // Catch overlong strings trashing memory
        inputString += inChar;  // Add character to inputString
      }
    }
    if (inChar == '\n') {
      stringComplete = true;  // Main loop will process the string
    }
  }
}
