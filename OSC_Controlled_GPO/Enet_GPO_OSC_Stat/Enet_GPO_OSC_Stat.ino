// ======================================
// Program Versions:
// 1.0    Initial Release
// 1.01   Enhanced preprocessor tests for host board and NIC type
// 1.02   Add support for setting status report port number.

/* ============================================================================
This sketch implements a GPO (General Purpose Output) interface with up to
eight outputs.

Each GPO output state (Off/On) is remote controlled using an OSC (Open Sound
Control) message. The control message format supports a salvo switching process 
such that any number of the outputs may be set to the required state using a 
single message.

The final GPO output hardware interface is often implemented using a relay 
module providing galvanic isolation between the GPO interface and the external 
controlled hardware. Some relay units require an active low control input to 
enable the relay, others need a high level (5V) control. This program includes 
an output invert control such that the control message uses the logical state 
and the program adapts the logical state to that required by the relay 
interface. The invert control process is per output, with the settings stored 
in EEPROM inside the Arduino processor.

This sketch was developed for use with two hardware systems:

Arduino Model:    Nano V3
Ethernet Shield:  ENC28J60 based
Ethernet Library: EthernetENC

Arduino Model:    UNO R3
Ethernet Shield:  WizNet based
Ethernet Library: Ethernet

The limited code memory in the Arduino, especially in a Nano using an ENC28J60
series NIC, requires the controlling OSC client to carefully craft the OSC 
messages sent to the GPO interface module. Each OSC message must be "raw", 
not sent as part of an OSC bundle. The OSC address for control message 
reception is:
 
/gpiswitch/out

The control message requires a single string parameter that encodes the 
switching data. The switch state string uses the format shown below:

Single element being switched:     "s3=1"
Multiple elements being switched:  "s1=1 s4=0 s5=0"

The number after the lower-case 's' is the switch id in the range 1 to 8. 
The value after the '=' is the switch state. Only states '0' and '1' are 
recognised. Badly formed messages are just ignored, no error reports are
provided.

When a pulsed output is required the controlling system must send two 
messages: one switching the output on, the second switching the output 
off after the desired pulse width time.

The Arduino NANO pins used for outputs are:
Relay Number   1   2   3   4   5   6   7   8
Pin Number     3   4   5   6   7   8   9  17 
Designation   D3  D4  D5  D6  D7  D8  D9  A3

    Note: Digital pin 2 is used by the ENC28J60 series Ethernet Shield.

The Arduino UNO pins used for outputs are:
Relay Number   1   2   3   4   5   6   7   8
Pin Number     2   3   5   6   7   8   9  17 
Designation   D2  D3  D5  D6  D7  D8  D9  A3

    Note: Digital pin 4 is used by the WizNet Ethernet Shield to enable/disable
          the micro SD interface on the shield. High output on D4 disables the
          micro SD.

Multiple GPO interfaces may be present on the same IP network, hence care must
be taken during installation to ensure the MAC adddress and the IP address 
used by each GPO unit are unique on that network.

Some interface instances may be moved between networks. To avoid the need 
to re-program the IP address data on each move, the GPO unit stores the MAC 
address and 4 sets of IP properties in the on-board EEPROM memory. Arduino 
inputs A0 and A1 are used as digital inputs functioning as IP set selectors. 
With no external connections made to pins A0 and A1 the first of the four 
IP address sets is used. 

Arduino pin A2 is also used as a digital input. If this pin returns a logic 1 
when read during boot-up the USB serial link is activated, otherwise the USB 
connector is only used as a power source. The physical input state is inverted 
by the reading software. Hence if pin A2 is open-circuit the serial link is 
disabled.

Arduino pin A4 is used to enable/disable recognition of a status report request 
message from a host. The status request is enabled when pin A4 is set at a low
(ground) level. When status replies are enabled the response is sent to the 
IP address that sent the status request. The target port number on the request 
system can be set to use the port number used to issue the status request, or 
set to a user-defined port id. The port number is stored in the EEPROM. Port 
0 is interpreted as a "use status source port" selection.

Using status request enables a controller process to know the state of all 
GPO outputs when the control process starts, and can be used as a background 
"ping" system that tests if the GPO interface is online.

The status request is a message sent to address /status but the parameters of 
the message are ignored

The status response message is a string in the form "ST01001100" where the 
digits are the state of outputs 1 (leftmost digit) to output 8 (rightmost 
digit). There is no carriage return or line feed after the string.

Arduino pin A5 enables/disables auto-status response mode. When enabled, a 
status message is sent after every switch set command. The auto-status 
response mode is enabled when pin A5 is connected to ground.

The elements of the active IP set are updated via the USB serial connection.
The serial link must be enabled before GPO boot by taking pin A2 low. Each
serial command is terminated by a line feed <lf> character. Input processing
ignores any carriage return characters on the link, so the terminal that
sends the commands can use <lf> only, <cr><lf>, or <lf><cr> as the line
terminate.

The implemented link commands are:

show
set ip <IP dotted string>
set mask <network mask dotted string>
set gw <network gateway ip dotted address>
set dns <dns ip dotted address>
set port <local udp listen port number>
set mac <6 hex value with colon seperators>
set invert <string of eight 0/1>
set statusport <port number used for status reply>
save

Andy Woodhouse
andy@amwtech.co.uk


Acknowledgements
================
My thanks go to the kind people who have contributed to the various libraries 
such as the Ethernet library and released them for others to use.

Licence: ISC

Copyright 2023 Andy Woodhouse

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
============================================================================ */

// Conditional defines select the relevant library for the NIC model in use.
// Some interfaces use the ENC28J60 as the ethernet controller, others use the
// WizNet WS5100 or WS5500 device. These two NIC units need different support
// libraries.
//
// Enable the #define for the NIC type in use. Comment out the other _NIC_XXX define.
#define _NIC_ENC28J60
// #define _NIC_WIZNET

// Enable the #define for the board type in use - NANO or UNO
#define _NANO_HOST
// #define _UNO_HOST

// Test that one host board and one NIC are defined.
#if !defined(_NIC_ENC28J60) && !defined(_NIC_WIZNET)
#error An ethernet NIC type must be defined.
#endif

#if defined(_NIC_ENC28J60) && defined(_NIC_WIZNET)
#error Only 1 ethernet NIC type must be defined.
#endif

#if !defined(_NANO_HOST) && !defined(_UNO_HOST)
#error A host board type must be defined.
#endif

#if defined(_NANO_HOST) && defined(_UNO_HOST)
#error Only 1 board type must be defined.
#endif

// Select the relevant NIC library support
#ifdef _NIC_ENC28J60
#include <EthernetENC.h>
#endif

#ifdef _NIC_WIZNET
#include <SPI.h>
#include <Ethernet.h>
#endif

#include <EthernetUdp.h>
#include <EEPROM.h>
#include "EthernetGPO.h"

// Define a buffer that is slightly larger than maximum UDP packet length expected
#define UDP_RX_PACKET_MAX_SIZE 100

// Define the maximum number of outputs from the GPO
#define MAXGPO 8

// Universally accessible constants and variables. The values in MAXGPO, l2p,
// and pinInvert need editing to match the users target hardware, and are
// set using the host board type definition.
//
// l2p array links the logical output number (eg 0 to 7) with the Arduino pin
// ID that controls the physical GPO output.
//
// Some relays have active low drives, others have active high drive. This 
// program assumes active high relays. Array pinInvert is loaded using
// values read from the EEPROM, configured using the serial command method.

#ifdef _NANO_HOST
const int l2p[MAXGPO] = { 3, 4, 5, 6, 7, 8, 9, 17 };
#endif
#ifdef _UNO_HOST
const int l2p[MAXGPO] = { 2, 3, 5, 6, 7, 8, 9, 17 };
#endif

bool pinInvert[MAXGPO] = { false, false, false, false, false, false, false, false };

// Define MAC address and IP address variables. Working values are read from EEPROM.
byte mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress my_ip = { 0, 0, 0, 0 };
IPAddress my_dns = { 0, 0, 0, 0 };
IPAddress my_gw = { 0, 0, 0, 0 };
IPAddress my_mask = { 0, 0, 0, 0 };
unsigned int localPort = 0;
unsigned int statusPort = 0;
int ipBlock = 0;                    // Holds which set of IP addresses are in use
bool statusReqEnable = false;       // Value read from pin A4 at boot
bool statusAutoEnable = false;      // Value read from pin A5 at boot

// Variables used to input and save operating properties to EEPROM
byte new_mac[] = { 0, 0, 0, 0, 0, 0 };
IPAddress new_ip(0, 0, 0, 0);
IPAddress new_dns(0, 0, 0, 0);
IPAddress new_gw(0, 0, 0, 0);
IPAddress new_mask(0, 0, 0, 0);
unsigned int newPort = 0;
unsigned int newStatusPort = 0;
bool newPinInvert[MAXGPO] = { false, false, false, false, false, false, false, false }; 

// Buffer for receiving data
char packetBuffer[UDP_RX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,

// Buffer used in decoding OSC switch commands
int switchSet[MAXGPO] = {-1, -1, -1, -1, -1, -1, -1, -1};

// Lookup table for first address of a set
const int ip_Data_Starts[IP_SETS] = {_IP_ADDRESS_01, _IP_ADDRESS_02, _IP_ADDRESS_03, _IP_ADDRESS_04}; 

EthernetUDP Udp; // EthernetUDP instance to let us receive and send packets over UDP

bool printEnabled = false;    // True if serial interface enabled. Pin A2 value at boot sets state.
#define MAXSTRINGINPUT 42
String inputString = "";      // A String to hold incoming serial data
bool stringComplete = false;  // True when input string is complete and ready to parse.


// ========================================================================= //
// Service functions start here                                              //
// ========================================================================= //

/*!
@brief void setPinModes(void) configures the input and output pins needed to
read control modes and output switch command states.
*/
void setPinModes(void)
{
  for (int i = _MIN_LOGICAL_OUT; i < _MIN_LOGICAL_OUT + MAXGPO; i++)
  {
    pinMode(logical_to_physical(i) , OUTPUT);
    digitalWrite(logical_to_physical(i), LOW);
  }

  // Set A0, A1, A2 and A4 lines as inputs with internal pullup
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
#ifdef _NIC_WIZNET
  pinMode(4, OUTPUT);
  digitalWrite(4, HIGH); // Disable micro SD interface
#endif
}


/*!
@brief Convert the logical_dest into an output pin number.

@param logical_dest Integer in range 0 to 7
@return Pin number used to control the logical output, or -1 for an invalid logical_dest input.
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
Note active pullup configuration is used, so unconnected pin reports high. Invert read state to get true value.

@return Integer in range 0 to 3
*/
int read_IP_block(void)
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
void fetchMACfromEeprom(byte* myMac)
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
int fetchIpEepromData(int whichSet)
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
  statusPort = ((unsigned int)EEPROM.read(addr1++) * 256) + (unsigned int)EEPROM.read(addr1++);
  return 0;
}


/*!
@brief Read output invert control settings from EEPROM memory. Values stored in global array pinInvert[].

*/
void readInvertSettings(void)
{
  int addr1 = _OUTPUT_INVERT; // Start of invert data

  for (int ix = 0; ix < MAXGPO; ix++) {
    pinInvert[ix] = EEPROM.read(addr1++) != 0 ? true : false;
  }
}


// Copy working IP properties to the edit array
void copyIPdataToEditable() {
  int ix;
  for (ix = 0; ix < MAXGPO; ix++) newPinInvert[ix] = pinInvert[ix];
  for (ix = 0; ix < 6; ix++) new_mac[ix] = mac[ix];
  for (ix = 0; ix < 4; ix++) new_ip[ix] = my_ip[ix];
  for (ix = 0; ix < 4; ix++) new_dns[ix] = my_dns[ix];
  for (ix = 0; ix < 4; ix++) new_gw[ix] = my_gw[ix];
  for (ix = 0; ix < 4; ix++) new_mask[ix] = my_mask[ix];
  newPort = localPort;
  newStatusPort = statusPort;
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
  addr1++;

  byte stPortHi, stPortLo;
  stPortHi = (byte)(newStatusPort / 256);
  stPortLo = (byte)(newStatusPort % 256);
  EEPROM.update(addr1, stPortHi);
  addr1++;
  EEPROM.update(addr1, stPortLo);

  return 0;
}


/*!
@brief identApplication is called to print the Name and key operating properties of the IP interface

*/
void identApplication(void)
{
  if (!printEnabled) return;
  Serial.println("Ethernet OSC GPO Interface");
  Serial.print("Using IP Address set #");
  Serial.println(ipBlock);
  printInvertSet();  // State of inverts array
  printIPstack(mac, my_ip, my_mask, my_gw, my_dns, localPort, statusPort);
  Serial.print("Status request ");
  Serial.println(statusReqEnable ? "enabled" : "disabled");
  Serial.print("Auto status reply ");
  Serial.println(statusAutoEnable ? "enabled" : "disabled");
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
void printIPstack(byte* my_mac, IPAddress addr_ip, IPAddress addr_mask, IPAddress addr_gw, IPAddress addr_dns, unsigned int myPort, unsigned int statPort)
{
  byte hexVal[3];
  byte hexChar[] ="0123456789ABCDEF";
  int addrLen = 0;

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
  // Show the status report port
  Serial.print("Status reply port ");
  Serial.println(statPort);
}


/* ----------------------------------------------------------------------------
  The OSC message structure used in this control application is very simple. 
  This allows the message parsing code to be significantly simplified, reducing 
  code footprints in both flash and ram memory. OSC bundles are NOT supported.

  The recognised OSC control message address is "/gpiswitch/out" and the 
  message body is a string that contains the switch ids and states required. 
  A single message may switch a single or multiple outputs.
  
  The limited number of output pins on the Arduino NANO/UNO limits the switch
  numbers to the range 1 to 8. Eight values are supported, even if a GPO unit
  is only actively using 4 physical outputs.

  The number of switches could be expanded by using an Arduino MEGA host 
  platform, but it will still not be more than about 48 outputs.

  The OSC address terminates with a NULL character, so it is easy to measure
  the length of the string. A required address is 15 characters long with the
  first 14 characters matching the address header "/gpiswitch/out".

  The message body must include a parameter type "string". This string is
  a variable length and includes the switching data with one or more tokens of 
  the form "s3=1" where digit after the 's' identifies the switch number and
  the digit after the '=' identifies the logical output state. Multiple
  outputs are switched using a string of the form "s1=1 s3=0 s4=1"

  No error reporting is available for the OSC format. If the Arduino is not 
  switching outputs enable the serial port for data transfers at 9600. Use a 
  terminal program such as the one in the Arduino Development software to 
  check the IP address properties in use. Wireshark can be used to check the 
  OSC control messages are directed at the correct address and port.

  When Arduino pin A5 is low during boot every valid OSC command causes a
  status message to be sent to the IP address and port that sent the 
  command.
*/

const char msgHead[] = "/gpiswitch/out";
const int  msgHeadLen = 14;

/*!
@brief Process a recived OSC message to check if the request is valid address and valid parameter type.
It parses the tokens within the delivered string, and actions any required switch outputs. It returns
a status message when status reports are enabled.

@param br The number of bytes in the message buffer
@param pktbuf A pointer to the buffer containing the received message
*/
void parseOscMessage(int br, char* pktBuf)  /* br is abbreviation for bytesRead */
{
  int ix1=0, ix2=0, ix3=0, ix4=0, refLen=0;
  int switchID = 0;

  if (br == 0) return;

  if (pktBuf[0] != '/') return; // First character must be a '/'

  // Extract the address string length by parsing forward from start of buffer
  while ((pktBuf[ix1] != '\0') && (ix1 < br)) ix1++; 
  
  if (ix1 != msgHeadLen) return;

  // Check if the provided address and the reference address match.
  int match = 0;
  for (int i = 0; i < msgHeadLen; i++) {
    if (pktBuf[i] != msgHead[i]) match++;
  }
  if (match != 0) return;

  // Scan past the address looking for first non-null character at start of data type list
  ix2 = ix1;
  while ((pktBuf[ix2] != ',') && (ix2 < br)) ix2++;

  ix3 = ix2;
  while ((pktBuf[ix3] != '\0') && (ix3 < br)) ix3++;  // Locate end of format string
  
  if ((ix3 - ix2) != 2) return; // Expect only 2 characters in format string

  if (pktBuf[ix2+1] != 's') return;   // Data type must be string

  ix4 = ix3 + 2;  // Index past the ",s"

  while ((pktBuf[ix4] == '\0') && (ix4 < br)) ix4++;  // Find start of switch tokens

  if (ix4 + 1 == br) return;  // No switch tokens found

  int ts = ix4;
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
        switchSet[swID-1] = swVal;
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
      if (switchSet[j] >= 0) {
        // Update the pin
        swState = (switchSet[j] == 1) ;
        swState = (pinInvert[j] == 1) ? !swState : swState;
        digitalWrite(logical_to_physical(j) , swState ? 1 : 0);
      }
    }

    // Send the status reply if the auto mode is enabled.
    if (statusAutoEnable) sendStatusReport();
  }
  // Clear the state changes array
  for (int j = 0; j < MAXGPO; j++) switchSet[j] = -1;
}


// function bool parseToken(int startIndex, int *endIndex, int *sw_id, int *sw_val)
// is called to extract and decode a single switch value token from the string held
// in array packetBuffer[].
// It returns a boolean value where true means a valid token was found and processed
// and false indicates one or more errors in the format of the token. The calling
// program function should not call the parser again until a new request is sent
// from the client. There are four parameters - one for input the others for 
// property value returns (i.e. they are pointers).
//
// startIndex   the start position of a token in the buffer
// *endindex    is set to the address of the next token or -1 if there are
//              no more tokens
// *sw_id       is iset to the switch number extracted frome the token, or -1 if 
//              the switch number format is invalid
// *sw_val      is set to the OFF or ON state defined by the token, or -1 for
//              a badly formatted token.
//
// Several tests of token validity are made. Failing any test causes the
// function to exit returning a false.
// 
// Test 1 - does the token content start with an 's' or 'S'?
// Test 2 - is there an '=' separator?
// Test 3 - are there one or more characters between the 's' and the '='? 
// Test 4 - are the characters between the 's' and '=' decimal characters?
// Test 5 - character after the '=' is either '0' or '1'
// Test 6 - is the character after the switch value a ' ' or a '\0'.

/*!
@brief Looks for a switch token in the message buffer. The message buffer is global variable 'packetBuffer[]'

@param startIndex Position in buffer to start
@param endindex Pointer to an integer that is the index of the end of the token
@param sw_id Pointer to an integer that returns the extracted switch number
@param sw_val Pointer to an integer holds the extracted switch state (0 or 1)
@return True if a valid token was found and switch state and value extracted. False if no token or invalid token.
*/
bool parseToken(int startIndex, int *endIndex, int *sw_id, int *sw_val) {
  int idx1 = startIndex;      // index for start of token
  int idx2 = startIndex + 1;  // index for the "="
  int idx3 = 0;               // index for start of next token, or terminating null

  // Configure the returned values as "failed to parse a token".
  *endIndex = -1;
  *sw_id = -1;
  *sw_val = -1;
  
  if (!(packetBuffer[idx1] == 's') || (packetBuffer[idx1] == 'S')) return false;  // Test 1
 
  while ((packetBuffer[idx2] != '=') && (packetBuffer[idx2] != '\0')) idx2++; // Test 2
  if (packetBuffer[idx2] == '\0') return false;

  if ((idx2 - idx1) < 2) return false; // Test 3

  for (int i = idx1+1; i < idx2; i++) { // Test 4
    if ((packetBuffer[i] < '0') || (packetBuffer[i] > '9')) return false;
  }

  if (!((packetBuffer[idx2+1] == '0') || (packetBuffer[idx2+1] == '1'))) return false; // Test 5

  if (!((packetBuffer[idx2+2] == ' ') || (packetBuffer[idx2+2] == '\0'))) return false; // Test 6
  
  // All tests passed so compute values and return values
  int sw = 0;

  idx3 = idx2+2;
  if (packetBuffer[idx2+2] != '\0') {
    while (packetBuffer[idx3] == ' ') idx3++;
  }

  *endIndex = (packetBuffer[idx3] == '\0') ? -1 : idx3;
  for (int i = idx1+1; i < idx2; i++) {
    sw = sw * 10;
    sw = sw + (packetBuffer[i] -'0');
  }
  *sw_id = sw;
  *sw_val = packetBuffer[idx2+1] - '0';  
  return true;
}


// Return the status of the GPO outputs to the IP address that sent the switch 
// command or the status report request. Use destination port from user set 
// value, or the port number that issued the status/switch command.

const char statusHead[] = "/status";
const int  statusHeadLen = 7;
char statusReply[] = "ST00000000";

/*!
@brief Emit a status report containing output switch states. UDP message sent 
to the IP address that requested the status. If the status reply port number 
is 0 the response is sent to the port number that issed the status request, 
otherwise it is directed at the configured port number.

*/
void sendStatusReport(void) {
  bool pinVal = false;
  unsigned int srp = 0;

  for (int i=0; i < MAXGPO ; i++) {
    pinVal = digitalRead(logical_to_physical(i)) == 1;
    if (pinInvert[i]) pinVal = !pinVal;
    statusReply[i + 2] = pinVal ? '1' : '0';
  }

  srp = statusPort == 0 ? Udp.remotePort() : statusPort;
  Udp.beginPacket(Udp.remoteIP(), srp);
  Udp.write(statusReply);
  Udp.endPacket();
}


/*!
@brief Check if the message buffer contains a valid status request, and issues the
status if the message is valid.

@param br Number of bytes in the message buffer
@param pktBuf Pointer to message buffer
*/
void parseStatusRequest(int br, char* pktBuf) {
  int ix1=0;

  if (br == 0) return;

  if (pktBuf[0] != '/') return; // First character must be a '/'

  // Extract the address string length by parsing forward from start of buffer
  while ((pktBuf[ix1] != '\0') && (ix1 < br)) ix1++; 
  
  if (ix1 != statusHeadLen) return; 

  // Check if the provided address and the reference address match.
  int match = 0;
  for (int i = 0; i < statusHeadLen; i++) {
    if (pktBuf[i] != statusHead[i]) match++;
  }
  if (match != 0) return;

  // Have the status request header. Send a reply to the IP address and port 
  // that sent us the packet we received.
  sendStatusReport();
}


/*!
@brief Report a problem with an input value, then reset serial input ready for new command

*/
void reportSerialError(void) {
  Serial.println("Error");
  resetInputString();
}


/*!
@brief Reset the serial input buffer to empty.

*/
void resetInputString(void) {
  inputString = "";
  stringComplete = false;
}


/*!
@brief Test if the provided string parameter is a valid IPv4 dotted address.

@param myInput Pointer to the character array that holds the string to check.
@return True if the address is a dotted IP value.
*/
bool checkIsDottedIP(String myInput) {
  IPAddress apip;   // Used in checking input is valid IP string

  return apip.fromString(myInput);
}


/*!
@brief Convert two input ASCII hex characters to unsigned byte value.

@param up The ASCII character representing the most significant 4 bits of results
@param lo The ASCII character representing the least significant 4 bits of results
@return Unsigned 8-bit byte value
*/
byte ascii2hexval(byte up, byte lo) {
  byte a, b;
  a = (up >= 'A') ? (up - 'A' + 10) << 4 : (up - '0') << 4 ;
  b = (lo >= 'A') ? (lo - 'A'+ 10): (lo - '0');
  return a+b; 
}


/*!
@brief Check if the string parameter has valid syntax to be a MAC address. 
17 characters long of the form "a3:d4:c2:98:24:01".

@param newmac The string to test as a potential MAC address
@return True if the string is a valid MAC syntax, false otherwise.
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
@brief Checks the input parameter syntax against requirements for output invert control string.
Must be 8 characters long containg only '0' and '1'.

@return True id string has valid invert control syntax.
*/
bool parseInvert(String invert) {
  if (invert.length() != MAXGPO) return false;

  const char* buf = invert.c_str();
  bool just01 = true;
  for (int i = 0; i < MAXGPO ; i++) {
    just01 = just01 && ((buf[i] == '0') || (buf[i] == '1'));
  }
  if (!just01) return false;
  // Swap the data around so that the last digit stores in invert array position 0
  for (int i = 0; i < MAXGPO; i++) {
    newPinInvert[i] = (buf[MAXGPO - 1 - i] == '1') ? true : false;
  }
  
  return true;
}


/*!
@brief Send the invert control data held in variable newPinInvert to the serial terminal.

*/
void printInvertSet(void)
{
  // Print the newPinInvert array.
  Serial.print("Invert control ");
  for (int i = 0; i < MAXGPO; i++) {
    Serial.print( newPinInvert[MAXGPO - 1 - i] ? '1' : '0');
  }
  Serial.println();
}


/*!
@brief Parse the content of the serial input buffer. Checks for supported command
verbs such as 'show' 'set' 'save'

*/
void parseInputString(void)
{
  inputString.trim(); // Remove leading and trailing whitespace
  // Have we received a "save" request?
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
    printIPstack(new_mac, new_ip, new_mask, new_gw, new_dns, newPort, newStatusPort);
    resetInputString();
    return;
  }

  if (inputString.startsWith("set ")) {
    // Remove the "set " string from the start of the input string
    inputString.remove(0, 4);

    // We hope to have two parts left - the "verb" and the "property_value" 
    // separated by a space. If not, we have an error.
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
    else if (myVerb == "statusport") {
      long pv = myProperty.toInt();
      if ((pv < 0) || (pv > 65535)) {
        reportSerialError();
        return;
      }
      else {
        newStatusPort = (unsigned int)pv ;
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
    printIPstack(new_mac, new_ip, new_mask, new_gw, new_dns, newPort, newStatusPort);
  }
  else {
    resetInputString();
  }
}


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// Arduino Initialisation Code
/*!
@brief This function is run once after the microcontroller is reset. It initialises the input and
output pin modes, and reads the active control properties.

*/
void setup(void) {

  setPinModes(); // Setup the input and output pin modes used by this application
  ipBlock = read_IP_block();  // Read the IP address set from A0 and A1 pins
  printEnabled = (digitalRead(A2) == 1) ? false : true; // Read the serial enable pin, low to enable.
  statusReqEnable = (digitalRead(A4)== 1) ? false : true; // Accept status requests?
  statusAutoEnable = (digitalRead(A5)== 1) ? false : true; // Send status on every set command?

  // Initialise the serial link if required for operational use. (When Pin A2 is low)
  if (printEnabled) {
    Serial.begin(9600);
    while (!Serial) {
      ; // wait for serial port to connect. Used by USB port only.
    }
  }

  fetchMACfromEeprom(mac);
  fetchIpEepromData(ipBlock);
  readInvertSettings();
  copyIPdataToEditable();   // Copy Current IP data to editable data array.
  inputString.reserve(MAXSTRINGINPUT); // Define maximum buffer for character input

  // Ident application and properties on console if serial port is enabled.
  identApplication(); 

  Ethernet.begin(mac, my_ip, my_dns, my_gw, my_mask); // Start the Ethernet object
  
  // Check for presence of Ethernet hardware
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    if (printEnabled) {
      Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    }
    while (true) {
      delay(10); // do nothing, no point running without Ethernet hardware
    }
  }
  
  // Set the outputs to 'off' allowing for the pinInvert[] array setting
  for (int i = 0; i < MAXGPO; i++) {
    digitalWrite(logical_to_physical(i), pinInvert[i]);
  } 

  Udp.begin(localPort); // Listen for UDP packets
}


// ++++++++++++++++++++++++++++++++++++
// Arduino Application Loop

/*!
@brief This function is repeatedly called once the setup process is complete.

*/
void loop(void) {
  // If data is available, read a packet.
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    // read the packet into packetBuffer
    unsigned int bytesRead = Udp.read(packetBuffer, UDP_RX_PACKET_MAX_SIZE);
    
    if (bytesRead >= 2) {
      if ((packetBuffer[0] == '/') && (packetBuffer[1] == 'g')) { 
        // OSC command address must start with "/g"
        parseOscMessage(bytesRead, packetBuffer);
      }
      else {
        if ((packetBuffer[0] == '/') && (packetBuffer[1] == 's') && statusReqEnable) { 
        // Status request address must start with "/s", and status reporting must be enabled.
        parseStatusRequest(bytesRead, packetBuffer);
        }
      }
    }
  }

  if (stringComplete) parseInputString();   // Serial data from user - process string

  delay(2);
  Ethernet.maintain();
}


/*!
@brief This function is called at the end of the loop() pass if there is 
serial data in the USB serial buffer.

*/
void serialEvent()
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
