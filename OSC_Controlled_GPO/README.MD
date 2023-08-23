# Network General Purpose Output Interface
This project implements a simple GPO (General Purpose Output) interface with 8 outputs.

## Output State Control
Each GPO output state (Off/On) is remote controlled over an IP connection using an OSC 
(Open Sound Control) message. The message supports single output control and salvo switching 
control of multiple outputs set to their required states using a single message.

## Physical Output Interface
The final GPO output hardware interface is often implemented using a relay 
module providing galvanic isolation between the GPO interface and the 
external controlled hardware. Some relay modules require an active low control 
input to enable the relay, others need an active high level (5V) control. This program 
includes an output invert control process that enables a controller to send the
logical output state which the program adapts the logical state to that required 
by the specific relay interface. The invert control process is per output, and the 
settings are stored in EEPROM inside the Arduino processor.

## Microcontroller Hardware
This sketch was developed for use with:

```
Arduino Model:    Nano V3

Ethernet Shield:  ENC28J60 based

Ethernet Library: EthernetENC
```

```
Arduino Model:    UNO R3

Ethernet Shield:  WizNet based

Ethernet Library: Ethernet
```

## Control Message Format
The limited code memory in the Arduino, especially in a Nano using an ENC28J60
series NIC, requires the controlling OSC client to carefully "craft" the OSC 
messages sent to the GPO interface module. Each OSC message must be "raw", 
not sent as part of an OSC bundle. The OSC address for control message 
reception is:
 
__/gpiswitch/out__

The control message requires a single string parameter that encodes the 
switching data. The switch state string uses the format shown below:

Single element being switched:     __"s3=1"__  

Multiple elements being switched:  __"s1=1 s4=0 s5=0"__  

The number after the lower-case 's' is the switch id in the range 1 to 8. 
The value after the '=' is the switch state. Only states '0' and '1' 
are recognised.

Badly formed messages are just ignored, no error reports are
provided.

When a pulsed output is required the controlling system must send two messages, 
one switching the output on, the second switching the output off after the 
desired pulse width time.

## Output Pin Allocation
The ENC28J60 ethernet uses pin 2 (D2) as an interface select control. WizNet
interfaces use pin 4 (D4) to enabe/disable a microSD sub-system on the shield. This implies there are different output pin connections that depend predominantly on the ethernet module used.

The Arduino NANO + ENC20J60 Ethernet Interface pins used for outputs are:  

| Output Number | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |  
|---------------|---|---|---|---|---|---|---|---|  
| Pin Number | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 17 |
| Designation | D3 | D4 | D5 | D6 | D7 | D8 | D9 | A3| 
  
The Arduino UNO + WizNet Ethernet Interface pins used for outputs are:  

| Output Number | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |  
|---------------|---|---|---|---|---|---|---|---|  
| Pin Number | 2 | 3 | 5 | 6 | 7 | 8 | 9 | 17 |
| Designation | D2 | D3 | D5 | D6 | D7 | D8 | D9 | A3| 

## Input Pin allocations
### IP Address Sets
Multiple interfaces can be present on the same IP network hence care must 
be taken during installation to ensure the MAC address and the IP address 
used by each GPO unit are unique.

Some interface instances may be moved between networks. To avoid the need 
to re-program the IP data on each move, the GPO unit stores the MAC address 
and 4 sets of IP properties in on-board EEPROM memory. Arduino inputs A0 
and A1 are used as digital inputs functioning as IP set selectors. With no pin 
connections made to A0 and A1 the first of the four IP address sets is used. 

### Serial Link Enable
Arduino pin A2 is used as a digital input. If this pin returns a logic 1 when
read during boot up the USB serial link is activated, otherwise the USB 
connector is only used as a power source. If pin A2 is open-circuit the serial 
mode is disabled. Serial mode must be active to edit the IP and MAC data. Serial 
mode should be disabled unless edits are required.

### Enable Status Request Message
Arduino pin A4 is used to enable/disable recognition of a status report request 
message sent by a controller. The status request is enabled when pin A4 is set at a low
(0V) level. When status replies are enabled the response is sent to the 
IP address that issued the status request. The port number for the reply can either be 
the port number used by the status request message, or a port number defined by the user during interface installation. Using status request enables a controller process 
to know the state of all GPI outputs when the controller process starts, and can 
also be used as a background ping system that tests if the GPI interface is online.

### Auto Status Enable Mode
Arduino pin A5 enables/disables auto-status response mode. When enabled, a 
status message is sent after every switch set command. The auto-status 
response mode is enabled when pin A5 is connected to ground.

### Status Response Message Format
The status response message is a string in the form "ST01001100" where the 
digits are the state of outputs 1 (leftmost digit) to output 8 (rightmost 
digit). 

## Serial Link Messages
The elements of the active IP set can be updated via the USB serial connection.
The serial link must be enabled before boot by taking pin A2 low. Each
serial command is terminated by a line feed <lf> character. Input processing
ignores any carriage return characters on the link, so the terminal that
sends the commands can use \<lf> only, \<cr>\<lf>, or \<lf>\<cr> as the line
terminate.

The implemented link commands are:

show  
set ip \<IP dotted string>  
set mask \<network mask dotted string>  
set gw \<network gateway ip dotted address>  
set dns \<dns ip dotted address>  
set port \<local udp listen port number>  
set mac <6 hex value with colon seperators>  
set invert \<string of eight characters (0 = not invert, or 1 = invert)>  
set statusport \<port number to receive status responses>  
save  

After settings are updated the new values must be saved, then the Arduino must 
be re-booted to activate the settings.


## Acknowledgements
Thanks to those kind people who have contributed to the various support libraries such as the Ethernet library and released them for others to use.
