# Network General Purpose Output Interface
This project implements a simple GPO (General Purpose Output) interface with 8 outputs.

## Output State Control
Each GPO output state (Off/On) is remote controlled over an IP connection using http GET 
messages. The message structure supports both single output control and salvo output 
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
Arduino Model:    UNO R3

Ethernet Shield:  WizNet based

Ethernet Library: Ethernet
```

## Control Message Format
The GPO interface implements a simple http server. The limited code memory in the Arduino 
requires the controlling http GET client carefully "craft" the messages sent to the GPO 
interface module. One control page address is used to set the switch states. Each switch 
message returns a status message which contains the state of all output switches. A NULL 
switching message (a GET with no switching data) also emits a status message providing a 
interface present and active check. 


The switching page address for control message reception is:
 
__/gpiswitch/out__

Switching output states is achieved by including a query to the basic GET to the target address. 
Some examples are:

Single element being switched:     **/gpiswitch/out?s1=1**  

Multiple elements being switched:  **/gpiswitch/out?s1=1&s4=0&s5=0** 

The number after the lower-case 's' is the switch id in the range 1 to 8. 
The value after the '=' is the switch state. Only states '0' and '1' 
are recognised.

Badly formed messages are just ignored, no error reports are
provided.

When a pulsed output is required the controlling system must send two messages, 
one switching the output on, the second switching the output off after the 
desired pulse duration.

## Output Pin Allocation
WizNet ethernet shields use pin 4 (D4) to enable/disable a microSD sub-system on 
the shield. This makes pin 4 unavailble even if the microSD card is unused.
  
The Arduino UNO + WizNet Ethernet Interface pins used for outputs are:  

| Output Number | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |  
|---------------|---|---|---|---|---|---|---|---|  
| Pin Number | 2 | 3 | 5 | 6 | 7 | 8 | 9 | 17 |
| Designation | D2 | D3 | D5 | D6 | D7 | D8 | D9 | A3| 

## Input Pin allocations
### IP Address Sets
Multiple interfaces can be present on the same IP network hence care must 
be taken during installation to ensure the MAC address and IP address 
used by each GPO unit are unique.

Some interface instances may be moved between networks. To avoid the need 
to re-program the IP data on each move, the GPO unit stores the MAC address 
and 4 sets of IP properties in on-board EEPROM memory. Arduino inputs A0 
and A1 are used as digital inputs functioning as IP set selectors. With no pin 
connections made to A0 and A1 the first of the four IP address sets is used. 

### Serial Link Enable
Arduino pin A2 is used as a digital input. If this pin is low when read during 
boot-up the USB serial link is activated, otherwise the USB connector is only 
used as a possible power source. If pin A2 is open-circuit the serial mode is 
disabled. Serial mode must be active to edit the IP and MAC data, but should 
be active only during the edit process.

### Status Response Message Format
The status response message is a stringified JSON object containing the state 
of all eight output switches. The string has the format:

{"s1":1,"s2":1,"s3":0,"s4":0,"s5":0,"s6":1,"s7":0,"s8":1}

## Serial Link Messages
The elements of the active IP set can be updated via the USB serial connection.
The serial link must be enabled before boot by taking pin A2 low. Each
serial command is terminated by a line feed <lf> character. Input processing
ignores any carriage return characters on the link, so the terminal that
sends the commands can use \<lf> only, \<cr>\<lf>, or \<lf>\<cr> as the line
terminate.

The implemented link commands are:

**show  
set ip \<IP dotted string>  
set mask \<network mask dotted string>  
set gw \<network gateway ip dotted address>  
set dns \<dns ip dotted address>  
set port \<listen port number>  
set mac <6 hex value with colon seperators>  
set invert \<string of eight characters (0 = not invert, or 1 = invert)>  
save**  

After any settings are updated, the new values must be saved, then the Arduino 
must be re-booted to activate the settings.


## Acknowledgements
Thanks to the kind people who have contributed to the various support libraries such as the Ethernet library and released them for others to use.
