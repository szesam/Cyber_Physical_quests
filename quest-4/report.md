# Electronic Voting
Authors: Carmen Hurtado, Samuel Sze

Date: 2021-04-08

-----

## Summary
**Contributors:**
Samuel | Carmen 
-------|--------
<img src="images/samuel.jpg" width="" height="200" /> | <img src="images/carmen.jpg" width="" height = "200" />) 

In this quest we are designing an electronic voting system. To do so, we created 5 invdividual unique devices (FOBS) acting as ID cards. These ID cards are able to communicate to one another through secured near field IR communications. They are also connected wirelessly to the local area network. Whenever a FOB votes, the vote is passed onto an elected poll leader FOB and subsequently transmitted to a Raspberry PI for database storage and front end display. 

Front end display includes a web client html page. The web client is also capable of sending commands back to the server, such as clearing out all votes and restarting the voting process.

### Investigative question: 


## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One: FOB performs IR NFC data exhcnage to another FOB with LED indication | 1 |  1     | 
| Objective Two: Receiving FOB communicates vote to Poll leader (LED indication) via network communication | 1 |  1     | 
| Objective Three: Poll leader (LED indication) is replaced if fails  | 1|  1     | 
| Objective Four: Poll leader reports votes to server database | 1 |  1     | 
| Objective Five: Portal allows query to databse to show actual vote counts per candidate | 1 |  1     | 
| Objective Six: Operates over multiple sites or with all available fobs | 1 |  1     | 
| Objective Seven: Investigative question response | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 5 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 3 |  3     | 


## Solution Design

### FOB circuitry:

FOB circuitry includes an ESP32, a L293 H-bridge, two push-buttons, three LEDs, an IR transmitter and an IR receiver. Detail circuitry is shown in the figure below.  

<img src="images/storyboard.png" width="500" height="" />

To change a voting color, the push-button corresponding can be depressed. To send a message, the other push-button can be depressed. The current FOB voting message will only be received by another receiving FOB if the convex side of the IR receiver is facing towards the IR transmitter. Once a message is received, a corresponding LED on the received FOB will light up indicating the voting color of the received FOB message. 

### Leader Election:

Leader election plays a big role in determining how the FOB transmits voting messages to the PI server. There is at any moment only one leader within the group of connected FOBs, while all the other FOBs should keep quiet during that moment. Leader election occurs when the channel is quiet for over a leader heartbeat (2s) amount of time, in which several outcomes will occur. The Finite State Machine of this leader election is shown below to better illustrate the corresponding events and states. 

<img src="images/fsm1.jpg" width="500" height="" />


### Local Area Network Architecture (Backend):

UDP multicast is the main networking administration structure used to connect multiple FOBs and the Raspberry PI server together. Essentially, a multicast address is opened within a local area network setup by a loan router. Within the multicast address, leader elections and UDP voting messages are broadcasted by each FOB when needed, and picked up by the relevant FOBs or PI server by using string matching. 

### Database and Front-end webpage:




## Sketches and Photos
<img src="images/storyboard.png" width="500" height="" />

<img src="images/graph.png" width="" height="" />

## Supporting Artifacts
- [Link to video demo](https://youtu.be/9RcBN8nKxJE)
- [Link to .c code file](https://github.com/BU-EC444/TeamRocket-Sze-Hurtado/blob/master/quest-3/code/udp_client.c)
- [Link to .js code file](https://github.com/BU-EC444/TeamRocket-Sze-Hurtado/blob/master/quest-3/code/demo_udp.js)
- [Link to .html code file](https://github.com/BU-EC444/TeamRocket-Sze-Hurtado/blob/master/quest-3/code/index.html)


## Modules, Tools, Source Used Including Attribution
- ESP32
- Raspberry Pi Zero W 
- RMT and UART for NFC IR
- UDP Protocol (Multicast)
- DDNS
- MongoDB
- node.js (socket.io, express, datagram)
- CanvasJS 

## References
- [UDP client expressif](https://github.com/espressif/esp-idf/tree/master/examples/protocols/sockets/udp_client)
- [ADXL343 specfications](http://whizzer.bu.edu/skills/accel)
- [Thermistor sensor spec sheet](https://www.eaa.net.au/PDF/Hitech/MF52type.pdf)
- [DDNS](http://whizzer.bu.edu/skills/dyndns-pi)
- [WIFI on ESP](http://whizzer.bu.edu/skills/wifi)
-----

