# Quest Name: Fish Feeder
Authors: Samuel Sze, Carmen Hurtado Garcia

Date: 2021/02/19


# Summary
**Contributors:**
Samuel | Carmen 
-------|--------
<img src="images/samuel.jpg" width="" height="200" /> | <img src="images/carmen.jpg" width="" height = "200" />) 

**Self-Assessment**

Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Servo spins right then left three times without chatter at prescribed time intervals | 1 |  1     | 
| Alphanumeric display indicates hourss and minutes | 1 |  1     | 
| Display shows countdown time report every second with no time loss | 1 |  1     | 
| Food dispensed at specified time and report submitted in team folder with all required components | 1  |  1     | 
| Investigative question response | 1 |  1     | 


Qualitative Criteria


| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution | 5 |  5     | 
| Quality of report.md including use of graphics | 3 |  3     | 
| Quality of code reporting | 3 |  3     | 
| Quality of video presentation | 3 |  3     | 


**Solution Design**
For achieveing the purpose of the fish feeder we inplemented the timer functionality of the ESP32 along with a 14-segment Adafruit display for actively showing the remaining time until next feed, and a servo motor with a container attached that rotates 3 times to dispense the food. In order to start the timer, we ask the consumer to input a desired time in hours and minutes to schedule the feeding. For this functionality we made use of the UART IO in the esp32. 

Investigative question: For setting the time interval dynamic we could use a button with a button interrump to incercept the signal and every time the user presses the button the program would stop and it would be possible to enter a new time. From here the timer would restart at this new time and continue top count down to zero. 


# Sketches and Photos
![Alt text](images/fishfeeder.png?raw=true "Title")
![Alt text](images/storyboard.png?raw=true "Storyboard")


# Supporting Artifacts
1. [Link to video demo](linkhere).
2. [Link to Code](https://github.com/BU-EC444/TeamRocket-Sze-Hurtado/blob/master/quest-1/code/fishtank.c)


# References
1. [EC444 Git](https://github.com/BU-EC444/code-examples).
2. [Adafruit ESP-IDF Pinouts](https://learn.adafruit.com/adafruit-huzzah32-esp32-feather/pinouts)
3. [Adafruit 14-pin Alphanumeric Pinouts](https://learn.adafruit.com/14-segment-alpha-numeric-led-featherwing/usage#library-reference-4-14)
-----

