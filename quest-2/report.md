# Quest Name
Authors: Carmen Hurtado, Sameul Sze

Date: 2021-03-05
-----

## Summary
In this quest we are asked to read analog signals from 3 different sensors and to display the results in an organized and readable way. For this we wired up a Thermistor, Ultrsonic sensor, and IR sensor to the ESPR32 board. The sensors are each connected to their own analog input pin from where we read in voltage and convert to engineering units (Celcius for the Thermistor and Meters for the two distance sensors). Using node.js and CanvasJS, we created a web server to display in real time, updating every 1 second, the readings from the ESP32 serial port. These are also returned back to the user in the console in real time every 1 second. 

Investigative question: 

## Self-Assessment

### Objective Criteria

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1 |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1|  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Qualitative Criteria

| Qualitative Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Quality of solution |  |  5     | 
| Quality of report.md including use of graphics |  |  3     | 
| Quality of code reporting |  |  3     | 
| Quality of video presentation |  |  3     | 


## Solution Design
We have 3 different sensors:
- Thermistor: Output voltage needs to go through a voltage divider.
- Ultrasonic Range Finder: Output voltage goes directly into analog pin; needs a 100mF capacitor to resolve noise. Specs: Range (30cm - 100cm)
- IR: Output voltage goes directly into analog pin; needs a 100mF capacitor to resolve noise. Specs: Range(20cm - 150cm)

We have 3 different analog pins to connect each sensor and read in an output voltage in mv. This voltage is then converted to engineering units. The equations are different for each sensor. Thermistor retuns temperatur in Celsius, Both distance sensors return values in meters.

This data is read through the ESP32 pinsa, and it is intersected by the node.js program. This program reads in the data using the serialport module and builds a web server using socket.io to display data in current time at intervals of 1 second. In the web server, the data is displayed in two charts. One chart shows the distance vs time values from both the IR and Ultrasonic; the second chart displays the temperature vs time. To compare the results with the output voltage read, we show in the top left corner the changing output values every 1 second. 

## Sketches and Photos
<center><img src="./images/ece444.png" width="25%" /></center>  
<center> </center>


## Supporting Artifacts
- [Link to video demo](). Not to exceed 120s


## Modules, Tools, Source Used Including Attribution

## References

-----

