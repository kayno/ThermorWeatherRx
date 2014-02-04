ThermorWeatherRx
================


"Thermor" DG950R Weather Station receiver v 0.3

Receives data from "Thermor" DG950R Weather Station receiver, via a 433Mhz RF receiver connected to pin 8 of the arduino, and outputs to serial.

Based on the Practical Arduino Weather Station Receiver project (http://www.practicalarduino.com/projects/weather-station-receiver). 
For more info: 
http://kayno.net/2010/01/15/arduino-weather-station-receiver-shield/

TODO:
- handle rain packets better - bucket tips are are sent incrementally - rain mm is calculated by subtracting current tip count from previous. should report tip count rather than doing calulation on arduino so that if the arduino is reset or a packet sent from it is missed, any missed rain value can still be calculated
- wind speed is currently determined by a 'fudge factor', which itself was calculated based on the value being transmitted compared to the value displayed on the weather station's base station. perhaps there is a better way?

