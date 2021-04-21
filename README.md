# MS5607
Fork of the MS5607 project by linear-zz 
https://github.com/llinear-zz/MS5607
Adding 2nd order temperature correction based on parallax's documentation and an example logger file.
The following is linear-zz's original README text
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
Python code for the Altimeter Module MS5607
Based on the documents found at https://www.parallax.com/product/29124

```
from MS5607 import MS5607

sensor = MS5607()
temperature = sensor.getDigitalTemperature()
pressure = sensor.getDigitalPressure()
converted = sensor.convertPressureTemperature(pressure, temperature)
altitude = sensor.getMetricAltitude(converted, sensor.inHgToHectoPascal(29.95)) #set the altimeter setting appropriately
```
