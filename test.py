
from MS5607 import MS5607

sensor = MS5607()
samples = 10
temperature = 0
pressure = 0
for i in range(samples):
	temperature += sensor.getDigitalTemperature()
	pressure += sensor.getDigitalPressure()

temperature /= samples
pressure /= samples
converted = sensor.convertPressureTemperature(pressure,temperature)
altitude = sensor.getMetricAltitude(converted, sensor.inHgToHectoPascal(29.95))

tempC = sensor.getTemperature()
tempF = tempC * 1.8 + 32
print("Alt: ",altitude, " meters ASL")
print("Alt: ",altitude/(3.23), " feet ASL")
print("Temp C: ", tempC)
print("Temp F: ", tempF)

