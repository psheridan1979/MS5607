from __future__ import print_function
import sys
from MS5607 import MS5607
import time
import csv
from os import path

file_path = '/home/pi/logs/altitude.csv'
print(file_path)
sensor = MS5607()
samples = 10
interval = 1

with open(file_path, 'wb') as csvfile:
#with open(file_path, 'w', newline='') as csvfile:
	writer = csv.writer(csvfile)
	writer.writerow(['ESTTime','AltM','AltFt','TempF','TempC','PressPa'])
	print("EST Time           AltM        AltFt        TempF       TempC    PressPa")
	while(1):
		d_temperature = 0
		d_pressure = 0
		for i in range(samples):
			d_temperature += sensor.getDigitalTemperature()
			d_pressure += sensor.getDigitalPressure()

		d_temperature /= samples
		d_pressure /= samples
		converted = sensor.convertPressureTemperature(d_pressure,d_temperature)
		altitudeM = sensor.getMetricAltitude(converted, sensor.inHgToHectoPascal(29.95))
		altitudeF = altitudeM * 3.28084
		tempC = sensor.getTemperature()
		tempF = tempC * 1.8 + 32
		pressPa = d_pressure
		est_time = time.time()
		writer.writerow([est_time,altitudeM,altitudeF,tempF,tempC,pressPa])
		print('\r' + ('%8.2f' % est_time), ('%8.2f' % altitudeM), ('%8.2f' % altitudeF) , ('%8.2f' % tempC) ,('%8.2f' % tempF) , ('%8.2f' % pressPa), sep='    ', end='')
		sys.stdout.flush()
		time.sleep(interval)
