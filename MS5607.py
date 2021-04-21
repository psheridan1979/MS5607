from __future__ import print_function
import time
import smbus
import math
from numpy import std as standardDeviation


#bus = smbus.SMBus(1)

class MS5607:
    """
    http://www.parallaxinc.com/sites/default/files/downloads/29124-APPNote_520_C_code.pdf
    http://www.parallaxinc.com/sites/default/files/downloads/29124-MS5607-02BA03-Datasheet.pdf
    Offset for humidity to provide better precision
    """
    #DEVICE_ADDRESS = 0x76
    _CMD_RESET = 0x1E
    _CMD_ADC_READ = 0x00
    _CMD_PROM_RD = 0xA0
    _CMD_ADC_CONV = 0x40
    _CMD_ADC_D1 = 0x00
    _CMD_ADC_D2 = 0x10
    _CMD_ADC_256 = 0x00
    _CMD_ADC_512 = 0x02
    _CMD_ADC_1024 = 0x04
    _CMD_ADC_2048 = 0x06
    _CMD_ADC_4096 = 0x08

    def __init__(self, bus_num = 1, address = 0x76):
        self.bus_num = bus_num
        self.address = address
        self.bus = smbus.SMBus(bus_num)
        self.resetSensor()
        self.coefficients = self.readCoefficients()
        self.sea_level_pressure = 101325 #in HPa * 100, sea level is 1013.25 HPa
        self.ground_level_pressure = self.sea_level_pressure 
    # Some utility methods
    def read16U(self, register1, register2):
        bytes = self.bus.read_i2c_block_data(self.address, register1, 2)
        return (bytes[0] << 8) + (bytes[1])
    def read24U(self, register):
        bytes = self.bus.read_i2c_block_data(self.address, register, 3)
        return (bytes[0] << 16) + (bytes[1] << 8) + bytes[2]
    def hectoPascalToInHg(self, milliBar):
        return milliBar * 29.5333727 / 100000
    def inHgToHectoPascal(self, inHg):
        return 100 * 1000 * inHg / 29.5333727
    def getImperialAltitude(self, currentMilliBar, baseMilliBar):
        return (1 - math.pow(currentMilliBar / baseMilliBar, .190284)) * 145366.45
    def getMetricAltitude(self, currentMilliBar, baseMilliBar):
        return 0.3048 * self.getImperialAltitude(currentMilliBar, baseMilliBar)
    def getExpectedPressureAtAltitude(self, altitude):
        return self.sea_level_pressure * exp(-0.00012 * altitude)
    def setGroundLevel(self, known_altitude, samples=500):
        temperature = []
        pressure = []
        temperature_sum = 0
        pressure_sum = 0
        for i in range(samples):
            t = sensor.getDigitalTemperature()
            temperature.append(t)
            temperature_sum += t
            p = sensor.getDigitalPressure()
            pressure.append(p)
            pressure_sum += p
        meant_temp = temperature_sum / samples
        mean_pressure = presure_sum / samples 
        temp_sigma = standardDeviation(temperature)
        press_sigma = standardDeviation(pressure)
        converted = sensor.convertPressureTemperature(mean_pressure, meant_temp)
        asl_altitude = self.getMetricAltitude(converted, self.sea_level_pressure)
        print('Known ASL altitude: ', ('%8.2f' % known_altitude), ' m')
        print('Calculated ASL altitude: ', ('%8.2f' % asl_altitude), ' m')
        print('Sea level pressure: ', self.sea_level_pressure, ' hPa')
        expected_pressure = self.getExpectedPressureAtAltitude(known_altitude)
        print('Expected ground level pressure at ', known_altitude, ' meters: ', ('%8.2f' % expected_pressure), ' hPa')
        print('Measured ground level pressure: ', ('%8.2f' % converted), ' hPa')
        



    # Commands		
    def resetSensor(self):
        self.bus.write_byte(self.address, self._CMD_RESET)
        time.sleep(0.003) # wait for the reset sequence timing
    def readCoefficient(self, i):
        return self.read16U(self._CMD_PROM_RD + 2 * i, self._CMD_PROM_RD + 2 * i + 1)
    def readCoefficients(self):
        coefficients = [0] * 6
        for i in range(6):
            coefficients[i] = self.readCoefficient(i + 1)
        return coefficients
    def readAdc(self, cmd):
        # set conversion mode
        self.bus.write_byte(self.address, self._CMD_ADC_CONV + cmd)
        sleepTime = {self._CMD_ADC_256: 0.0009, self._CMD_ADC_512: 0.003, self._CMD_ADC_1024: 0.004, self._CMD_ADC_2048: 0.006, self._CMD_ADC_4096: 0.010}
        time.sleep(sleepTime[cmd & 0x0f])
        return self.read24U(self._CMD_ADC_READ)
    def getDigitalPressure(self):
        return self.readAdc(self._CMD_ADC_D1 + self._CMD_ADC_4096)
    def getDigitalTemperature(self):
        return self.readAdc(self._CMD_ADC_D2 + self._CMD_ADC_4096)
    def getTemperature(self):
        dT = self.getDigitalTemperature() - self.coefficients[4] * math.pow(2, 8)
        return (2000 + dT * self.coefficients[5] / math.pow(2, 23)) / 100
        
    def convertPressureTemperature(self, pressure, temperature):
        # if necessary, calculate 2nd order corrections
        T2 = 0
        off2 = 0
        sens2 = 0
        if  temperature < 20:
            T2 = (dT*dT) / (2 << 30)
            off2 = 61 * math.pow((temperature - 2000),2) / 16
            sens2 = 2 * math.pow((temperature - 2000),2)
            if temperature < -15:
                off2 = off2 + 15 * math.pow((temperature + 1500), 2)
                sens2 = sens2 + 8 * math.pow((temperature + 1500), 2)
        #apply 2nd order corrections are necessary
        temperature = temperature - T2
        # Calculate 1st order pressure and temperature
        dT = temperature - self.coefficients[4] * 256
        # Offset at actual temperature
        off = self.coefficients[1] * 4 + ((float(dT) / 2048) * (float(self.coefficients[3]) / 1024))
        off = off - off2
        # Sensitivity at actual temperature
        sens = self.coefficients[0] * 2 + ((float(dT) / 4096) * (float(self.coefficients[2]) / 1024))
        sens = sens - sens2
        # Temperature compensated pressure
        press = (float(pressure) / 2048) * (float(sens) / 1024) - off
        return press

