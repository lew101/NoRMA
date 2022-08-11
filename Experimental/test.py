import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1015(i2c)

MAX = 2047
FORWARD_CENTER = 1233
LEFT_CENTER = 1232
DEVIATION_ERROR = 10

## Individual pin reading assignement
forward1 = AnalogIn(ads, ADS.P0)
forward2 = AnalogIn(ads, ADS.P1)
left1 = AnalogIn(ads, ADS.P2)
left2 = AnalogIn(ads, ADS.P3)

## Difference readings assignement
forwardDiff = AnalogIn(ads, ADS.P0, ADS.P1)
leftDiff = AnalogIn(ads, ADS.P2, ADS.P3)

# while True:
print("F1 {} F2 {} L1 {} L2 {}".format(forward1.value, forward2.value, left1.value, left2.value))