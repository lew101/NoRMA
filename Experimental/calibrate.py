## Test program to indentify the location of the centre point of the joystick
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1015(i2c)

forward1 = AnalogIn(ads, ADS.P0)
forward2 = AnalogIn(ads, ADS.P1)
left1 = AnalogIn(ads, ADS.P2)
left2 = AnalogIn(ads, ADS.P3)

forward1_max = 0
forward1_min = 2048
left1_max = 0
left1_min = 2048
forward2_max = 0
forward2_min = 2048
left2_max = 0
left2_min = 2048

for sample in range(4096):

    f1 = forward1.value/16 
    f2 = forward2.value/16
    l1 = left1.value/16
    l2 = left2.value/16

    if f1 < forward1_min:
        forward1_min = f1
    
    if f2 < forward2_min:
        forward2_min = f2

    if f1 > forward1_max:
        forward1_max = f1
    
    if f2 > forward2_max:
        forward2_max = f2

    if l1 < left1_min:
        left1_min = l1
    
    if l2 < left2_min:
        left2_min = l2

    if l1 > left1_max:
        left1_max = l1
    
    if l2 > left2_max:
        left2_max = l2

    print(sample)

print("F1Max= {0}, F1Min= {1}, L1Max= {2}, L1Min= {3}".format(forward1_max, forward1_min, left1_max, left1_min))
print("F2Max= {0}, F2Min= {1}, L2Max= {2}, L2Min= {3}".format(forward2_max, forward2_min, left2_max, left2_min))