# Test program to test ADC and DAC cooperative operation
# Program attempts to replicate the loopback connector by passing the joystick signals straight back to the GC2
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_mcp4728

i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)
dac = adafruit_mcp4728.MCP4728(i2c)

forward1 = AnalogIn(adc, ADS.P0)
forward2 = AnalogIn(adc, ADS.P1)
left1 = AnalogIn(adc, ADS.P2)
left2 = AnalogIn(adc, ADS.P3)

while True:
    f1 = forward1.value
    f2 = forward2.value
    l1 = left1.value
    l2 = left2.value

    print("F1 {} F2 {} L1 {} L2 {}".format(f1, f2, l1, l2))
    dac.channel_a.value = f1*2
    dac.channel_b.value = f2*2
    dac.channel_c.value = l1*2
    dac.channel_d.value = l2*2