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

