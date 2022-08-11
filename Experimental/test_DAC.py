# Test program to control test the functionality of the DAC when interacting with the modified GC2 controller
# Program moves the wheelchair forwards, stop, then moves the wheelchair backwards
import board
import adafruit_mcp4728

i2c = board.I2C()   # uses board.SCL and board.SDA
dac =  adafruit_mcp4728.MCP4728(i2c)

BASE = 19500*2

# 65535 is max voltage, 0 is min. 16-bit is 65536

def CentTo16bit(percentage):
    return (int(percentage/100*65536)-1)

def set_vals(a,b,c,d):
    dac.channel_a.value = a
    dac.channel_b.value = b
    dac.channel_c.value = c
    dac.channel_d.value = d

def set_all(x):
    print(x)
    set_vals(x,x,x,x)

val = BASE
set_all(val)

input("Press enter")
val = int(BASE*1.4)
set_vals(val,val,BASE,BASE)

input("Press enter")
val = BASE
set_all(val)

input("Press enter")
val = int(BASE*0.6)
set_vals(val,val,BASE,BASE)

input("Press enter")
val = BASE
set_all(val)

input("Press enter")
val = 0
set_all(val)