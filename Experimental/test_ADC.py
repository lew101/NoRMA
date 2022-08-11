# Test program to control test the functionality of the ADC when interacting with the modified GC2 controller
# Initially completes a calibration and then outputs the joystick position as a percentage or forward and left
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

def getForward():
    # Divides by 16 as ADC is 12 bit but is outputting a 16 bit int
    ## use .voltage or .value
    f1 = forward1.value/16 # Forward sensor 1
    fd = forwardDiff.value/16 # Difference between Forward sensor 1 and 2
    return f1-fd/2 # Overall forward
    

def getLeft():
    l1 = left1.value/16 # Left sensor 1
    ld = leftDiff.value/16 # Difference between Left sensor 1 and 2
    return l1-ld/2 # Overall left

## Callibration loop
## Reads 1024 samples and averages them to find the electrical center.
forwardCalib = []
leftCalib = []
print("Sensor calibration, keep joystick centered")
input("Press enter to continue...")
print("measuring center")
for sample in range (1024):
    forwardCalib.append(getForward())
    leftCalib.append(getLeft())

FORWARD_CENTER = round(sum(forwardCalib)/len(forwardCalib))
LEFT_CENTER = round(sum(leftCalib)/len(leftCalib))

print("Done")
print("F={0}, L={1}".format(FORWARD_CENTER, LEFT_CENTER))
input("Press enter to continue...")

while True:

    f1 = forward1.value/16 # Forward sensor 1
    fd = forwardDiff.value/16 # Difference between Forward sensor 1 and 2
    l1 = left1.value/16 # Left sensor 1
    ld = leftDiff.value/16 # Difference between Left sensor 1 and 2

    f = f1-fd/2 # Overall forward
    l = l1-ld/2 # Overall left

    f_amount = (getForward()-FORWARD_CENTER)/720*100
    l_amount = (getLeft()-LEFT_CENTER)/720*100

    ## Deviation error detection
    if (fd >= DEVIATION_ERROR):
        print("[ERROR] Large forward sensor deviation")
    if (ld >= DEVIATION_ERROR):
        print("[ERROR] Large left sensor deviation")
        
    ## Parking break estimate
    if (f_amount <= 10 and l_amount <= 10 and f_amount >= -10 and l_amount >= -10):
        print("[BRAKE]")

    print("F= {:.0f}%, L= {:.0f}%".format(f_amount,l_amount))