from re import L
import board
import busio
import adafruit_ads1x15.ads1015 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_mcp4728

i2c = busio.I2C(board.SCL, board.SDA)
adc = ADS.ADS1015(i2c)
dac = adafruit_mcp4728.MCP4728(i2c)

class Joystick():
    def __init__(self):
        self.MAX = 2047
        self.FB_CENTER = 1233
        self.LR_CENTER = 1232
        self.DEVIATION_ERROR = 15

        i2c = busio.I2C(board.SCL, board.SDA)
        adc = ADS.ADS1015(i2c)
        self.dac = adafruit_mcp4728.MCP4728(i2c)

        ## Individual pin reading assignement
        self.in_fb_1 = AnalogIn(adc, ADS.P0)
        self.in_fb_2 = AnalogIn(adc, ADS.P1)
        self.in_lr_1 = AnalogIn(adc, ADS.P2)
        self.in_lr_2 = AnalogIn(adc, ADS.P3)

        ## Difference readings assignement
        self.fb_Diff = AnalogIn(adc, ADS.P0, ADS.P1)
        self.lr_Diff = AnalogIn(adc, ADS.P2, ADS.P3)

    def __get_forward_backward(self):
        """Gets raw FB values.
        As two sensors are being read, this will return the midpoint between the two. 

        Returns:
            int: The overall forward/back value
        """
        # Divides by 16 as ADC is 12 bit but is outputting a 16 bit int
        ## use .voltage or .value
        fb1 = self.in_fb_1.value/16 # Forward sensor 1
        fbd = self.fb_Diff.value/16 # Difference between Forward sensor 1 and 2
        return fb1-fbd/2 # Overall forward
    
    def __get_left_right(self):
        """Gets raw LR values.
        As two sensors are being read, this will return the midpoint between the two. 

        Returns:
            int: The overall left/right value.
        """
        lr1 = self.in_lr_1.value/16 # Left sensor 1
        lrd = self.lr_Diff.value/16 # Difference between Left sensor 1 and 2
        return lr1-lrd/2 # Overall left

    def __set_DAC_vals(self,fb_1,fb_2,lr_1,lr_2):
        """Sets raw DAC values

        Args:
            fb_1 (int): Forward/backward 1
            fb_2 (int): Forward/backward 2
            lr_1 (int): Left/right 1
            lr_2 (int): Left/right 2
        """
        self.dac.channel_a.value = int(fb_1)
        self.dac.channel_b.value = int(fb_2)
        self.dac.channel_c.value = int(lr_1)
        self.dac.channel_d.value = int(lr_2)

    def __deviation_lockout(self):
        """Will lockout the joystick by simulating a joystick deviation.
        Should be called when the joystick deviation is outside normal ranges.
        """
        self.__set_DAC_vals(65535,0,0,65535) # 65535 is max voltage, 0 is min. 16-bit is 65536

    def set_calibration_vals(self, fb_val, lr_val):
        """Sets the joystick calibration values.
        Can be used to set the calibration values to known good values.

        Args:
            fb_val (int): The forward backward calibration value
            lr_val (int): The left right calibration value
        """
        self.FB_CENTER = fb_val
        self.LR_CENTER = lr_val

    def calibrate(self, samples = 1024):
        """Calibrates the joystick centre.
        Records a number of samples of the joystick and then uses the average as the centre.

        Args:
            samples (int, optional): The number of samples to take for calibration. Defaults to 1024.
        """
        forwardCalib = []
        leftCalib = []
        
        for _ in range (samples):
            forwardCalib.append(self.__get_forward_backward())
            leftCalib.append(self.__get_left_right())

        self.__set_calibration_vals(
            round(sum(forwardCalib)/len(forwardCalib)),
            round(sum(leftCalib)/len(leftCalib))
        )

    def getPercent(self):
        """Gets the joystick values as a percentage between -100 and 100 for forward/back and left/right
        
        Returns:
            float: The overall forward/backward value as a percentage between -100 and 100.
            float: The overall left/right value as a percentage between -100 and 100.
        """
        fb_D = self.fb_Diff.value/16 # Difference between Forward sensor 1 and 2
        lr_D = self.lr_Diff.value/16 # Difference between Left sensor 1 and 2

        fb_amount = (self.__get_forward_backward()-self.FB_CENTER)/720*100
        lr_amount = (self.__get_left_right()-self.LR_CENTER)/720*100

        ## Deviation error detection
        if (fb_D >= self.DEVIATION_ERROR or lr_D >= self.DEVIATION_ERROR):
            self.__deviation_lockout()

        return(fb_amount,lr_amount)

    def setPercent(self, fb, lr):
        """Sets the position of the virtual joystick as a percentage between -100 and 100.

        Args:
            fb (float): _description_
            lr (float): _description_
        """

        fb_amount = (16 * ((fb/100*720) + self.FB_CENTER)) * 2
        lr_amount = (16 * ((lr/100*720) + self.LR_CENTER)) * 2

        self.__set_DAC_vals(fb_amount, fb_amount, lr_amount, lr_amount)

        return(fb_amount, lr_amount)
