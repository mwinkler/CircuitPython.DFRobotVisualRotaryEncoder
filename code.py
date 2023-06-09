import time
import board
from DFRobot_VisualRotaryEncoder import *

with board.I2C() as i2c:

    encoder = DFRobot_VisualRotaryEncoder(i2c, i2c_addr = 0x54)

    while (encoder.begin() == False):
        print ('Please check that the device is properly connected')
        time.sleep(3)

    while True:

        val = encoder.get_encoder_value()
        btn = encoder.detect_button_down()
        print(val, btn)

        time.sleep(0.1)