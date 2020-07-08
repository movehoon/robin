
import time

from board import SCL, SDA
import busio

from adafruit_pca9685 import PCA9685
from adafruit_motor import motor

i2c = busio.I2C(SCL, SDA)

pca = PCA9685(i2c, address=0x60)
pca.frequency = 100


