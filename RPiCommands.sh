sudo apt-get install i2c-tools python3-smbus 
# Follow these steps from https://gallaugher.com/makersnack-installing-circuitpython-on-a-raspberry-pi/ :
# - Press the down arrow on the keyboard until “Interfacing Options” is highlighted, then press return.
# - Press the down arrow until I2C is highlighted, then press return.
# - When asked if you want to enable I2C, select <Yes> (use the arrow keys to highlight <Yes> if it’s not already selected), then press return.
# - Press return again on the highlighted <OK> confirmation.

#sudo apt-get install python3-pyqt5 python3-dev
sudo pip3 install rpi_ws281x
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3 1
#sudo apt-get install -y libopencv-dev python3-opencv
sudo apt-get install -y python3-pil python3-tk
pip3 install RPI.GPIO
pip3 install adafruit-blinka
pip3 install adafruit-circuitpython-mpu6050

# Also, to use the gyroscope, you must to remove/comment out "start_x=1" under /boot/config.txt and then reboot, or else
# GPIO 0 and 1 which use the gyroscope will not detect anything. Source:
# https://www.raspberrypi.org/forums/viewtopic.php?t=241637