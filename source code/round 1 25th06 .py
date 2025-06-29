from gpiozero import Robot, Motor, Pin
from gpiozero.pins.rpigpio import RPiGPIOPin
from gpiozero import DistanceSensor
import serial, time
import numpy as np
import logging
from logging import handlers
import time
import sys
sys.path.append("/home/pi/bob/lib/python3.11/site-packages")#
import board
import busio
import adafruit_mpu6050
#Code for error correction and keeping the robot to go straight#
From mpu6050 import mpu6050
from simple_pid import PID
import time

# Initialize MPU6050
sensor = mpu6050(0x68)

# PID controller for yaw correction
pid = PID(1.0, 0.0, 0.05, setpoint=0)  # Tune Kp, Ki, Kd as needed
pid.output_limits = (-50, 50)  # Limit the correction

def get_yaw():
    gyro_data = sensor.get_gyro_data()
    # Integrate gyro Z-axis for yaw (simple approach)
    # In practice, use sensor fusion for better accuracy
    return gyro_data['z']

#
total = 0
i2c = busio.I2C(board.SCL, board.SDA)
mpu = adafruit_mpu6050.MPU6050(i2c)

log_filename = "wro-2024.log"
handler = handlers.TimedRotatingFileHandler(log_filename, 'midnight', 10)
FORMAT = "%(asctime)s.%(msecs)03d %(name)s %(levelname)s %(lineno)d %(message)s"
formatter = logging.Formatter(FORMAT)
handler.setFormatter(formatter)
logging.basicConfig(level=logging.INFO, format=FORMAT)

logger = logging.getLogger(__name__)
logger.addHandler(handler)

class Robot:
    def __init__(self, driving, steering, luna):
        self.driving = driving
        self.steering = steering
        self.luna = luna

    def move_forward(self, speed=0.5):
        self.driving.forward(speed=speed)

    def turn_right(self, speed=0.5):
        self.steering.forward(speed=speed)
        self.wait(timeout=0.8)
        self.move_forward(speed=0.5)

    def turn_left(self, speed=0.5):
        self.steering.backward(speed=speed)
        self.move_forward(speed=speed)

    def move_backward(self, speed=0.5):
        self.driving.backward(speed=speed)

    def stop(self):
        self.driving.stop()
        self.steering.stop()
        self.wait(timeout=0.1)

    def stop_steering(self):
        self.steering.stop()
        self.wait(timeout=0.1)

    def wait(self, timeout=1):
        try:
            time.sleep(timeout)
        except:
            pass

    def keep_straight(self):
        self.stop_steering()
        self.turn_left(speed=0.2)
        self.wait(timeout=0.1)
        self.turn_right(speed=0.2)
        self.wait(timeout=0.1)
        self.stop_steering()

    def move_back(self, speed=0.5):
        self.turn_left(speed=0.8)
        self.move_backward(speed=speed)
        self.wait(1)
        self.keep_straight()
        self.wait(timeout=0.5)
        self.stop()

    def move_straight(self, speed=0.5):
        front_distance = self.luna.get_distance()
        self.safeguard(front_distance)
        self.move_forward(speed=speed)
        self.keep_straight()
#
        self.move_forward(speed=speed)#
            self.wait(timeout=0.1)
            front_distance = self.luna.get_distance()
            self.safeguard(front_distance)
            print(front_distance)
        self.safeguard(front_distance)
        self.stop()

    def move_left(self, speed=0.5):
        front_distance = self.luna.get_distance()
        self.safeguard(front_distance)
        self.turn_left(speed=speed)
        self.move_forward(speed=speed)

        timeout = 0.0
        while front_distance > 50 and timeout < 0.5:
            self.wait(timeout=0.1)
            timeout += 0.1
            front_distance = self.luna.get_distance()
        while total < 80:
            gyro_z = mpu.gyro[2] * 57.2958
            total += gyro_z
            self.turn_right(speed=speed)

        self.keep_straight()
        self.stop()

    def move_right(self, speed=1.0):
        front_distance = self.luna.get_distance()
        self.safeguard(front_distance)
        print("turning")#0
        gyro_z = mpu.gyro[1]
        total = gyro_z
        print(total)
        while total > -60:
            gyro_z = (mpu.gyro[1] * 57.2958)+2.2# 2.2 is a correction value to remove drift
            total += gyro_z
            print(total)#
            self.turn_right(speed=speed)

        timeout = 0.0
        self.keep_straight()
        self.stop()

  def move_right_45(self, speed=1.0):
        front_distance = self.luna.get_distance()
        self.safeguard(front_distance)
        print("turning")#0
        gyro_z = mpu.gyro[1]
        total = gyro_z
        print(total)
        while total > -45:
            gyro_z = (mpu.gyro[1] * 57.2958)+2.2# 2.2 is a correction value to remove drift
            total += gyro_z
            print(total)#
            self.turn_right(speed=speed)

        timeout = 0.0
        self.keep_straight()
        self.stop()
    def safeguard(self, front_distance):
        self.stop()
        pass
    def coin_flip():
      right = True
      robot.move_right_45(1.0)
      if distance <20:
          right = False
      else:
          right = True
class Luna:
    def __init__(self, ser):
        self.ser = ser

    def get_distance(self):
        return self.read_tf_luna()

    def read_tf_luna(self):
        while True:
            counter = self.ser.in_waiting
            if counter > 8:
                bytes_serial = self.ser.read(9)
                self.ser.reset_input_buffer()

                if bytes_serial[0] == 0x59 and bytes_serial[1] == 0x59:
                    distance = bytes_serial[2] + bytes_serial[3] * 256
                    return distance
    
def move_in_lap_right(robot, forward_speed=0.5, turn_speed=0.5):
    print("hello")
    gyro_z = 0#
    #robot.move_straight(speed=forward_speed)
    print("turning right")
    robot.move_right(speed=turn_speed)
    print("moving forward")
    gyro_z = 0#
    robot.move_straight(speed=forward_speed)
    print("turning right")#0000
    robot.move_right(speed=turn_speed)
    gyro_z = 0#
    robot.move_straight(speed=forward_speed)
    robot.move_right(speed=turn_speed)
    gyro_z = 0#
    robot.move_straight(speed=forward_speed)
    robot.move_right(speed=turn_speed)

def move_in_lap_right(robot, forward_speed=0.5, turn_speed=0.5):
    print("hello")
    gyro_z = 0#
    #robot.move_straight(speed=forward_speed)
    print("turning right")
    robot.move_left(speed=turn_speed)
    print("moving forward")
    gyro_z = 0#
    robot.move_straight(speed=forward_speed)
    print("turning right")#0000
    robot.move_left(speed=turn_speed)
    gyro_z = 0#
    robot.move_straight(speed=forward_speed)
    robot.move_left(speed=turn_speed)
    gyro_z = 0#
    robot.move_straight(speed=forward_speed)
    robot.move_left(speed=turn_speed)

def move(robot, num_laps=1, forward_speed=0.6, turn_speed=0.8):
    lap = 1
    if right == True:
      while lap <= num_laps:
          move_in_lap_right(robot, forward_speed=forward_speed, turn_speed=turn_speed)
          lap += 1
      robot.stop()
    elif right == False:
      while lap <= num_laps:
        move_in_lap_left(robot, forward_speed=forward_speed, turn_speed=turn_speed)
          lap += 1
      robot.stop()

if __name__ == "__main__":
    ser = serial.Serial("/dev/serial0", 115200, timeout=0)
    driving = Motor(17, 18)
    steering = Motor(22, 23)
    if not ser.isOpen():
        ser.open()
    luna = Luna(ser)
    robot = Robot(driving, steering, luna)
    robot.move_straight(0.5)
    robot.coin_flip()
    move(robot, 3, forward_speed=0.5, turn_speed=0.5)#
    #robot.move_right(speed=0.5,)#
