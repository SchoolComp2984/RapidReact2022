import math, ctre, wpilib
from subsystems.drive import Drive
from subsystems.intaker import Intaker
from subsystems.ball_sensor import BallSensor

class Intake:

   DEFAULT = -1
   IDLE = 0
   TURNING = 1
   MOVING = 2

   state = TURNING

   minBallDistance = -150
   maxBallDistance = -100

   startSpinTime = 0

   def __init__(self, _drive : Drive, _intaker : Intaker, _ball_sensor : BallSensor):
      self.drive = _drive
      self.intaker = _intaker
      self.ball_sensor = _ball_sensor

   def execute(self, motor_power_mult):
      retval = False
      if self.state == self.IDLE:
         retval = False # not using motors
         if self.intaker.getCameraInfo()[0]:
            self.state = self.TURNING
      elif self.state == self.TURNING:
         delta_angle = self.intaker.getCameraInfo()[1] # get angle of target
         self.target_angle = self.drive.getYaw() - delta_angle
         self.drive.absoluteDrive(0, 0, self.target_angle, False, motor_power_mult)
         retval = True # using motors
         if delta_angle < 2 and delta_angle > -2: #limit angle in degrees
            self.state = self.MOVING
      elif self.state == self.MOVING:
         self.state = self.IDLE
         retval = False # not using motors
         # power = 0
         # delta_angle = self.intaker.getCameraInfo()[1] # get angle of target
         # self.target_angle = self.drive.getYaw() - delta_angle
         # if self.intaker.getCameraInfo()[2] < self.minBallDistance:
         #    power = .2 #back up if too close
         # if self.intaker.getCameraInfo()[2] > self.maxBallDistance:
         #    power = -.5 #move forward if too far
         # if not self.intaker.hasTarget():
         #    power = 0
         # self.drive.absoluteDrive(power, 0, self.target_angle, motor_power_mult)
         # if self.intaker.getCameraInfo()[2] < -120 and self.intaker.getCameraInfo()[2] > -150:
         #    self.state = self.IDLE
      else:
         pass
      return retval

   def reset(self):
      self.state = self.IDLE