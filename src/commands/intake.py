import math, ctre, wpilib
from subsystems.drive import Drive
from subsystems.intaker import Intaker
from subsystems.ball_sensor import BallSensor

class Intake:

   DEFAULT = -1
   IDLE = 0
   TURNING = 1
   MOVING = 2

   STATE = TURNING

   minBallDistance = -150
   maxBallDistance = -100

   startSpinTime = 0

   def __init__(self, _drive : Drive, _intaker : Intaker, _ball_sensor : BallSensor):
      self.drive = _drive
      self.intaker = _intaker
      self.ball_sensor = _ball_sensor

   def execute(self, motor_power_mult):
      if self.STATE == self.IDLE:
         if self.intaker.getCameraInfo()[0] and self.ball_sensor.getBallStatus():
            self.STATE = self.TURNING
      elif self.STATE == self.TURNING:
         delta_angle = self.intaker.getCameraInfo()[1] # get angle of target
         self.target_angle = self.drive.getYaw() - delta_angle
         self.drive.absoluteDrive(0, 0, self.target_angle, motor_power_mult)
         if delta_angle < 2 and delta_angle > -2: #limit angle in degrees
            self.STATE = self.MOVING
      elif self.STATE == self.MOVING:
         power = 0
         delta_angle = self.intaker.getCameraInfo()[1] # get angle of target
         self.target_angle = self.drive.getYaw() - delta_angle
         if self.intaker.getCameraInfo()[2] < self.minBallDistance:
            power = .2 #back up if too close
         if self.intaker.getCameraInfo()[2] > self.maxBallDistance:
            power = -.5 #move forward if too far
         if not self.intaker.hasTarget():
            power = 0
         self.drive.absoluteDrive(power, 0, self.target_angle, motor_power_mult)
         if self.intaker.getCameraInfo()[2] < -120 and self.intaker.getCameraInfo()[2] > -150:
            self.startSpinTime = wpilib.Timer.getFPGATimestamp()
            self.STATE = self.IDLE
      else:
         pass

   def idle(self):
      pass

   def turn(self):
      pass

   def move(self):
      pass

   def consume(self):
      pass