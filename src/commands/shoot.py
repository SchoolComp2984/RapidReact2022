import math
from subsystems.drive import Drive
from subsystems.shooter import Shooter
from utils import math_functions

class Shoot:
   TURNING = 0
   MOVING = 1
   SHOOTING = 2
   DISCARDING = 3
   IDLE = 4
   POSITIONING = 5
   SPINNING = 6
   FIRING = 7
   DEFAULT = -1
   state = TURNING
   min_LimelightDistance = 18 #angle
   max_LimelightDistance = 3 #angle

   def __init__(self, _drive : Drive, _shooter : Shooter):
      self.drive = _drive
      self.shooter = _shooter
      self.target_angle = self.drive.getYaw() - 180

   def turning(self, motor_power_multiplyer):
      if self.shooter.hasTarget():
         delta_angle = self.shooter.getCameraInfo()[1] # get angle of target
         self.target_angle = self.drive.getYaw() - delta_angle
         if abs(delta_angle) < 2: # if aim is accurate enough
            self.state = self.MOVING # set state to shooting
      self.drive.absoluteDrive(0, 0, self.target_angle, motor_power_multiplyer)

   def moving(self):
      if self.ball_color == self.alliance_color:
         self.state = self.SHOOTING
      else:
         self.state = self.DISCARDING

   def positioning(self, motor_power_multiplyer):
      power = 0
      delta_angle = self.shooter.getCameraInfo()[1] # get angle of target
      self.target_angle = self.drive.getYaw() - delta_angle
      if self.shooter.getCameraInfo()[2] > self.min_LimelightDistance:
         power = -.5
      if self.shooter.getCameraInfo()[2] < self.max_LimelightDistance:
         power = .5
      self.drive.absoluteDrive(power, 0, self.target_angle, motor_power_multiplyer)
      return (power == 0 and abs(delta_angle) < 2)

   def spinning(self, motor_power_multiplyer):
      delta_angle = self.shooter.getCameraInfo()[1] # get angle of target
      self.target_angle = self.drive.getYaw() - delta_angle
      self.drive.absoluteDrive(0, 0, self.target_angle, motor_power_multiplyer)
      
      vel = math_functions.shootInterp(self.shooter.getCameraInfo()[2])
      current_vel = self.shooterMotor.getSelectedSensorVelocity(0)
      delta = vel-current_vel
      pwr = delta * 0.001
      if (pwr > 0.99):
         pwr = 0.99
      if (pwr < 0):
         pwr = 0
      if (vel == 0):
         pwr = 0
      #self.shooterMotor.set(ctre.TalonFXControlMode.PercentOutput, pwr)

   def shooting(self):
      #shoot ball
      pass
   def discarding(self):
      #get rid of wrong-color ball
      pass
   
   #always run this function in robot.py teleop
   def auto_execute(self, button_pressed, motor_power_multiplyer):
      if self.state == self.IDLE:
         if button_pressed:
            ball = self.shooter.getBallStatus()
            if (ball == True or ball == False) and self.shooter.hasTarget(): #if robot has ball and sees target
               self.state = self.POSITIONING
      if self.state == self.POSITIONING:
         # roughly turn and move into min/max distance
         if button_pressed:
            if self.positioning(motor_power_multiplyer):
               self.state = self.SPINNING
         else:
            self.state = self.IDLE
      if self.state == self.SPINNING:
         if button_pressed:
            self.spinning(motor_power_multiplyer)
         else:
            self.state = self.IDLE

   def finish(self):
      self.state = self.TURNING

   def execute(self, state):
      if self.state == self.TURNING:
         self.turning()
      if self.state == self.MOVING:
         self.moving()
      if self.state == self.SHOOTING: # if or elif
         #Code for shoot the ball
         self.shooting()
         #if shooting is done:
         self.finish()
      if self.state == self.DISCARDING:
         self.discarding()