import math, ctre, wpilib
from rev import RelativeEncoder
from subsystems.drive import Drive
from subsystems.shooter import Shooter
from utils import math_functions, ID, pid

class Shoot:
   IDLE = 0
   POSITIONING = 1
   SPINNING = 2
   FIRING = 3
   RELAXING = 4
   state = IDLE
   min_LimelightDistance = 18 #angle
   max_LimelightDistance = 0 #angle
   flywheel_desiredSpeed = 0

   def __init__(self, _drive : Drive, _shooter : Shooter):
      self.drive = _drive
      self.shooter = _shooter
      self.target_angle = self.drive.getYaw() - 180
      self.pidshoot = pid.PID()
      self.pidshoot.set_pid(0.0002, 0.000005, 0.0002, 0)

   def positioning(self, motor_power_multiplyer):
      power = 0
      delta_angle = self.shooter.getCameraInfo()[1] # get angle of target
      self.target_angle = self.drive.getYaw() - delta_angle
      if self.shooter.getCameraInfo()[2] > self.min_LimelightDistance:
         power = .2 #back up if too close
      if self.shooter.getCameraInfo()[2] < self.max_LimelightDistance:
         power = -.5 #move forward if too far
      if not self.shooter.hasTarget():
         power = 0
         self.target_angle = self.drive.getYaw()
      self.drive.absoluteDrive(power, 0, self.target_angle, False, motor_power_multiplyer)
      print ("limelight x, limelight y: ", self.shooter.getCameraInfo()[1], self.shooter.getCameraInfo()[2])
      return (power == 0 and abs(delta_angle) < 2 and self.shooter.hasTarget())

   def spinning(self, ball, motor_power_multiplyer):
      delta_angle = self.shooter.getCameraInfo()[1] # get angle of target
      self.target_angle = self.drive.getYaw() - delta_angle
      if ball == False:
         self.target_angle -= 22 # shoot away from target
      if self.shooter.getCameraInfo()[2] < 10:
         forwards = -.2
         print ("Forwards state= ", self.state, motor_power_multiplyer)
      else:
         forwards = 0
      self.drive.absoluteDrive(forwards, 0, self.target_angle, False, motor_power_multiplyer)
      self.flywheel_desiredSpeed = math_functions.shootInterp(self.shooter.getCameraInfo()[2])
      if ball == False:
         return abs(delta_angle) > 22-2
      return (abs(delta_angle) < 2)

   def firing(self):
      self.flywheel_desiredSpeed = math_functions.shootInterp(self.shooter.getCameraInfo()[2])
      self.shooter.transportServo.setAngle(180)

   def spin_pid(self, debug, motor_power_multiplier, manual_enabled):
      current_vel = self.shooter.shooterMotor.getSelectedSensorVelocity(0)
      delta = self.flywheel_desiredSpeed - current_vel
      #pwr = delta * 0.001
      pwr = self.pidshoot.shoot_pid(delta)
      if (pwr > 0.99):
         pwr = 0.99
      if (pwr < -0.5):
         pwr = -0.5
      if (self.flywheel_desiredSpeed == 0):
         pwr = 0
      if not manual_enabled:
         self.shooter.shooterMotor.set(ctre.TalonFXControlMode.PercentOutput, pwr * motor_power_multiplier)
      if debug:
         print ("spin: ", current_vel, " ", self.flywheel_desiredSpeed, " ", delta)
      return (abs(delta) < self.flywheel_desiredSpeed/10) # within 10% of desired speed
   
   def transporting(self, angle, manual_enabled):
      if not manual_enabled:
         self.shooter.transportServo.setAngle(angle)

   #always run this function in robot.py teleop
   #THX FOR PUTTING THE MANUAL STUFF IN, GOOD WORK - GREG
   def execute(self, button_pressed, manual_transport_enabled, manual_shooter_enabled, motor_power_multiplyer):
      retval = False # not taking control of drive motors
      self.spin_pid(False, motor_power_multiplyer, manual_shooter_enabled)
      # THIS CODE MIGHT NOT WORK BECAUSE THE DESIREDSPEED MIGHT BE SWITCHED
      if manual_shooter_enabled:
         self.flywheel_desiredSpeed = 7500
      else:
         self.flywheel_desiredSpeed = 0
      if self.state == self.IDLE:
         # if manual_transport_enabled:
         #    self.transporting(ID.SERVO_MAX)
         # else:
         #    self.transporting(ID.SERVO_MIN)
         # THIS GOT MOVED TO OUTSIDE THE FUNCTION IN ROBOT.PY
         if button_pressed:
            self.ball = self.shooter.getBallStatus()
            #self.shooter.printBallStatus()
            if (self.ball == True or self.ball == False) and self.shooter.hasTarget():
               self.retval = True
               self.state = self.POSITIONING
               print ("state=positioning ", self.ball)
      elif self.state == self.POSITIONING:
         # roughly turn and move into min/max distance
         if button_pressed:
            retval = True # controlling drive motors
            self.flywheel_desiredSpeed = math_functions.shootInterp(self.shooter.getCameraInfo()[2])
            if self.positioning(motor_power_multiplyer):
               self.state = self.SPINNING
               self.pidshoot.integral = 0
               print ("state=spinning")
         else:
            self.state = self.IDLE
            print ("state=idle")
      elif self.state == self.SPINNING:
         if button_pressed:
            retval = True # controlling drive motors
            if self.spinning(self.ball, motor_power_multiplyer) and self.spin_pid(True, motor_power_multiplyer, manual_shooter_enabled):
               self.state = self.FIRING
               print ("state=firing")
               self.startServoTime = wpilib.Timer.getFPGATimestamp()
         else:
            self.state = self.IDLE
            print ("state=idle")
      elif self.state == self.FIRING:
         if button_pressed:
            retval = True # controlling drive motors
            self.spinning(self.ball, motor_power_multiplyer)
            self.transporting(ID.SERVO_MAX, manual_transport_enabled)
            if self.startServoTime + 0.4 < wpilib.Timer.getFPGATimestamp():
               self.state = self.RELAXING
               print ("state=relaxing")
         else:
            self.state = self.RELAXING
      elif self.state == self.RELAXING:
         self.transporting(ID.SERVO_MIN, manual_transport_enabled)
         if self.startServoTime + 0.8 < wpilib.Timer.getFPGATimestamp():
            self.state = self.IDLE
            print ("state=idle")
      else:
         self.state = self.IDLE
         print ("state=idle")
      return retval