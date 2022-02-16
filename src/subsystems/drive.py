from ctre import WPI_TalonFX
import math
from utils import pid, imutil

# parameter : type
class Drive:
   #CONTRUCTOR
   def __init__(self, _frontLeft : WPI_TalonFX, _backLeft : WPI_TalonFX, _frontRight : WPI_TalonFX, _backRight : WPI_TalonFX, _drive_imu : imutil, _pid : pid):
      self.frontLeft = _frontLeft
      self.backLeft = _backLeft

      self.frontRight = _frontRight
      self.backRight = _backRight
      
      self.drive_imu = _drive_imu
      self.pid = _pid

   #HELPER FUNCTIONS
   def setRightSpeed(self, speed):
      # speed is a float value from -1 to 1
      speed = max(-1, min(speed, 1))
      self.frontRight.set(speed)
      self.backRight.set(speed)

   def setLeftSpeed(self, speed):
      # speed is a float value from -1 to 1
      speed = max(-1, min(speed, 1))
      self.frontLeft.set(speed)
      self.backLeft.set(speed)
   
   def setSpeed(self, left, right):
      self.setLeftSpeed(left)
      self.setRightSpeed(right)

   def getYaw(self):
      return self.drive_imu.getYaw()

   #DRIVE FUNCTIONS
   def arcadeDrive(self, y, x):
      left_speed = y + x
      right_speed = y - x
      self.setSpeed(left_speed, right_speed)

   def TankDrive(self, right_y, left_y):
      left_speed = left_y
      right_speed = right_y
      self.setSpeed(left_speed, right_speed)

   def absoluteDrive(self, speed, leftright, desired_angle, mult):
      # speed is a float value from -1 to 1
      cur_rotation = self.drive_imu.getYaw()
        # finds angle difference (delta angle) in range -180 to 180
      delta_angle = desired_angle - cur_rotation
      delta_angle = ((delta_angle + 180) % 360) - 180
        # PID steering power limited between -12 and 12
      steer = max(-12, min(12, self.pid.steer_pid(delta_angle)))
      left_speed = speed / 12
      right_speed = speed / 12
      left_speed -= steer / 12
      right_speed += steer / 12
        #self._drive.DifferentialDrive(left, right)
      # self._drive.arcadeDrive(left,steer)
      # Use PID or something in this next step idk
      #self.setSpeed(left_speed, right_speed)
      
      self.frontLeft.set((-speed - leftright + steer) * mult)
      self.frontRight.set((-speed + leftright - steer) * mult)
      self.backLeft.set((-speed + leftright + steer) * mult)
      self.backRight.set((-speed - leftright - steer) * mult)

   #Drive method for mecanum wheels
   def mecanumDrive(self, joy_y, joy_x, desired_angle):
      #Set power without turning
      flspeed = joy_y + joy_x
      frspeed = joy_y - joy_x
      brspeed = flspeed
      blspeed = frspeed
      #Add turning
      # cur_rotation = self.drive_imu.getYaw()
      # delta_angle = desired_angle - cur_rotation
      # delta_angle = ((delta_angle + 180) % 360) - 180
      # steer = max(-1, min(1, self.pid.steer_pid(delta_angle)/12))
      steer = 0
      flspeed -= steer
      frspeed += steer
      blspeed -= steer
      brspeed += steer
      #Set speed for all wheels
      self.frontLeft.set(flspeed)
      self.frontRight.set(frspeed)
      self.backLeft.set(blspeed)
      self.backRight.set(brspeed)

   def driftDrive(self, left, right, x):
      speed = left + right
      speed_left = speed - x
      speed_right = speed + x
      self.setLeftSpeed(speed_left)
      self.setRightSpeed(speed_right)
   