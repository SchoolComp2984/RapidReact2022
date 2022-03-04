from ctre import WPI_TalonSRX
from networktables import NetworkTables
import math
from utils import math_functions
import rev

#best power is somewhere between 3.2-3.9 v and 3.1-3.6 a
class Intaker:

   #Big wheels
   INTAKE_POWER = 2.9
   
   def __init__(self, _lift_motor : WPI_TalonSRX, _spin_motor : WPI_TalonSRX, _upper_spin_motor : rev.CANSparkMax) -> None:
      self.top_motor = _lift_motor
      self.spin_motor = _spin_motor
      self.upper_spin_motor = _upper_spin_motor

   """
   def spin(self, dir : int):
      if dir < 0:
         self.spin_motor.setVoltage(self.INTAKE_POWER * -1.4)
      else:
         self.spin_motor.setVoltage(self.INTAKE_POWER)   
   """

   def spin(self, dir : int, both : bool):
      if dir < 0:
         if both:
            self.spin_motor.setVoltage(self.INTAKE_POWER * -1.4)
            self.upper_spin_motor.setVoltage(self.INTAKE_POWER * -1.4)
         else:
            self.spin_motor.setVoltage(self.INTAKE_POWER * -1.4)
      else:
         if both:
            self.spin_motor.setVoltage(self.INTAKE_POWER)
            self.upper_spin_motor.setVoltage(self.INTAKE_POWER)
         else:
            self.spin_motor.setVoltage(self.INTAKE_POWER)

   def stop(self):
      self.spin_motor.set(0)
      self.upper_spin_motor.set(0)

   #small wheels backwards
   def lower(self):
      self.top_motor.set(-0.2)

   #small wheels main
   def _raise(self):
      self.top_motor.set(0.2)

   def stop_lift(self):
      self.top_motor.set(0)

   def getCameraInfo(self):
      networkTableData = NetworkTables.getTable("SmartDashboard")
      hasBall = networkTableData.getNumber("ball_detected", False)
      x = networkTableData.getNumber("ball_", 0) # -200 to 200 px
      y = networkTableData.getNumber("ball_y", 0) # -150 to 150 px
      data = [hasBall, x, y]
      return data

   def hasTarget(self):
      return self.networkTableData.getNumber("intake-ball")
