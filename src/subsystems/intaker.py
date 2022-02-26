from ctre import WPI_TalonSRX
from networktables import NetworkTables
import math
from utils import math_functions

#best power is somewhere between 3.2-3.9 v and 3.1-3.6 a
class Intaker:

   INTAKE_POWER = 3.4
   
   def __init__(self, _lift_motor : WPI_TalonSRX, _spin_motor : WPI_TalonSRX) -> None:
      self.lift_motor = _lift_motor
      self.spin_motor = _spin_motor

   def spin(self, dir : int):
      dir = math.copysign(1.0, dir)
      self.spin_motor.setVoltage(self.INTAKE_POWER * dir)
   
   def stop(self):
      self.spin_motor.set(0)

   def lower(self):
      pass

   def _raise(self):
      pass

   def getCameraInfo(self):
      networkTableData = NetworkTables.getTable("SmartDashboard")
      hasBall = networkTableData.getNumber("intake-ball", False)
      x = networkTableData.getNumber("intake-ball-offset-x", 0) # -200 to 200 px
      y = networkTableData.getNumber("intake-ball-offset-y", 0) # -150 to 150 px
      data = [hasBall, x, y]
      return data

   def hasTarget(self):
      return self.networkTableData.getNumber("intake-ball")
