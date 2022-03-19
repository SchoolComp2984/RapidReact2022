from ctre import WPI_TalonSRX
from networktables import NetworkTables
import math
from utils import math_functions
import rev

#best power is somewhere between 3.2-3.9 v and 3.1-3.6 a
class Intaker:

   #Big wheels
   INTAKE_POWER = 2.4
   OUTAKE_POWER = -3.2
   def __init__(self, _bottom_spin_motor : WPI_TalonSRX, _top_spin_motor : rev.CANSparkMax) -> None:
      self.bottom_spin_motor = _bottom_spin_motor
      self.top_spin_motor = _top_spin_motor

   def spin_top(self, dir : int):
      if dir > 0:
         self.top_spin_motor.setVoltage(self.OUTAKE_POWER * 2.4)
      elif dir < 0:
         self.top_spin_motor.setVoltage(self.INTAKE_POWER * 2.4)
      else:
         self.top_spin_motor.setVoltage(0)

   def spin_bottom(self, dir : int):
      if dir < 0:
         self.bottom_spin_motor.setVoltage(self.OUTAKE_POWER+.5)
      else:
         self.bottom_spin_motor.setVoltage(self.INTAKE_POWER+.5)

   def stop_top(self):
      self.top_spin_motor.set(0)

   def stop_bottom(self):
      self.bottom_spin_motor.set(0)

   def stop(self):
      self.stop_top()
      self.stop_bottom()

   def getCameraInfo(self):
      networkTableData = NetworkTables.getTable("SmartDashboard")
      hasBall = networkTableData.getValue("ball_on_screen", False)
      x = networkTableData.getNumber("ball_x", 0) # -160 to 160 px
      y = networkTableData.getNumber("ball_y", 0) # -120 to 120 px
      angle = networkTableData.getNumber("angle", 0)
      distance = networkTableData.getNumber("distance", 0)
      data = [hasBall, x, y, angle, distance]
      return data

   def hasTarget(self):
      return self.getCameraInfo()[0]
