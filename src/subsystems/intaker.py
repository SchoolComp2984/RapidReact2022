from ctre import WPI_TalonSRX
from networktables import NetworkTables

#best power is somewhere between 3.2-3.9 v and 3.1-3.6 a
class Intaker:

   INTAKE_POWER = 3.4
   
   def __init__(self, _lift_motor : WPI_TalonSRX, _spin_motor : WPI_TalonSRX) -> None:
      self.lift_motor = _lift_motor
      self.spin_motor = _spin_motor

   def spin(self):
      self.spin_motor.setVoltage(self.INTAKE_POWER)
   
   def stop(self):
      self.spin_motor.set(0)

   def lower(self):
      pass

   def _raise(self):
      pass

   def getCameraInfo(self):
      networkTableData = NetworkTables.getTable("SmartDashboard")
      x = networkTableData.getNumber("intake-ball-offset-x", 0)
      y = networkTableData.getNumber("intake-ball-offset-y", 0)
      data = [x, y]
      return data
