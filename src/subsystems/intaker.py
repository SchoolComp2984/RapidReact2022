from ctre import WPI_TalonSRX

#best power is somewhere between 3.2-3.9 v and 3.1-3.6 a
class Intaker:

   INTAKE_POWER = 3.2
   
   def __init__(self, _lift_motor : WPI_TalonSRX, _spin_motor : WPI_TalonSRX) -> None:
      self.lift_motor = _lift_motor
      self.spin_motor = _spin_motor

   def spin(self, power = INTAKE_POWER):
      self.spin_motor.setVoltage(power)
      return self.spinPower
   
   def lower(self):
      pass

   def _raise(self):
      pass
