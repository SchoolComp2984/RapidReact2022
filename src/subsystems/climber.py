from ctre import WPI_TalonSRX

class Climber:
   def __init__(self, _left_motor : WPI_TalonSRX, _right_motor : WPI_TalonSRX):
      self.left_motor = _left_motor
      self.right_motor = _right_motor

   def moveClimb(self, speed):
      # speed inbetween 0 and 1
      self.left_motor.set(speed)
      self.right_motor.set(speed)