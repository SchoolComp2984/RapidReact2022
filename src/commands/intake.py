import math, ctre, wpilib
from subsystems.drive import Drive
from subsystems.intaker import Intaker

class Intake:

   def __init__(self, _drive : Drive, _intaker : Intaker):
      self.drive = _drive
      self.intaker = _intaker

      