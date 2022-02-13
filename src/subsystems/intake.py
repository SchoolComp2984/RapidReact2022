from subsystems.drive import Drive

#best power is somewhere between 3.2-3.9 v and 3.1-3.6 a
class Intake:
   def spin(self):
      return self.spinPower
   
   def lower(self):
      return self.lowerPower
