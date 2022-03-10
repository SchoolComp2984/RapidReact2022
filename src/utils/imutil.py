from ctre import WPI_TalonSRX, PigeonIMU

class Imutil(PigeonIMU):
   # Wrapper for PigeonIMU

   def __init__(self, _parent_motor : WPI_TalonSRX):
       super().__init__(_parent_motor)

   def getRotation(self):
      # Get the yaw from the IMU
      return self.getYawPitchRoll()[1]

   def getYaw(self):
      return self.getRotation()[0]

   def isWorking(self):
      return True if self.getState() == PigeonIMU.PigeonState.Ready else False