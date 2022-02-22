from ctre import WPI_TalonFX
from wpilib import Servo
from rev import ColorSensorV3
from networktables import NetworkTables
from utils import constants

class Shooter:
   def __init__(self, _shooterMotor : WPI_TalonFX, _transportServo : Servo, _colorSensor : ColorSensorV3, _alliance_color : int):
      self.shooterMotor = _shooterMotor
      self.transportServo = _transportServo
      self.colorSensor = _colorSensor
      self.alliance_color = _alliance_color

   def setSpeed(self, speed):
      self.shooterMotor.set(speed)

   def getCameraInfo(self):
      networkTableData = NetworkTables.getTable("limelight")
      tv = networkTableData.getNumber("tv", None) # valid targets (1 or 0)
      tx = networkTableData.getNumber("tx", None) # horizontal offset (-27 to 27 degrees)
      ty = networkTableData.getNumber("ty", None) # vertical offset (-20.5 to 20.5 degrees)
      ta = networkTableData.getNumber("ta", None) # % of screen covered by target
      data = [tv, tx, ty, (ta * 100)]
      return data
   
   def printBallStatus(self):
      print("prox, red, blue: ", self.colorSensor.getProximity(), self.colorSensor.getRawColor().red, self.colorSensor.getRawColor().blue + 57)

   def getBallStatus(self):
      color = constants.NOBALL
      if self.colorSensor.getProximity() > 230:
         if self.colorSensor.getRawColor().red < self.colorSensor.getRawColor().blue + 57:
            color = constants.BLUE
         else:
            color = constants.RED
         if color == self.alliance_color:
            return True
         else: 
            return False
      else:
         color = constants.NOBALL
         return None
   
   def transportBall(self):
      self.transportServo.setAngle(180)
   
   def hasTarget(self):
      target = self.getCameraInfo()[0]
      if target == 1:
         return True
      else:
         return False



      


      
