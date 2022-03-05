from rev import ColorSensorV3

class BallSensor:

   NOBALL = 0
   BLUE = 1
   RED = 2

   def __init__(self, _color_sensor : ColorSensorV3, _alliance_color : int) -> None:
       self.colorSensor = _color_sensor
       self.alliance_color = _alliance_color

   def printBallStatus(self):
      print("prox, red, blue: ", self.colorSensor.getProximity(), self.colorSensor.getRawColor().red, self.colorSensor.getRawColor().blue + 57)

   def getBallStatus(self):
      color = self.NOBALL
      if self.colorSensor.getProximity() > 230:
         if self.colorSensor.getRawColor().red < self.colorSensor.getRawColor().blue + 57:
            color = self.BLUE
         else:
            color = self.RED
         if color == self.alliance_color:
            return True
         else: 
            return False
      else:
         color = self.NOBALL
         return None