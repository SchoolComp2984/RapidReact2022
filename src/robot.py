import wpilib, ctre, rev
from commands import shoot
from utils import ID, pid, math_functions, imutil
from subsystems import rotary_joystick, drive, shooter, intake
from networktables import NetworkTables
import math

sd = NetworkTables.getTable('SmartDashboard')

#py -3 robot.py deploy to deploy the code

class MyRobot(wpilib.TimedRobot):
   COUNTS_PER_RAD = 2048 / (2 * 3.14159)
   RADS_PER_COUNT = 2 * 3.14159 / 2048

   def robotInit(self):

      
      self.pid = pid.PID()
      #Original PID constants: 0.4, 0.001, 3.2
      self.pid.set_pid(0.4, 0.001, 2, 0)

      self.printTimer = wpilib.Timer()
      self.printTimer.start()

      self.NOBALL = 0
      self.BLUE = 1
      self.RED = 2
      self.alliance_color = self.RED
      
      self.ball_color = self.NOBALL
      self.driver_station = wpilib.DriverStation
      self.alliance = self.driver_station.getAlliance()
      # To access what type of alliance it is: wpilib.DriverStation.Alliance.kBlue
      if wpilib.DriverStation.Alliance.kBlue == self.alliance:
         self.alliance_color = self.BLUE
      else:
         self.alliance_color = self.RED

      #components: These are classes representing all the electrical sensors and actuators on the robot.
      self.frontLeft = ctre.WPI_TalonFX(ID.DRIVE_LEFT_FRONT)
      self.backLeft = ctre.WPI_TalonFX(ID.DRIVE_LEFT_BACK)

      self.frontRight = ctre.WPI_TalonFX(ID.DRIVE_RIGHT_FRONT)
      self.backRight = ctre.WPI_TalonFX(ID.DRIVE_RIGHT_BACK)
      
      self.enable_driving = True
      self.shooterMotor = ctre.WPI_TalonFX(ID.SHOOTER)
      self.use_shooter =  False # enable/disable code for the shooter motor
      if (self.use_shooter):
         self.shooter_control_mode = ctre.TalonFXControlMode(2) 
         #self.shooterMotor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 10) # IntegratedSensor
         self.shooterMotor.configNominalOutputForward(0.2,10)
         self.shooterMotor.configNominalOutputReverse(0.1,10)
         self.shooterMotor.configPeakOutputForward(0.99,10) # limit to 0.1x of the max power
         self.shooterMotor.configPeakOutputReverse(0.5,10) # limit to 0.1x of the max power
         #self.shooterMotor.setNeutralMode(self.shooterMotor.NeutralMode.Coast)
         self.shooterMotor.selectProfileSlot(0,0)
         self.shooterMotor.config_kF(0, 0.3, 10)
         self.shooterMotor.config_kP(0, 0.2, 10)
         self.shooterMotor.config_kI(0, 0.0, 10)
         self.shooterMotor.config_kD(0, 0.0, 10)
         self.shooterMotor.configMotionAcceleration(15, 10)
         self.shooterMotor.configMotionCruiseVelocity(200000,10)
         #self.shooterMotor.configMotionAcceleration(20,10)

      self.intakeSpin = ctre.WPI_TalonSRX(ID.INTAKESPIN)
      self.intakeLower = ctre.WPI_TalonSRX(ID.INTAKELOWER)

      self.drive_imu = imutil.Imutil(self.intakeLower)

      self.color_sensor = rev.ColorSensorV3(wpilib.I2C.Port(0))
      self.testServo = wpilib.Servo(0)

      # Might change to XBOX controller depending on it working or not.
      self.rotary_controller = rotary_joystick.RotaryJoystick(ID.OPERATOR_CONTROLLER)
      self.rotary_buttons = wpilib.interfaces.GenericHID(ID.OPERATOR_CONTROLLER)
      self.operator_controller = wpilib.interfaces.GenericHID(ID.OPERATOR_CONTROLLER)
      self.drive_controller = wpilib.XboxController(ID.DRIVE_CONTROLLER)
      #self.HAND_LEFT = wpilib.interfaces.GenericHID.Hand.kLeftHand
      #self.HAND_RIGHT = wpilib.interfaces.GenericHID.Hand.kRightHand

      #subsystems: These combine multiple components into a coordinated system
      self._drive = drive.Drive(self.frontLeft, self.backLeft, self.frontRight, self.backRight, self.drive_imu, self.pid)
      self._shooter = shooter.Shooter(self.shooterMotor)

      #commands: These utilize subsystems to perform autonomous routines.
      self._shoot = shoot.Shoot(self._drive, self._shooter)

   def teleopInit(self):
      print("Starting")
      self.frontLeft.setInverted(True)
      self.backLeft.setInverted(True)
      self.frontRight.setInverted(False)
      self.backRight.setInverted(False)
      self.rotary_controller.reset_angle(self._drive.getYaw())
      
   def teleopPeriodic(self):
      try:
         vel = 0
         #self.shooterMotor.set(self.drive_controller.getRawAxis(1))
         
         if (self.operator_controller.getRawButton(1)):
            vel = 25000
         elif (self.operator_controller.getRawButton(2)):
            vel = 20000
         elif (self.operator_controller.getRawButton(3)):
            vel = 15000
         elif (self.operator_controller.getRawButton(4)):
            vel = 10000
         elif (self.operator_controller.getRawButton(5)):
            vel = 5000
         if (self.use_shooter):
            current_vel = self.shooterMotor.getSelectedSensorVelocity(0)
            delta = vel-current_vel
            pwr = delta * 0.001
            if (pwr > 0.99):
               pwr = 0.99
            if (pwr < 0):
               pwr = 0
            if (vel == 0):
               pwr = 0
            #self.shooterMotor.set(ctre.TalonFXControlMode.Velocity, vel/10.0)
            self.shooterMotor.set(ctre.TalonFXControlMode.PercentOutput, pwr)
            #self.shooterMotor.set(ctre.ControlMode.Velocity, vel * self.COUNTS_PER_RAD / 10, ctre.DemandType.ArbitraryFeedForward, 2)
            if self.printTimer.hasPeriodPassed(0.5):
               print(pwr, " curr=",current_vel," set_vel=",vel)
               #print(self.shooterMotor.getSelectedSensorVelocity(0))

         # self.testServo.setAngle(self.drive_controller.getLeftX() * 180 - 90)
         # #print(self.color_sensor.getProximity())
         # #Nothing in front of sensor ~ 180, gets greater as object gets closer
         # if self.color_sensor.getProximity() > 190:
         #    if self.color_sensor.getRawColor().red < self.color_sensor.getRawColor().blue:
         #       self.ball_color = self.BLUE
         #    else:
         #       self.ball_color = self.RED
         #    if self.ball_color == self.alliance_color:
         #       print("shoot")
         #    else: 
         #       print("discard")
         # else:
         #    self.ball_color = self.NOBALL
         #    print("no ball")

         #y = self.drive_controller.getRawAxis(3)
         #x = self.drive_controller.getRawAxis(0)
         #self._drive.arcadeDrive(y, x)

         if (self.enable_driving):
            if self.operator_controller.getRawButton(1):
               # also check if shooter has balls before aiming so we can stop the shooter from running when we finish shooting.
               self._shoot.execute()
               self.rotary_controller.reset_angle(self._shoot.target_angle)
            else:
               angle = self.rotary_controller.rotary_inputs() 
               speed_x = self.drive_controller.getRawAxis(2)
               speed_y = self.drive_controller.getRawAxis(3)
               speed = self.rotary_controller.getTwist()
               self._drive.absoluteDrive(speed, speed_x, angle)
               #self._drive.mecanumDrive(speed_y, -speed_x, angle)
      except:
         raise


if __name__ == "__main__":
   wpilib.run(MyRobot)