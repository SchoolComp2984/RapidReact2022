import wpilib, ctre, rev
from commands import shoot
from utils import ID, pid, math_functions, imutil, constants
from subsystems import intaker, rotary_joystick, drive, shooter
from networktables import NetworkTables
import math

sd = NetworkTables.getTable('SmartDashboard')

#py -3 robot.py deploy to deploy the code

class MyRobot(wpilib.TimedRobot):
   COUNTS_PER_RAD = 2048 / (2 * 3.14159)
   RADS_PER_COUNT = 2 * 3.14159 / 2048

   def robotInit(self):

      #SUBSYSTEM ENABLERS
      self.enable_intake = True
      self.enable_color_sensor = False
      self.enable_driving = True
      self.enable_shooter =  True
      self.enable_shooter_test = True

      self.pid = pid.PID()
      #Original PID constants: 0.4, 0.001, 2
      self.pid.set_pid(0.01, 0.000025, 0.05, 0)

      self.printTimer = wpilib.Timer()
      self.printTimer.start()

      #BALL COLOR PROCESSING
      self.alliance_color = constants.RED
      self.driver_station = wpilib.DriverStation
      self.alliance = self.driver_station.getAlliance()
      # To access what type of alliance it is: wpilib.DriverStation.Alliance.kBlue
      if wpilib.DriverStation.Alliance.kBlue == self.alliance:
         self.alliance_color = constants.BLUE
      else:
         self.alliance_color = constants.RED

      #BATTERY POWER LIMITING
      self.battery_voltage = self.driver_station.getBatteryVoltage()
      self.motor_power_multiplyer = 1

      #components: These are classes representing all the electrical sensors and actuators on the robot.
      self.frontLeft = ctre.WPI_TalonFX(ID.DRIVE_LEFT_FRONT)
      self.backLeft = ctre.WPI_TalonFX(ID.DRIVE_LEFT_BACK)

      self.frontRight = ctre.WPI_TalonFX(ID.DRIVE_RIGHT_FRONT)
      self.backRight = ctre.WPI_TalonFX(ID.DRIVE_RIGHT_BACK)
      
      self.shooterMotor = ctre.WPI_TalonFX(ID.SHOOTER)
      self.shooterServo = wpilib.Servo(ID.SHOOTER_SERVO)

      if (self.enable_shooter):
         self.shooter_control_mode = ctre.TalonFXControlMode(2) 
         #self.shooterMotor.configSelectedFeedbackSensor(ctre.TalonFXFeedbackDevice.IntegratedSensor, 0, 10) # IntegratedSensor
         self.shooterMotor.configNominalOutputForward(0.2,10)
         self.shooterMotor.configNominalOutputReverse(0.1,10)
         self.shooterMotor.configPeakOutputForward(0.99,10) # limit to 0.1x of the max power
         self.shooterMotor.configPeakOutputReverse(0.5,10) # limit to 0.1x of the max power
         limits = ctre.SupplyCurrentLimitConfiguration(True, 40, 40, 0)
         self.shooterMotor.configSupplyCurrentLimit(limits, 10)
         #self.shooterMotor.setNeutralMode(self.shooterMotor.NeutralMode.Coast)
         self.shooterMotor.selectProfileSlot(0,0)
         self.shooterMotor.config_kF(0, 0.3, 10)
         self.shooterMotor.config_kP(0, 0.2, 10)
         self.shooterMotor.config_kI(0, 0.0, 10)
         self.shooterMotor.config_kD(0, 0.0, 10)
         self.shooterMotor.configMotionAcceleration(15, 10)
         self.shooterMotor.configMotionCruiseVelocity(200000,10)
         #self.shooterMotor.configMotionAcceleration(20,10)

      self.intakeSpin = ctre.WPI_TalonSRX(ID.INTAKE_SPIN)
      self.intakeLift = ctre.WPI_TalonSRX(ID.INTAKE_LIFT)

      self.drive_imu = imutil.Imutil(self.intakeSpin)

      self.colorSensor = rev.ColorSensorV3(wpilib.I2C.Port(0))

      self.talon_motors = [ 
         self.intakeSpin,
         self.intakeLift,
         self.backLeft, 
         self.backRight, 
         self.frontLeft, 
         self.frontRight
      ]

      # Might change to XBOX controller depending on it working or not.
      while rotary_joystick.RotaryJoystick(ID.DRIVE_CONTROLLER).getRawButton(12) or not rotary_joystick.RotaryJoystick(ID.OPERATOR_CONTROLLER).getRawButton(12):
         ID.OPERATOR_CONTROLLER = 1- ID.OPERATOR_CONTROLLER 
         ID.DRIVE_CONTROLLER = 1- ID.DRIVE_CONTROLLER 
      self.rotary_controller = rotary_joystick.RotaryJoystick(ID.OPERATOR_CONTROLLER)
      self.operator_controller = wpilib.interfaces.GenericHID(ID.OPERATOR_CONTROLLER)
      self.drive_controller = wpilib.XboxController(ID.DRIVE_CONTROLLER)
      #self.HAND_LEFT = wpilib.interfaces.GenericHID.Hand.kLeftHand
      #self.HAND_RIGHT = wpilib.interfaces.GenericHID.Hand.kRightHand

      #subsystems: These combine multiple components into a coordinated system
      self._drive = drive.Drive(self.frontLeft, self.backLeft, self.frontRight, self.backRight, self.drive_imu, self.pid)
      self._shooter = shooter.Shooter(self.shooterMotor, self.shooterServo, self.colorSensor, self.alliance_color)
      self._intaker = intaker.Intaker(self.intakeLift, self.intakeSpin)

      #commands: These utilize subsystems to perform autonomous routines.
      self._shoot = shoot.Shoot(self._drive, self._shooter)

   def teleopInit(self):
      print("Starting")
      self.frontLeft.setInverted(True)
      self.backLeft.setInverted(True)
      self.frontRight.setInverted(False)
      self.backRight.setInverted(False)
      self.intakeSpin.setInverted(True)

      for motor in self.talon_motors:
         motor.setNeutralMode(ctre.NeutralMode.Brake)

      if self.enable_driving:
         self.rotary_controller.reset_angle(self._drive.getYaw())
      
   def teleopPeriodic(self):
      try:
         self.battery_voltage = self.driver_station.getBatteryVoltage()
         self.motor_power_multiplyer = math_functions.clamp(self.battery_voltage - 9.5, 0, 1)

         if self.printTimer.hasPeriodPassed(0.5):
            print("mult= ", self.motor_power_multiplyer)

         #SHOOTER
         vel = 0
         #self.shooterMotor.set(self.drive_controller.getRawAxis(1))
         if (self.operator_controller.getRawButton(1)):
            vel = 25000 # WAY TOO FAST
         elif (self.operator_controller.getRawButton(2)):
            vel = 20000 # maybe too fast
         elif (self.operator_controller.getRawButton(3)):
            vel = 15000 # speed for long distance
         elif (self.operator_controller.getRawButton(4)):
            vel = 10000 # speed for short distance
         elif (self.operator_controller.getRawButton(5)):
            vel = 5000 # too slow
         if (self.enable_shooter):
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
            #if self.printTimer.hasPeriodPassed(0.5):
               #print(pwr, " curr=",current_vel," set_vel=",vel)
               #print(self.shooterMotor.getSelectedSensorVelocity(0))
            
         #INTAKER
         if self.enable_intake:
            if self.operator_controller.getRawButton(6):
               self._intaker.spin()
            else:
               self._intaker.stop()
         
         #COLOR SENSOR TESTING
         #Nothing in front of sensor ~ 180, gets greater as object gets closer
         #if self.enable_color_sensor:
            #print(self._shooter.getBallStatus())

         # print ("IMU= ", self.drive_imu.getYaw())
         #DRIVING
         if (self.enable_driving):
            self._shoot.auto_execute(self.operator_controller.getRawButton(7), self.motor_power_multiplyer)
            if self.operator_controller.getRawButton(7):
               # also check if shooter has balls before aiming so we can stop the shooter from running when we finish shooting.
               
               self.rotary_controller.reset_angle(self._shoot.target_angle)
            else:
               angle = self.rotary_controller.rotary_inputs()
               speed_x = self.drive_controller.getRawAxis(0)
               speed_y = self.drive_controller.getRawAxis(1)
               self._drive.absoluteDrive(speed_y, speed_x, angle, self.motor_power_multiplyer)
               #self._drive.mecanumDrive(speed_y, -speed_x, angle)
         
         if self.enable_shooter_test:
            if self.operator_controller.getRawButton(8):
               self._shoot.transporting(ID.SERVO_MIN)
            if self.operator_controller.getRawButton(9):
               self._shoot.transporting(ID.SERVO_MAX)
      except:
         raise


if __name__ == "__main__":
   wpilib.run(MyRobot)