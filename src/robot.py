import wpilib, ctre, rev
from commands import shoot, intake
from utils import ID, pid, math_functions, imutil, constants
from subsystems import intaker, rotary_joystick, drive, shooter, ball_sensor
from networktables import NetworkTables
import math

sd = NetworkTables.getTable('SmartDashboard')

#py -3 robot.py deploy to deploy the code

class MyRobot(wpilib.TimedRobot):
   COUNTS_PER_RAD = 2048 / (2 * 3.14159)
   RADS_PER_COUNT = 2 * 3.14159 / 2048

   def robotInit(self):

      #SUBSYSTEM ENABLERS
      self.enable_intake_test = True
      self.enable_driving = True
      self.enable_shooter_test = False

      # automated drive is used when the limelight and IMU are both working...
      # ...if they are not working then we can default to a completely teleoperated drive
      self.automated_drive = True

      self.pid = pid.PID()
      #Original PID constants: 0.4, 0.001, 2
      self.pid.set_pid(0.01, 0.0002, 0.05, 0)

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
         self.frontRight,
         self.shooterMotor
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
      self._ball_sensor = ball_sensor.BallSensor(self.colorSensor, self.alliance_color)

      #commands: These utilize subsystems to perform autonomous routines.
      self._shoot = shoot.Shoot(self._drive, self._shooter)
      self._intake = intake.Intake(self._drive, self._intaker, self._ball_sensor)

      
   def autonomousInit(self) -> None:
       self.autoState = self.SHOOT

   def autonomousPeriodic(self) -> None:
       pass

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
         auto_shoot_button = self.operator_controller.getRawButton(7)
         manual_transport_button = self.operator_controller.getRawButton(8)
         manual_shoot_button = self.operator_controller.getRawButton(5)

         auto_intake_button = self.operator_controller.getRawButton(6)
         manual_intake_spin = self.operator_controller.getRawButton(10)
         manual_intake_spin_reverse = self.operator_controller.getRawButton(11)
         manual_intake_spin_toggle = self.operator_controller.getRawButton(9)
         manual_intake_raise = self.operator_controller.getRawButton(2)
         manual_intake_lower = self.operator_controller.getRawButton(3)



         self.battery_voltage = self.driver_station.getBatteryVoltage()
         self.motor_power_multiplyer = math_functions.clamp(self.battery_voltage - 9.5, 0, 1)

         #if self.printTimer.hasPeriodPassed(0.5):
         #   print("intake camera coords: ", self._intaker.getCameraInfo())

         #DRIVING AND COOL STATE MACHINES
         if (self.enable_driving):
            if self._shoot.execute(auto_shoot_button, manual_transport_button, manual_shoot_button, self.motor_power_multiplyer):
               # also check if shooter has balls before aiming so we can stop the shooter from running when we finish shooting.
               self.rotary_controller.reset_angle(self.drive_imu.getYaw())
            elif False:
               if (manual_intake_spin or manual_intake_spin_reverse or manual_intake_spin_toggle):
                  manual_intake = True
               self._intake.execute(self.motor_power_multiplyer)
               self.rotary_controller.reset_angle(self.drive_imu.getYaw())
            else:
               if (self.automated_drive):
                  self.automatedDrive()
               else:
                  self.manualDrive()

            self.manualShooter(manual_shoot_button, manual_transport_button)
            self.manualIntake(manual_intake_spin_toggle, manual_intake_spin, manual_intake_spin_reverse, manual_intake_raise, manual_intake_lower)


         #SHOOTER TEST
         if self.enable_shooter_test:
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
            #if self._shoot.flywheel_desiredSpeed != vel:
            #   self._shoot.pidshoot.integral = 0
            #self._shoot.flywheel_desiredSpeed = vel
            self._shoot.spin_pid(False)
            current_vel = self.shooterMotor.getSelectedSensorVelocity(0)
            #delta = vel-current_vel
            #pwr = delta * 0.001
            #if (pwr > 0.99):
            #   pwr = 0.99
            #if (pwr < 0):
            #   pwr = 0
            #if (vel == 0):
            #   pwr = 0
            #self.shooterMotor.set(ctre.TalonFXControlMode.Velocity, vel/10.0)
            #self.shooterMotor.set(ctre.TalonFXControlMode.PercentOutput, pwr)
            #self.shooterMotor.set(ctre.ControlMode.Velocity, vel * self.COUNTS_PER_RAD / 10, ctre.DemandType.ArbitraryFeedForward, 2)
            if self.printTimer.hasPeriodPassed(0.5):
               print(" curr=",current_vel," set_vel=",vel)
               #print(self.shooterMotor.getSelectedSensorVelocity(0))
      except:
         raise

   def automatedDrive(self):
      angle = self.rotary_controller.rotary_inputs()
      speed_x = math_functions.good_joystick_interp(self.drive_controller.getRawAxis(0), 0.1)
      speed_y = math_functions.good_joystick_interp(self.drive_controller.getRawAxis(1), 0.05)
      self._drive.absoluteDrive(speed_y, speed_x, angle, self.motor_power_multiplyer)

     
   
   def manualDrive(self):
      y = math_functions.good_joystick_interp(self.drive_controller.getRawAxis(1), 0.05)
      x = math_functions.good_joystick_interp(self.drive_controller.getRawAxis(0), 0.1)
      # twist = math_functions.good_joystick_interp(self.drive_controller.getRawAxis(0), 0.2)
      self._drive.arcadeDrive(y, x, self.motor_power_multiplyer)

   def manualIntake(self, toggle, spin, reverse, _raise, lower):
      if reverse:
         self._intaker.spin(-1)
      elif spin or toggle:
         self._intaker.spin(1)
      else:
         self._intaker.stop()
      # CONFIG LIMIT SWITCHES IN PHOENIX TUNER BEFORE USING THIS CODE ALSO CHECK IF NORMALLY CLOSED OR OPEN
      # if self.intakeSpin.isFwdLimitSwitchClosed() and self.intakeSpin.isRevLimitSwitchClosed():
      if _raise:
         self._intaker._raise()
      elif lower:
         self._intaker.lower()
      else:
         self._intaker.stop_lift()


   def manualShooter(self, shoot, transport):
      if shoot:
         self._shooter.setSpeed(0.6)
      # if transport:
      #    self._shooter.transportUp()
      # else:
      #    self._shooter.transportDown()


if __name__ == "__main__":
   wpilib.run(MyRobot)