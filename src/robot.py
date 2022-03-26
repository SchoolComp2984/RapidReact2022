import wpilib, ctre, rev
from commands import shoot, intake
from utils import ID, pid, math_functions, imutil, constants
from subsystems import intaker, rotary_joystick, drive, shooter, ball_sensor, climber
from networktables import NetworkTables
import math

sd = NetworkTables.getTable('SmartDashboard')

#py -3 robot.py deploy to deploy the code

class MyRobot(wpilib.TimedRobot):
   COUNTS_PER_RAD = 2048 / (2 * 3.14159)
   RADS_PER_COUNT = 2 * 3.14159 / 2048

   def robotInit(self):
      
      #SUBSYSTEM ENABLERS
      self.enable_driving = True
      self.enable_intake = True
      self.enable_shooter = True
      self.enable_climber = True
      # automated drive is used when the limelight and IMU are both working...
      # ...if they are not working then we can default to a completely teleoperated drive
      self.automated_drive = True

      self.pid = pid.PID()
      #Original PID constants: 0.4, 0.001, 2
      #PID constants I (charlie) changed on 3/7/22
      #  self.pid.set_pid(0.01, 0.0002, 0.05, 0)
      self.pid.set_pid(0.01, 0.0002, 0.05, 0)

      # self.printTimer = wpilib.Timer()
      # self.printTimer.start()

      #BALL COLOR PROCESSING
      self.alliance_color = constants.RED
      self.driver_station = wpilib.DriverStation

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
      self.intakeUpperSpin = rev.CANSparkMax(ID.INTAKE_UPPER, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
      self.drive_imu = imutil.Imutil(self.intakeSpin)

      self.colorSensor = rev.ColorSensorV3(wpilib.I2C.Port(0))

      self.climbLeftMotor = ctre.WPI_TalonSRX(ID.CLIMB_LEFT)
      self.climbRightMotor = ctre.WPI_TalonSRX(ID.CLIMB_RIGHT)
      # self.climbLeftMotor.configForwardLimitSwitchSource(ctre.LimitSwitchSource.FeedbackConnector, ctre.LimitSwitchNormal.NormallyClosed, ID.CLIMB_LEFT)
      # self.climbRightMotor.configForwardLimitSwitchSource(ctre.LimitSwitchSource.FeedbackConnector, ctre.LimitSwitchNormal.NormallyClosed, ID.CLIMB_RIGHT)


      self.talon_motors = [ 
         self.intakeSpin,
         self.backLeft, 
         self.backRight, 
         self.frontLeft, 
         self.frontRight,
         self.climbRightMotor,
         self.climbLeftMotor
      ]

      #subsystems: These combine multiple components into a coordinated system
      self._drive = drive.Drive(self.frontLeft, self.backLeft, self.frontRight, self.backRight, self.drive_imu, self.pid)
      self._shooter = shooter.Shooter(self.shooterMotor, self.shooterServo, self.colorSensor, self.alliance_color)
      self._intaker = intaker.Intaker(self.intakeSpin, self.intakeUpperSpin)
      self._climber = climber.Climber(self.climbLeftMotor, self.climbRightMotor)
      self._ball_sensor = ball_sensor.BallSensor(self.colorSensor, self.alliance_color)

      #commands: These utilize subsystems to perform autonomous routines.
      self._shoot = shoot.Shoot(self._drive, self._shooter)
      # self._intake = intake.Intake(self._drive, self._intaker, self._ball_sensor)

      self.frontLeft.setInverted(True)
      self.backLeft.setInverted(True)
      self.frontRight.setInverted(False)
      self.backRight.setInverted(False)
      self.intakeSpin.setInverted(True)
      self.climbRightMotor.setInverted(True)
      
      for motor in self.talon_motors:
         motor.setNeutralMode(ctre.NeutralMode.Brake)

      self.climbLeftMotor.setNeutralMode(ctre.NeutralMode.Brake)

      
   def autonomousInit(self) -> None:
      self.alliance = self.driver_station.getAlliance()

      if wpilib.DriverStation.Alliance.kBlue == self.alliance:
         self.alliance_color = constants.BLUE
         sd.putNumber("alliance_color", constants.BLUE)
         # print("color: blue")
      else:
         self.alliance_color = constants.RED
         sd.putNumber("alliance_color", constants.RED)
         # print("color: red")

      self._shooter.alliance_color = self.alliance_color

      self.NO_BALL = 0
      self.ONE_BALL = 1
      self.TWO_BALLS = 2
      self.THREE_BALLS = 3
      self.MOVE_BACK = 4
      # This var determines which type of autonomous routine you want to use at the start of match
      self.autoType = self.ONE_BALL

      self.backup_start_time = 0.0
      self.rev_start_time = 0.0
      self.trans_start_time = 0.0

      self.premove_start_time = 0.0
      self.move_start_time = 0.0

      self.start_time = 0.0
      # This var determines the state inside of each different routine
      self.IDLE = 0
      self.SHOOTING = 1
      self.PREMOVE = 2
      self.MOVING = 3
      self.WAIT = 4
      self.TURNING = 5
      self.TRANSPORTING = 6
      self.SECONDMOVING = 7
      self.SECONDINTAKE = 8
      self.THIRDMOVING = 11
      self.SECONDWAIT = 12
      self.SHOOTINGSECOND = 9
      self.FOURTHMOVING = 13
      self.DONE = 10
      self.state = self.IDLE 


   def autonomousPeriodic(self) -> None:
      try:
         if self.autoType == self.MOVE_BACK:
            if self.state == self.IDLE:
               self.start_time = wpilib.Timer.getFPGATimestamp()
               self.state = self.MOVING
            elif self.state == self.MOVING:
               self._drive.arcadeDrive(0.2, 0, 1)
               if self.start_time + 2.5 < wpilib.Timer.getFPGATimestamp():
                  self.state = self.DONE
            elif self.state == self.DONE:
               self._drive.stop()
            else:
               self.state = self.IDLE

         elif self.autoType == self.ONE_BALL:
            if self.state == self.IDLE:
               self.start_time = wpilib.Timer.getFPGATimestamp()
               self.state = self.SECONDMOVING
               self._shoot.finishedShoot = False

            elif self.state == self.SECONDMOVING:
               self._drive.arcadeDrive(0.25, 0, 1)
               if self.start_time + 0.5 < wpilib.Timer.getFPGATimestamp():
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.PREMOVE

            elif self.state == self.PREMOVE:
               self._drive.stop()
               if self.start_time + 1 < wpilib.Timer.getFPGATimestamp():
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.SHOOTING

            elif self.state == self.SHOOTING:
               self._shoot.execute(True, False, False, 1)
               if self._shoot.finishedShoot:
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.MOVING
                  self._shoot.finishedShoot = False

            elif self.state == self.MOVING:
               self._drive.arcadeDrive(0.2, 0, 1) #Drive backwards
               if self.start_time + 2.5 < wpilib.Timer.getFPGATimestamp():
                  self.state = self.DONE

            elif self.state == self.DONE:
               self._drive.stop()

            else:
               self.state = self.IDLE

         elif self.autoType == self.TWO_BALLS:

            if self.state == self.IDLE:
               self.startingAngle = self.drive_imu.getYaw()
               self._shoot.finishedShoot = False
               self.start_time = wpilib.Timer.getFPGATimestamp()
               self.state = self.MOVING

            elif self.state == self.MOVING:
               self._intaker.spin_bottom(1)
               self._drive.arcadeDrive(-0.3, 0, 1) #drive forwards
               if self.start_time + 0.5 < wpilib.Timer.getFPGATimestamp():
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.WAIT
            
            elif self.state == self.WAIT:
               self._drive.stop()
               if self.start_time + 0.6 < wpilib.Timer.getFPGATimestamp():
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.SECONDMOVING

            elif self.state == self.SECONDMOVING:
               self._intaker.spin_bottom(1)
               self._drive.arcadeDrive(-0.3, 0, 1) #drive forwards
               if self.start_time + 1.8 < wpilib.Timer.getFPGATimestamp():
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.SECONDWAIT

            elif self.state == self.SECONDWAIT:
               self._intaker.spin_bottom(1)
               self._drive.stop()
               self.startingAngle = self.drive_imu.getYaw()
               if self.start_time + 0.4 < wpilib.Timer.getFPGATimestamp():
                  self.state = self.TURNING

            elif self.state == self.TURNING:
               self._drive.absoluteDrive(0, 0, self.startingAngle + 180, True, 0.6)
               if self._shooter.hasTarget() and self._shoot.withinAngle(self.startingAngle + 180, 10):
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.FOURTHMOVING

            elif self.state == self.FOURTHMOVING:
               self._drive.arcadeDrive(-0.3, 0, 1)
               if self.start_time + 1.8 < wpilib.Timer.getFPGATimestamp():
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self.state = self.PREMOVE

            elif self.state == self.PREMOVE:
               self._drive.stop()
               self._shooter.setSpeed(0)
               if self.start_time + 0.2 < wpilib.Timer.getFPGATimestamp():
                  self.state = self.SHOOTING

            elif self.state == self.SHOOTING:
               self._shoot.execute(True, False, False, 1)
               if self._shoot.finishedShoot:
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self._shoot.finishedShoot = False
                  self.state = self.SECONDINTAKE

            elif self.state == self.SECONDINTAKE:
               self._intaker.spin_top(1)
               self._intaker.spin_bottom(1)
               self._shooter.transportDown()
               if self.start_time + 1.2 < wpilib.Timer.getFPGATimestamp():
                  self.state = self.SHOOTINGSECOND

            elif self.state == self.SHOOTINGSECOND:
               self._intaker.spin_top(1)
               self._intaker.spin_bottom(1)
               self._shoot.execute(True, False, False, 1)
               if self._shoot.finishedShoot:
                  self.start_time = wpilib.Timer.getFPGATimestamp()
                  self._shoot.finishedShoot = False 
                  self.state = self.THIRDMOVING

            elif self.state == self.THIRDMOVING:
               self._drive.arcadeDrive(0.5, 0, 1) #drive forwards
               if self.start_time + 2 < wpilib.Timer.getFPGATimestamp():
                  self.state = self.DONE

            elif self.state == self.DONE:
               self._intaker.stop()
               self._drive.stop()

            else:
               self.state = self.IDLE
               self._drive.stop()
      except:
         pass

   def autonomousExit(self) -> None:
      self._drive.stop()
      self._shooter.setSpeed(0)
      self._intaker.stop()

   def teleopInit(self):
      # print("teleop starting")

      self.alliance = self.driver_station.getAlliance()

      if wpilib.DriverStation.Alliance.kBlue == self.alliance:
         self.alliance_color = constants.BLUE
         sd.putNumber("alliance_color", constants.BLUE)
         # print("color: blue")
      else:
         self.alliance_color = constants.RED
         sd.putNumber("alliance_color", constants.RED)
         # print("color: red")

      self._shooter.alliance_color = self.alliance_color

      #Shorting rotary controller with the spinny thing 
      # This is assuming the drive controller has button 12 shorted.
      while not wpilib.Joystick(ID.ROTARY_CONTROLLER).getRawButton(12) or wpilib.Joystick(ID.OPERATOR_CONTROLLER).getRawButton(12):
         # Switch the numbers for each controller
         op_id = ID.OPERATOR_CONTROLLER
         ID.OPERATOR_CONTROLLER = ID.ROTARY_CONTROLLER 
         ID.ROTARY_CONTROLLER = op_id

      self.rotary_controller = rotary_joystick.RotaryJoystick(ID.ROTARY_CONTROLLER)
      self.rotary_buttons = wpilib.interfaces.GenericHID(ID.ROTARY_CONTROLLER)
      self.operator_controller = wpilib.interfaces.GenericHID(ID.OPERATOR_CONTROLLER)
      self.joystick_controller = wpilib.XboxController(ID.OPERATOR_CONTROLLER)

      # print("Operator controller id: ", str(ID.OPERATOR_CONTROLLER))
      # print("Rotary controller id: ", str(ID.ROTARY_CONTROLLER))

      if self.enable_driving:
         self.rotary_controller.reset_angle(self._drive.getYaw())
      
   def teleopPeriodic(self):
      try:
      
         # TRANSFER THESE TO ID.PY WHEN WE GET THE FINAL BUTTON ID'S
         auto_shoot_button = self.operator_controller.getRawButton(3)
         manual_transport_button = self.operator_controller.getRawButton(2)
         manual_shoot_button = self.operator_controller.getRawButton(1)
         # THIS BUTTON MIGHT BECOME A TOGGLE SWITCH BUT SHOULD STILL WORK THE SAME

         auto_intake_button = self.operator_controller.getRawButton(4)
         manual_intake_spin_bottom = self.operator_controller.getRawButton(5)
         manual_intake_spin_top = self.operator_controller.getRawButton(11)
         manual_intake_spin_both = self.operator_controller.getRawButton(7)
         manual_intake_spin_reverse = self.operator_controller.getRawButton(6)
         manual_intake_spin_toggle =  self.operator_controller.getRawButton(8)

         manual_climb_up = self.operator_controller.getRawButton(10)
         manual_climb_down =  self.operator_controller.getRawButton(9)

         self.battery_voltage = self.driver_station.getBatteryVoltage()
         self.motor_power_multiplyer = math_functions.clamp(self.battery_voltage - 9.5, 0, 1)

         # CAMERA DEBUGGING LINES
         # if self.printTimer.hasPeriodPassed(0.5):
            # print("intake camera data: ", self._intaker.getCameraInfo())
            # print("limelight x, limelight y: ", self._shooter.getCameraInfo()[1], self._shooter.getCameraInfo()[2])

         self.manual_drive = self.rotary_controller.getRawButton(11)
         
         #DRIVING AND COOL STATE MACHINES
         if (self.enable_driving):
            y = math_functions.good_joystick_interp(self.joystick_controller.getRawAxis(1), 0.05, 2)
            #if False:
            if self._shoot.execute(auto_shoot_button, manual_transport_button, manual_shoot_button, self.motor_power_multiplyer):
               self.rotary_controller.reset_angle(self.drive_imu.getYaw())

            # elif False:
            # # elif self._intake.execute(auto_intake_button, y, self.motor_power_multiplyer):
            #    self.rotary_controller.reset_angle(self.drive_imu.getYaw())
            else:
               if (self.manual_drive):
                  self.manualDrive()
               else:
                  self.automatedDrive()

               if self.enable_shooter:
                  self.manualShooter(manual_shoot_button, manual_transport_button)
               if self.enable_intake:
                  self.manualIntake(manual_intake_spin_toggle, manual_intake_spin_bottom, manual_intake_spin_top, manual_intake_spin_reverse, manual_intake_spin_both)
               if self.enable_climber:
                  self.manualClimber(manual_climb_up, manual_climb_down)

      except:
         pass

   def automatedDrive(self):
      angle = self.rotary_controller.rotary_inputs()
      #deadzone to stop robot driving off when rotary controller isnt spinning
      speed_x = math_functions.good_joystick_interp(self.joystick_controller.getRawAxis(0), 0.15, 3)
      speed_y = math_functions.good_joystick_interp(self.joystick_controller.getRawAxis(1), 0.05, 2)
      self._drive.absoluteDrive(speed_y, speed_x, angle, True, self.motor_power_multiplyer)

   def manualDrive(self):
      y = math_functions.good_joystick_interp(self.joystick_controller.getRawAxis(1), 0.05, 2)
      x = math_functions.good_joystick_interp(self.joystick_controller.getRawAxis(0), 0.15, 3)
      # twist = math_functions.good_joystick_interp(self.joystick_controller.getRawAxis(0), 0.2)
      self._drive.arcadeDrive(y, x, self.motor_power_multiplyer)
      self.rotary_controller.reset_angle(self._drive.getYaw())

   def manualIntake(self, toggle, bottom, top, reverse, both):
      if bottom:
         self._intaker.spin_top(0)
         if reverse:
            self._intaker.spin_bottom(-1)
         else:
            self._intaker.spin_bottom(1)
      elif toggle or both: 
         if reverse:
            self._intaker.spin_bottom(-1)
            self._intaker.spin_top(-1)
         else:
            self._intaker.spin_bottom(1)
            self._intaker.spin_top(1)
      else:
         self._intaker.stop()

   def manualShooter(self, shoot, transport):
      # if shoot:
      #    self._shooter.setSpeed(0.5)
      # else:
      #    self._shooter.setSpeed(0)
      if transport:
         self._shooter.transportUp()
      else:
         self._shooter.transportDown()

   def manualClimber(self, up, down):
      if up:
         self._climber.moveClimb(.5)
      elif down:
         self._climber.moveClimb(-.7)
      else:
         self._climber.stop()

         


   def disabledInit(self) -> None:
      sd.delete("alliance_color")

if __name__ == "__main__":
   wpilib.run(MyRobot)
