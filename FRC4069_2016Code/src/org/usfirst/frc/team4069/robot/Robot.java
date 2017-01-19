package org.usfirst.frc.team4069.robot;


import edu.wpi.first.wpilibj.Encoder;

import java.io.BufferedWriter;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileWriter;
import java.io.IOException;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
//import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;


public class  Robot extends SampleRobot 
{
    
    // Robot camera for driver station
    //private CameraServer cameraServer;
    
    // Left and right drive motor talons control the drive motors on either side of the robot
    public Talon leftDriveMotorTalon;
    public Talon rightDriveMotorTalon;
    
    // Circumference of the drive wheels in inches
    //public static final int DRIVE_WHEEL_CIRCUMFERENCE = 100;
    
    public PowerDistributionPanel powerpanel;
    
    // Left and right shifters control the speed of the robot
    private DoubleSolenoid leftShifter;
    private DoubleSolenoid rightShifter;
    // The metal panel which pushes the ball upward for shooting
    private DoubleSolenoid kicker;
    // The pneumatic compressor attached to the roboRIO
    private Compressor compressor;
    // Tells whether the pneumatic kicker is forward or back
    private boolean kickerForward = false;
    
    // Limit switch which, when activated, starts the compressor
    private DigitalInput compressorLimit;
    // Tells if the compressor is currently started
    private boolean compressorEnabled=false;
    
    // Control sensitivity of drive motors for driving and steering
    private LowPassFilter leftDriveMotorLowPassFilter;
    private LowPassFilter rightDriveMotorLowPassFilter;
    
    private LowPassFilter autoLowPassFilter;
    
    //RobotDrive is a class given to all teams which allows easy arcade drive functionality.
    //Arcade drive is where a single joystick controls forward/backward and left/right all at once.
    //vs TankDrive where one joystick controls the left wheels and one joystick controls the right wheels.
    public RobotDrive mrobotDrive;
    private Encoder leftDriveEncoder;
    private Encoder rightDriveEncoder;
    
    // TalonSRX motors for left and right sides of the arm, must be kept in sync or robot will be violently ripped apart
    private CANTalon leftArm;
    private CANTalon rightArm;
    
    // Limit switch which deactivates the arm when hit
    private DigitalInput armLimit;
    // Tells whether the arm limit switch has been hit
    private boolean armLimitHit = false;
    // This is either set to true or false upon startup, depending on whether the arm is in the right position
    private boolean armShouldMove = true;
    // Last position of arm when it was activated, used for keeping the arm from falling under its own weight
    private double armPosBeforeInactive = 0;
    // Is the arm activated by any inputs
    private boolean armMoving = false;
    
    // Speed of the left and right arm motors, should always be the same
    private double leftArmSpeed = 0.0;
    private double rightArmSpeed = 0.0;
    
    // Controls the shooter wheel
    private Talon shooter;
    // Shooter wheel speed
    private double shooterSpeed = 0.0;
    // Encoder attached to the shooter wheel
    private Encoder shooterEncoder;
    
    // Is the robot currently automatically shooting
    private boolean shooting=false;
    private boolean shootingWithDrive=false;
    private boolean shootingWithDriveFurther=false;
    // Time passed since shooting was true
    private int shootingTime = 0;
    // Greater shootingSpeedMultiplier means the shooting process takes longer
    public static final double shootingSpeedMultiplier = 2;
    
    // Is the robot lifting the guard rail
    private boolean liftingGuardRail=false;
    // Time passed since liftingGuardRail was true
    private int liftingGuardTime = 0;
    
    // Intake for intake roller, rollerWheels for polycord wheels
    private Talon intake;
    private Talon rollerWheels;
    // Speed of both intake and polycord wheels (they should be in sync)
    private double intakeSpeed = 0.0;
    // Used for detecting balls and stopping intake before they reach the shooter
    private DigitalInput intakeSensor;
    private boolean intakeSensorHit=false;
    
    private DigitalInput readyToShootSensor;
    private boolean readyToShootSensorHit=false;
    
    // Tells whether pneumatic shift is enabled
    private boolean shiftEnabled = false;
    
    Joystick driverStick;  // left stick is the 'Driver' stick
    Joystick controlStick; // right stick is all the controls
    
    double driverRobotSpeed=0;   //velocity of robot -1 full reverse, +1 = full forward
    double driverRobotTurnDirection=0;    //x axis of drivers left joystick
    
    public NetworkTable myTable=null;
    
    // Control stick buttons
    public boolean Y_Button_Control_Stick=false; // Activates the shooter
    public boolean A_Button_Control_Stick=false;
    public boolean X_Button_Control_Stick=false;
    public boolean X_Button_Control_Stick_Prev=false;
    public boolean B_Button_Control_Stick=false;
    public boolean RB_Button_Control_Stick=false; // Toggles the kicker
    public boolean RB_Button_Control_Stick_Prev=false;
    // Driver stick buttons
    public boolean Y_Button_Driver_Stick=false;
    public boolean A_Button_Driver_Stick=false;
    public boolean X_Button_Driver_Stick=false; // When pressed, makes arm stay in place and not weigh itself down
    public boolean B_Button_Driver_Stick=false;
    public boolean B_Button_Driver_Stick_Prev=false;
    public boolean RB_Button_Driver_Stick=false; // Toggles pneumatic shift
    public boolean RB_Button_Driver_Stick_Prev=false;
    
    /**
     * Initialize all the variables.
     */
    public Robot() 
    {     
      myTable = NetworkTable.getTable("MyTables");
      
      /*cameraServer = CameraServer.getInstance();
      cameraServer.setQuality(50);
      cameraServer.startAutomaticCapture("cam0");*/
      
      autoLowPassFilter = new LowPassFilter(150);
      
      leftDriveMotorLowPassFilter = new LowPassFilter(350);
      rightDriveMotorLowPassFilter = new LowPassFilter(250);
      powerpanel = new PowerDistributionPanel();
      
      leftShifter = new DoubleSolenoid(2, 5);
      rightShifter = new DoubleSolenoid(1, 6);
      /*leftShifter.set(Value.kForward);
      rightShifter.set(Value.kForward);
      shiftEnabled = true;*/
      kicker = new DoubleSolenoid(0, 7);
      
      leftArm = new CANTalon(IOMapping.LEFT_ARM_CANBUS_PORT);
      rightArm = new CANTalon(IOMapping.RIGHT_ARM_CANBUS_PORT);
      // Zero the arm encoders
      leftArm.setPosition(0);
      rightArm.setPosition(0);
      armPosBeforeInactive = 0;
      // Brake the arm motors
      //leftArm.enableBrakeMode(true);
      //rightArm.enableBrakeMode(true);
      armPosBeforeInactive = leftArm.getPosition();
      
      shooter = new Talon(IOMapping.SHOOTER_PWM_PORT);
      shooterEncoder = new Encoder(IOMapping.SHOOTER_ENCODER_DIO_1, IOMapping.SHOOTER_ENCODER_DIO_2);
      rollerWheels = new Talon(IOMapping.ROLLERWHEELS_PWM_PORT);
      intake = new Talon(IOMapping.INTAKE_PWM_PORT);
      intakeSensor = new DigitalInput(IOMapping.INTAKE_SENSOR_DIO);
      
      readyToShootSensor = new DigitalInput(IOMapping.READY_TO_SHOOT_DIO);
      
      armLimit = new DigitalInput(IOMapping.ARM_DIO);
      
      compressor = new Compressor(IOMapping.COMPRESSOR_PORT);
      compressor.stop();
      
      compressorLimit = new DigitalInput(IOMapping.COMPRESSOR_DIO);
      
      leftDriveMotorTalon = new Talon(IOMapping.LEFT_DRIVE_MOTOR_PWM_PORT);
      rightDriveMotorTalon = new Talon(IOMapping.RIGHT_DRIVE_MOTOR_PWM_PORT);
      driverStick = new Joystick(IOMapping.DRIVE_JOYSTICK_PORT);
      controlStick = new Joystick(IOMapping.CONTROL_JOYSTICK_PORT);
      
      leftDriveEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
      rightDriveEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);
      
      mrobotDrive = new RobotDrive(leftDriveMotorTalon,rightDriveMotorTalon);
      mrobotDrive.setInvertedMotor(MotorType.kRearLeft, true);
      mrobotDrive.setInvertedMotor(MotorType.kRearRight, true);
      mrobotDrive.setExpiration(0.1);
    }
    
    /*
     * -------------------------Tele-operated code-----------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------ 
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     */
    
    /**
     * Teleoperated period.
     */
    public void operatorControl() 
    {
      mrobotDrive.setSafetyEnabled(true);
      compressor.start();
      armShouldMove = !armLimit.get();
      while (isOperatorControl() && isEnabled()) 
      {
        // Read and store all input from buttons and such
        ReadAllInput();
        
        // Now update each component of the robot systematically
        UpdateDriveMotors();
        UpdateArm();
        UpdateShooter();
        UpdateIntake();
        UpdatePneumatics();
        UpdateAutomaticShooting();
        UpdateAutomaticShootingWithDriving();
        //UpdateAutomaticShootingWithDrivingFurther();
        //UpdateAutomaticGuardLift();
        
        // Let the robotdrive class handle the driving aspect of the robot
        mrobotDrive.arcadeDrive(driverRobotSpeed,driverRobotTurnDirection);  //move robot
        
        // Apply all of the calculated speeds of the motors to the motors
        leftArm.set(leftArmSpeed);
        rightArm.set(rightArmSpeed);
        shooter.set(shooterSpeed);
        rollerWheels.set(intakeSpeed);
        intake.set(-intakeSpeed);
        
        SendDataToSmartDashboard();
        Timer.delay(0.005);    // wait for a motor update time
      }
    }
    
    void ReadAllInput()
    {
      Y_Button_Control_Stick = controlStick.getRawButton(IOMapping.CONTROL_Y_BUTTON);
      A_Button_Control_Stick = controlStick.getRawButton(IOMapping.CONTROL_A_BUTTON);
      X_Button_Control_Stick_Prev = X_Button_Control_Stick;
      X_Button_Control_Stick = controlStick.getRawButton(IOMapping.CONTROL_X_BUTTON);
      B_Button_Control_Stick = controlStick.getRawButton(IOMapping.CONTROL_B_BUTTON);
      RB_Button_Control_Stick_Prev = RB_Button_Control_Stick;
      RB_Button_Control_Stick = controlStick.getRawButton(IOMapping.CONTROL_RB_BUTTON);
      
      Y_Button_Driver_Stick = driverStick.getRawButton(IOMapping.DRIVER_Y_BUTTON);
      A_Button_Driver_Stick = driverStick.getRawButton(IOMapping.DRIVER_A_BUTTON);
      X_Button_Driver_Stick = driverStick.getRawButton(IOMapping.DRIVER_X_BUTTON);
      B_Button_Driver_Stick_Prev = B_Button_Driver_Stick;
      B_Button_Driver_Stick = driverStick.getRawButton(IOMapping.DRIVER_B_BUTTON);
      RB_Button_Driver_Stick_Prev = RB_Button_Driver_Stick;
      RB_Button_Driver_Stick = driverStick.getRawButton(IOMapping.DRIVER_RB_BUTTON);
      
      armLimitHit = !armLimit.get();
      intakeSensorHit = intakeSensor.get();
      readyToShootSensorHit = readyToShootSensor.get();
    }
    
    //--------------------------------------------------------------------------------------------
    
    /**
     * Update the drive motor speed and turn direction variables.
     * @return
     */
    boolean UpdateDriveMotors()
    {
      // Combined speed of the drive motors
      double combined = driverStick.getRawAxis(3)*-1+driverStick.getAxis(AxisType.kZ);
      driverRobotSpeed = leftDriveMotorLowPassFilter.calculate(combined);
      // Calculate robot turn direction from the left drive joystick's x-axis (left-right)
      driverRobotTurnDirection = rightDriveMotorLowPassFilter.calculate(driverStick.getAxis(AxisType.kX));
      return true;
    }
    
    /**
     * Set the speed of the arm, taking into account the upper and lower limits.
     */
    private void setArmSpeed(double armSpeed){
      if(armShouldMove){
        if(!armLimitHit && armSpeed >= 0){
          // If arm limit isn't hit and the arm is to go backward, move the arm
          leftArmSpeed = armSpeed;
          rightArmSpeed = armSpeed;
        }
        else if(-leftArm.getPosition() <= 3500 && armSpeed < 0){
          // If the arm's position is less than the maximum and the arm is to go forward, move the arm
          leftArmSpeed = armSpeed;
          rightArmSpeed = armSpeed;
        }
        else{
          leftArmSpeed = 0.0;
          rightArmSpeed = 0.0;
        }
      }
      else{
        leftArmSpeed = 0.0;
        rightArmSpeed = 0.0;
      }
    }
    
    /**
     * Update leftArmSpeed and rightArmSpeed.
     */
    void UpdateArm(){
      double armSpeed = threshold(driverStick.getRawAxis(5) * 0.3, 0.05);
      SmartDashboard.putNumber("ARMSPEED=", armSpeed);
      if(armShouldMove){
        //setArmSpeed(armSpeed);
        if(!X_Button_Driver_Stick){
          armPosBeforeInactive = -leftArm.getPosition();
        }
        // The following code holds the arm in place and allows it to be locked into a certain position
        if(Math.abs(armSpeed) > 0){
          armMoving = true;
          setArmSpeed(armSpeed);
          armPosBeforeInactive = -leftArm.getPosition();
        }
        else{
          armMoving = false;
          if(X_Button_Driver_Stick){
            armPosBeforeInactive = 2000;
            double armPosition = -leftArm.getPosition();
            if(armPosBeforeInactive != armPosition){
              double counterForce = (armPosition - armPosBeforeInactive) / 1000.0;
              counterForce = limit(counterForce, 0.3);
              setArmSpeed(counterForce);
            }
          }
          else{
            setArmSpeed(0);
          }
        }
      }
      else{
        // This code is called when the arm was not in the right position
        // at the beginning of teleoperated mode, which occurs under some
        // autonomous modes. It moves the arm upright when the joystick
        // is activated.
        if(Math.abs(armSpeed) > 0){
          if(!armShouldMove){
            leftArmSpeed = rightArmSpeed = 0.3;
          }
        }
        if(armLimitHit){
          armShouldMove = true;
          leftArm.setPosition(0);
          rightArm.setPosition(0);
        }
      }
      if(armLimitHit){
        leftArm.setPosition(0);
        rightArm.setPosition(0);
      }
    }
    
    /**
     * Update the shooter speed variable.
     */
    void UpdateShooter(){
      SmartDashboard.putNumber("SHOOTERENCODER", shooterEncoder.get());
      // If y button is pressed enable shooter at full speed, otherwise disable it
      if(Y_Button_Control_Stick){
        shooterSpeed = -1.0;
      }
      else{
        shooterSpeed = 0.0;
      }
    }
    
    /**
     * Update the automatic shooting functionality which affects the shooter as well as the intake and the kicker.
     */
    void UpdateAutomaticShooting(){
      // If a button is pressed, start shooting the ball
      if(A_Button_Control_Stick){
        if(!shootingWithDrive && !shooting && !shootingWithDriveFurther){
          shooting = true;
        }
      }
      // If the ball is being shot, continue shooting
      if(shooting){
        if(shootingTime >= (int)(120 * shootingSpeedMultiplier)){
          shooting = false;
          shootingTime = 0;
        }
        else{
          if(shootingTime == (int)(60 * shootingSpeedMultiplier)){
            kicker.set(Value.kReverse);
          }
          if(shootingTime <= (int)(120 * shootingSpeedMultiplier)){
            shooterSpeed = -1.0;
          }
          if(shootingTime <= (int)(120 * shootingSpeedMultiplier) && shootingTime >= (int)(15 * shootingSpeedMultiplier)){
            intakeSpeed = 1.0;
          }
          if(shootingTime == (int)(115 * shootingSpeedMultiplier)){
            kicker.set(Value.kForward);
          }
          shootingTime++;
        }
      }
    }
    
    /**
     * Same thing as other method except drives further before shooting.
     */
    void UpdateAutomaticShootingWithDrivingFurther(){
      // Actual start time of the shooting before moving backwards
      final int shootingStartTime = 150;
      // If a button is pressed, start shooting the ball
      if(X_Button_Control_Stick){
        if(!shooting && !shootingWithDrive && !shootingWithDriveFurther){
          shootingWithDriveFurther = true;
        }
      }
      // If the ball is being shot, continue shooting
      if(shootingWithDriveFurther){
        if(shootingTime >= (int)((120 + shootingStartTime) * shootingSpeedMultiplier)){
          shootingWithDriveFurther = false;
          shootingTime = 0;
        }
        else{
          if(shootingTime < (int)(shootingStartTime * shootingSpeedMultiplier)){
            driverRobotSpeed = -0.5;
          }
          if(shootingTime == (int)((40 + shootingStartTime) * shootingSpeedMultiplier)){
            kicker.set(Value.kReverse);
          }
          if(shootingTime <= (int)((120 + shootingStartTime) * shootingSpeedMultiplier)){
            shooterSpeed = -1.0;
          }
          if(shootingTime <= (int)((120 + shootingStartTime) * shootingSpeedMultiplier) && shootingTime >= (int)((15 + shootingStartTime) * shootingSpeedMultiplier)){
            intakeSpeed = 1.0;
          }
          if(shootingTime == (int)((95 + shootingStartTime) * shootingSpeedMultiplier)){
            kicker.set(Value.kForward);
          }
          shootingTime++;
        }
      }
    }
    
    /**
     * Update the automatic shooting functionality while driving backwards for shooting in the high goal from the ramp.
     */
    void UpdateAutomaticShootingWithDriving(){
      // Actual start time of the shooting before moving backwards
      final int shootingStartTime = 75;
      // If a button is pressed, start shooting the ball
      if(Y_Button_Control_Stick){
        if(!shooting && !shootingWithDrive && !shootingWithDriveFurther){
          shootingWithDrive = true;
        }
      }
      // If the ball is being shot, continue shooting
      if(shootingWithDrive){
        if(shootingTime >= (int)((120 + shootingStartTime) * shootingSpeedMultiplier)){
          shootingWithDrive = false;
          shootingTime = 0;
        }
        else{
          if(shootingTime < (int)(shootingStartTime * shootingSpeedMultiplier)){
            driverRobotSpeed = -0.5;
          }
          if(shootingTime == (int)((40 + shootingStartTime) * shootingSpeedMultiplier)){
            kicker.set(Value.kReverse);
          }
          if(shootingTime <= (int)((120 + shootingStartTime) * shootingSpeedMultiplier)){
            shooterSpeed = -1.0;
          }
          if(shootingTime <= (int)((120 + shootingStartTime) * shootingSpeedMultiplier) && shootingTime >= (int)((15 + shootingStartTime) * shootingSpeedMultiplier)){
            intakeSpeed = 1.0;
          }
          if(shootingTime == (int)((95 + shootingStartTime) * shootingSpeedMultiplier)){
            kicker.set(Value.kForward);
          }
          shootingTime++;
        }
      }
    }
    
    /**
     * Updates the automatic guard rail lifting functionality which affects the arm as well as the drive motor.
     */
    void UpdateAutomaticGuardLift(){
      // If b button is pressed, start activating the automatic guard lift
      if(B_Button_Control_Stick){
        liftingGuardRail = true;
      }
      // If the guard rail is being lifted, continue lifting
      if(liftingGuardRail){
        if(liftingGuardTime >= 100){
          liftingGuardRail = false;
          liftingGuardTime = 0;
        }
        else{
          driverRobotSpeed = 0.5;
          setArmSpeed(-0.2);
        }
        liftingGuardTime++;
      }
    }
    
    /**
     * Update the intake speed variable based on the control stick's triggers.
     */
    void UpdateIntake(){
      SmartDashboard.putNumber("INTAKESPEED=", controlStick.getRawAxis(3));
      intakeSpeed = controlStick.getRawAxis(3) - controlStick.getRawAxis(2);
      if(!readyToShootSensorHit && intakeSpeed > 0){
        intakeSpeed = 0.0;
      }
    }
    
    /**
     * Toggles the pneumatic kicker.
     */
    private void toggleKicker(){
      kickerForward = !kickerForward;
      kicker.set(kickerForward ? Value.kForward : Value.kReverse);
    }
    
    /**
     * Update all things pneumatics-related on the robot.
     */
    void UpdatePneumatics(){
      if(RB_Button_Control_Stick){
        // Toggle the kicker position
        if(!RB_Button_Control_Stick_Prev){
          toggleKicker();
        }
      }
      if(RB_Button_Driver_Stick){
        // Toggle shift
        if(!RB_Button_Driver_Stick_Prev){
          shiftEnabled = !shiftEnabled;
        }
        leftShifter.set(shiftEnabled ? Value.kForward : Value.kReverse);
        rightShifter.set(shiftEnabled ? Value.kForward : Value.kReverse);
      }
      // Make sure shift is at the right value
      if(shiftEnabled && !(leftShifter.get() == Value.kForward || rightShifter.get() == Value.kForward)){
        leftShifter.set(Value.kForward);
        rightShifter.set(Value.kForward);
      }
      if(!shiftEnabled && (leftShifter.get() == Value.kForward || rightShifter.get() == Value.kForward)){
        leftShifter.set(Value.kReverse);
        rightShifter.set(Value.kReverse);
      }
    }
    
    void SendDataToSmartDashboard()
    {
      SmartDashboard.putNumber("LEFTARM", -leftArm.getPosition());
      SmartDashboard.putNumber("RIGHTARM", rightArm.getPosition());
      SmartDashboard.putBoolean("ARMSHOULDMOVE", armShouldMove);
      SmartDashboard.putBoolean("ARMLIMITHIT", armLimitHit);
      SmartDashboard.putNumber("LEFTDRIVE", leftDriveEncoder.get());
      SmartDashboard.putNumber("RIGHTDRIVE", leftDriveEncoder.get());
    }
    
    /*
     * -------------------------Autonomous code--------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------ 
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     */
    
    // Negative to move arm backwards, positive to move arm forwards
    private int autoDesiredTicksMovingArm = 0;
    
    // Negative to drive backwards, positive to drive forwards
    private int autoDesiredTicksDriving = 0;
    
    // Arm speed in autonomous mode
    private double autoArmSpeed = 0;
    
    // Drive speed in autonomous mode
    private double autoDriveSpeed = 0;
    
    // Number of ticks in autonomous mode, incremented each iteration
    private int autoTickCount;
    
    private long prevTime;
    private long currentTime;
    
    public static final double autoSpeedMultiplier = 2.2;
    
    public static final int AUTO_MODE = 1;
    
    private void moveForwardNoEncoders(int ticks, double speed){
      autoDesiredTicksDriving += ticks;
      autoDriveSpeed = -speed;
    }
    
    private void moveArmNoEncoders(int ticks, double speed){
      autoDesiredTicksMovingArm += ticks;
      autoArmSpeed = -speed;
    }
    
    //Called once each time the robot enters the autonomous state.
    public void autonomous()
    {
      mrobotDrive.setSafetyEnabled(false);
      
      autoTickCount = 0;
      autoArmSpeed = 0;
      autoDriveSpeed = 0;
      autoDesiredTicksMovingArm = 0;
      autoDesiredTicksDriving = 0;
      
      if(AUTO_MODE != 0){
        if(AUTO_MODE == 1){
          moveForwardNoEncoders((int)(400 * autoSpeedMultiplier), 1.0);
        }
        if(AUTO_MODE == 2){
          moveArmNoEncoders((int)(200 * autoSpeedMultiplier), 0.3);
        }
        if(AUTO_MODE == 3){
          moveArmNoEncoders((int)(200 * autoSpeedMultiplier), 0.3);
        }
        if(AUTO_MODE == 4){
          moveArmNoEncoders((int)(200 * autoSpeedMultiplier), 0.3);
        }
        prevTime = currentTime = System.currentTimeMillis();
        while(isAutonomous() && isEnabled()){
          currentTime = System.currentTimeMillis();
          
          armLimitHit = !armLimit.get();
          
          AutoUpdateDrive();
          AutoUpdateArm();
          
          if(AUTO_MODE == 2 && autoTickCount == (int)(200 * autoSpeedMultiplier)){
            moveForwardNoEncoders((int)(650 * autoSpeedMultiplier), 0.75);
          }
          if(AUTO_MODE == 3 && autoTickCount == (int)(200 * autoSpeedMultiplier)){
            moveArmNoEncoders((int)(-100 * autoSpeedMultiplier), 0.3);
            moveForwardNoEncoders((int)(350 * autoSpeedMultiplier), 1.0);
          }
          if(AUTO_MODE == 3 && autoTickCount == (int)(300 * autoSpeedMultiplier)){
            moveArmNoEncoders((int)(-500 * autoSpeedMultiplier), 0.1);
          }
          if(AUTO_MODE == 4){
            if(autoTickCount == (int)(200 * autoSpeedMultiplier)){
              moveForwardNoEncoders((int)(200 * autoSpeedMultiplier), 0.75);
            }
            if(autoTickCount == (int)(400 * autoSpeedMultiplier)){
              moveArmNoEncoders((int)(-300 * autoSpeedMultiplier), 0.3);
              moveForwardNoEncoders((int)(450 * autoSpeedMultiplier), 0.75);
            }
          }
          
          mrobotDrive.arcadeDrive(autoLowPassFilter.calculate(driverRobotSpeed), rightDriveMotorLowPassFilter.calculate(driverRobotTurnDirection));
          leftArm.set(leftArmSpeed);
          rightArm.set(rightArmSpeed);
          
          autoTickCount++;
          
          try{
            Thread.sleep(1);
          }
          catch(InterruptedException e){
            e.printStackTrace();
          }
          
          prevTime = currentTime;
        }
      }
      
      driverRobotSpeed = 0.0;
      driverRobotTurnDirection = 0.0;
    }
    
    /**
     * Update arm in autonomous mode.
     */
    private void AutoUpdateArm(){
      if(autoDesiredTicksMovingArm > 0){
        autoDesiredTicksMovingArm--;
        setArmSpeed(autoArmSpeed);
      }
      else if(autoDesiredTicksMovingArm < 0){
        autoDesiredTicksMovingArm++;
        setArmSpeed(-autoArmSpeed);
      }
      else{
        autoArmSpeed = 0;
        setArmSpeed(0);
      }
    }
    
    /**
     * Update drive in autonomous mode.
     */
    private void AutoUpdateDrive(){
      if(autoDesiredTicksDriving > 0){
        autoDesiredTicksDriving--;
        driverRobotSpeed = autoDriveSpeed;
      }
      else if(autoDesiredTicksDriving < 0){
        autoDesiredTicksDriving++;
        driverRobotSpeed = -autoDriveSpeed;
      }
      else{
        autoDriveSpeed = 0;
        driverRobotSpeed = 0;
        driverRobotTurnDirection = 0;
      }
    }
    
    /*public void autonomous()
    {
      mrobotDrive.setSafetyEnabled(false);
      
      autoTickCount = 0;
      autoArmSpeed = 0;
      autoDriveSpeed = 0;
      autoDesiredTicksMovingArm = 0;
      autoDesiredTicksDriving = 0;
      
      if(AUTO_MODE != 0){
        if(AUTO_MODE == 1){
          moveForwardNoEncoders((int)(300 * autoSpeedMultiplier), 1.0);
        }
        if(AUTO_MODE == 2){
          moveArmNoEncoders((int)(200 * autoSpeedMultiplier), 0.3);
        }
        if(AUTO_MODE == 3){
          moveArmNoEncoders((int)(200 * autoSpeedMultiplier), 0.3);
        }
        if(AUTO_MODE == 4){
          moveArmNoEncoders((int)(200 * autoSpeedMultiplier), 0.3);
        }
        prevTime = currentTime = System.currentTimeMillis();
        while(isAutonomous() && isEnabled()){
          currentTime = System.currentTimeMillis();
          
          armLimitHit = !armLimit.get();
          
          AutoUpdateDrive();
          AutoUpdateArm();
          
          if(AUTO_MODE == 2 && inTimeRange((int)(200 * autoSpeedMultiplier))){
            moveForwardNoEncoders((int)(650 * autoSpeedMultiplier), 0.75);
          }
          if(AUTO_MODE == 3 && inTimeRange((int)(200 * autoSpeedMultiplier))){
            moveArmNoEncoders((int)(-100 * autoSpeedMultiplier), 0.3);
            moveForwardNoEncoders((int)(350 * autoSpeedMultiplier), 1.0);
          }
          if(AUTO_MODE == 3 && inTimeRange((int)(300 * autoSpeedMultiplier))){
            moveArmNoEncoders((int)(-500 * autoSpeedMultiplier), 0.1);
          }
          if(AUTO_MODE == 4){
            if(inTimeRange((int)(200 * autoSpeedMultiplier))){
              moveForwardNoEncoders((int)(200 * autoSpeedMultiplier), 0.75);
            }
            if(inTimeRange((int)(400 * autoSpeedMultiplier))){
              moveArmNoEncoders((int)(-300 * autoSpeedMultiplier), 0.3);
              moveForwardNoEncoders((int)(450 * autoSpeedMultiplier), 0.75);
            }
          }
          
          mrobotDrive.arcadeDrive(autoLowPassFilter.calculate(driverRobotSpeed), rightDriveMotorLowPassFilter.calculate(driverRobotTurnDirection));
          leftArm.set(leftArmSpeed);
          rightArm.set(rightArmSpeed);
          
          autoTickCount++;
          
          try{
            Thread.sleep(1);
          }
          catch(InterruptedException e){
            e.printStackTrace();
          }
          
          prevTime = currentTime;
        }
      }
      
      driverRobotSpeed = 0.0;
      driverRobotTurnDirection = 0.0;
    }
    
    private void AutoUpdateArm(){
      int passedTime = (int)(currentTime - prevTime);
      if(autoDesiredTicksMovingArm > 0){
        if(oppositeSign(autoDesiredTicksDriving - passedTime, autoDesiredTicksDriving)){
          autoDesiredTicksMovingArm = 0;
          setArmSpeed(0);
        }
        else{
          autoDesiredTicksMovingArm -= passedTime;
          setArmSpeed(autoArmSpeed);
        }
      }
      else if(autoDesiredTicksMovingArm < 0){
        if(oppositeSign(autoDesiredTicksDriving - passedTime, autoDesiredTicksDriving)){
          autoDesiredTicksMovingArm = 0;
          setArmSpeed(0);
        }
        else{
          autoDesiredTicksMovingArm += passedTime;
          setArmSpeed(-autoArmSpeed);
        }
      }
      else{
        autoArmSpeed = 0;
        setArmSpeed(0);
      }
    }
    
    private void AutoUpdateDrive(){
      int passedTime = (int)(currentTime - prevTime);
      if(autoDesiredTicksDriving > 0){
        if(oppositeSign(autoDesiredTicksDriving - passedTime, autoDesiredTicksDriving)){
          autoDesiredTicksDriving = 0;
        }
        else{
          autoDesiredTicksDriving -= passedTime;
        }
        driverRobotSpeed = autoDriveSpeed;
      }
      else if(autoDesiredTicksDriving < 0){
        if(oppositeSign(autoDesiredTicksDriving + passedTime, autoDesiredTicksDriving)){
          autoDesiredTicksDriving = 0;
        }
        else{
          autoDesiredTicksDriving += passedTime;
        }
        driverRobotSpeed = -autoDriveSpeed;
      }
      else{
        autoDriveSpeed = 0;
        driverRobotSpeed = 0;
        driverRobotTurnDirection = 0;
      }
    }*/
    
	private boolean inTimeRange(int n){
      return n > prevTime && n <= currentTime;
    }
	
    /*
     * -------------------------Miscellaneous code-----------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------ 
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     * ------------------------------------------------------------------------------
     */
    
    // Called once each time the robot enters the disabled state.
    public void disabled() 
    {  
      leftDriveMotorTalon.set(0);
      rightDriveMotorTalon.set(0);  //disable all motors if disable is called
      leftArm.set(0);
      rightArm.set(0);
      shooter.set(0);
      intake.set(0);
      rollerWheels.set(0);
      compressor.stop();
    }
	
	// Tells whether num1 has the opposite sign as num2
	private boolean oppositeSign(int num1, int num2){
      if(num1 < 0 && num2 > 0){
        return true;
      }
      else if(num1 > 0 && num2 < 0){
        return true;
      }
      return false;
    }
    
    /**
     * If num is greater than or equal to theshold, return num, otherwise return 0.
     * @param num
     * @param threshold
     * @return
     */
    private double threshold(double num, double threshold){
      if(Math.abs(num) <= threshold){
        return 0.0;
      }
      return num;
    }
    
    /**
     * Limit the magnitude of num to thresh.
     * @param num
     * @param thresh
     * @return
     */
    private double limit(double num, double thresh){
      if(num > thresh){
        return thresh;
      }
      else if(num < -thresh){
        return -thresh;
      }
      return num;
    }
    
    //--------------------------------------------------------------------------------------------
    double EALerp(double y0,double y1,double x0,double x1,double x2) 
    {
      double y2 = y0*(x2-x1) / (x0-x1)+y1*(x2-x0) / (x1-x0);
      return y2;
    }

    //--------------------------------------------------------------------------------------------
    double BoundEALerp(double y0,double y1,double x0,double x1,double x2) 
    {
      double y2 = y0*(x2-x1) / (x0-x1)+y1*(x2-x0) / (x1-x0);

      if (y2 < y0) y2=y0;
      else
        if (y2 > y1) y2=y1;
      return y2;
    }

} //public Class Robot

class AutoEvent{
  
  private int targetTime;
  
  private String name;
  
  public AutoEvent(int targetTime, String name){
    this.targetTime = targetTime;
    this.name = name;
  }
  
  public boolean shouldExecute(int timePassed){
    return timePassed >= targetTime;
  }
  
  public String getName(){
    return name;
  }
  
}