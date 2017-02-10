package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;

import org.usfirst.frc.team4069.robot.ControlMove.TurnOneWheelCommand;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot
{
  //private ShooterControl mShooterController; // shooter functions
  private ControlWinch mWinchController; // winch functions
  private ControlMove mMoveController; // ALL robot movement functions
  private ControlTurret mTurretController;
  
  Preferences prefs = Preferences.getInstance();

  Joystick driverStick = new Joystick(0); // set to ID 1 in DriverStation
  Joystick controlStick = new Joystick(1); // set to ID 2 in DriverStation

  long mLastDashboardUpdateTime = 0;

  ThreadVideoCapture video_capture_instance;
  Thread VideoCaptureThreadHandle;

  ThreadVisionNew vision_processor_instance;
  Thread VisionProcessorThreadHandle;
  
  ThreadArduino arduino_thread_instance;
  Thread arduinoThreadHandle;
  
  ThreadLIDAR lidar_instance;
  Thread lidarThreadHandle;
  
  public int ctr = 0;

  
  @Override
  public void robotInit()
  {
  }

  /********************************************************************************
   * Robot Constructor, init all variables, create and start Thread(s)
   */
  public Robot()
  {
  //  mShooterController = new ShooterControl(controlStick);
    mWinchController = new ControlWinch();
    mMoveController = new ControlMove(driverStick); // pass joystick
    mTurretController = new ControlTurret(this);
    
    lidar_instance = new ThreadLIDAR();
    lidarThreadHandle = new Thread(lidar_instance);
    lidarThreadHandle.start();
    
    video_capture_instance = new ThreadVideoCapture();
    VideoCaptureThreadHandle = new Thread(video_capture_instance);
    VideoCaptureThreadHandle.start();
    video_capture_instance.Enable(); // begin getting frames.


    vision_processor_instance = new ThreadVisionNew(video_capture_instance, VideoCaptureThreadHandle,this); // pass in refs to video capture thread so it can grab frames
    VisionProcessorThreadHandle = new Thread(vision_processor_instance);
    VisionProcessorThreadHandle.start();
    
    
    arduino_thread_instance = new ThreadArduino();
    arduinoThreadHandle = new Thread(arduino_thread_instance);
    arduinoThreadHandle.start();

    mMoveController.leftEncoder.reset();
    mMoveController.rightEncoder.reset();
    mMoveController.MoveOperatorControl();
    
    
    
    mLastDashboardUpdateTime = System.currentTimeMillis();
  }// Robot()

  
  
  // ---------------------------------------------------------------------------------------------

  @Override
  public void operatorControl()
  {
    mMoveController.mRobotDrive.setSafetyEnabled(false);
  //  SendDataToSmartDashboard();
    //mShooterController.Enable();
    mMoveController.MoveOperatorControl(); // human driving watch out!
    
    while (isOperatorControl() && isEnabled())
    {
   //   turretCANTalon.set(driverStick.getAxis(AxisType.kY));
      
      InputSystem.ReadAllInput(driverStick, controlStick); // Read all sensor/input devices

      // ALL UPDATE ROUTINES updating based on read/updated sensor values
      //mShooterController.Tick();
      mWinchController.Tick();
      mMoveController.Tick();

      SendDataToSmartDashboard();
      Timer.delay(0.005); // wait for a motor update time
    } // while isEnabled
  } // operatorControl

  
  
  @Override
  public void autonomous()
  {
    mMoveController.mRobotDrive.setSafetyEnabled(false);
    mMoveController.leftEncoder.reset();
    mMoveController.rightEncoder.reset();
    mMoveController.MoveStraight(-0.45, 100);
    mMoveController.DoTurn();
   
    while (isAutonomous() && isEnabled())
    {
      SendDataToSmartDashboard();
      mMoveController.Tick();
      mTurretController.Tick();
      
    }
  }// autonomous

  /**
   * Update smart dashboard every 1 second
   */
  void SendDataToSmartDashboard()
  {
    long deltat = System.currentTimeMillis() - mLastDashboardUpdateTime;
    if (deltat > 1000)
    {
      SmartDashboard.putNumber("LEFTENCODER", mMoveController.leftEncoder.get());
      SmartDashboard.putNumber("RIGHTENCODER", mMoveController.rightEncoder.get());
      SmartDashboard.putNumber("LEFTDISTANCE", mMoveController.leftEncoder.getDistance());
      SmartDashboard.putNumber("RIGHTDISTANCE", mMoveController.rightEncoder.getDistance());
      SmartDashboard.putNumber("Error", mMoveController.error);
      SmartDashboard.putNumber("Correction:", mMoveController.correctionFactor);
      SmartDashboard.putNumber("resultleftspeed", mMoveController.resultantleftspeed);
      SmartDashboard.putNumber("resultantrightspeed",mMoveController.resultantrightspeed);
      SmartDashboard.putNumber("Tickcount:", mMoveController.TickCounter);
      //SmartDashboard.putNumber("SHOOTER MotorOut:", mShooterController.motorOutput);
      //SmartDashboard.putNumber("SHOOTER RPM", mShooterController.shooterCANTalon.getSpeed());
      //SmartDashboard.putNumber("SHOOTER TARGET RPM:", mShooterController.targetRPM);
      //SmartDashboard.putNumber("SHOOTER ERROR:", mShooterController.shooterCANTalon.getClosedLoopError());
      SmartDashboard.putNumber("XCENTER", vision_processor_instance.cregions.mXGreenLine); // .lastXCenter);
      SmartDashboard.putNumber("NumContours:", vision_processor_instance.cregions.mContours.size());
      SmartDashboard.putNumber("MAPPED:", vision_processor_instance.lastMapped);
      SmartDashboard.putNumber("CM Traveled:", mMoveController.AverageDistanceTraveledInCM());
      mLastDashboardUpdateTime = System.currentTimeMillis();
      SmartDashboard.putNumber("LAST HEADING:", arduino_thread_instance.lastHeading);
      SmartDashboard.putNumber("LIDAR Angle:", lidar_instance.lastAngle);
      SmartDashboard.putNumber("LIDAR SS", lidar_instance.lastSignalStrength);
      SmartDashboard.putNumber("LIDAR distance", lidar_instance.lastDistance);
      SmartDashboard.putNumber("LIDAR status:", lidar_instance.lastStatus);
      SmartDashboard.putString("LIDAR LAST ERROR", lidar_instance.lastError);
      SmartDashboard.putString("LIDARMessage:", lidar_instance.lastMessage);
      SmartDashboard.putString("GYRO Last Error", arduino_thread_instance.lastError);
      SmartDashboard.putString("GYRO Message", arduino_thread_instance.lastMessage);
    }
  } // SendDataToSmartDashboard

  // --------------------------------------------------------------------------------------------
  /**
   * Linear Interpolation, given a value x2 between x0 and x1 calculate position between Y0 and Y1
   * 
   * @author EA
   */
  public double Lerp(double y0, double y1, double x0, double x1, double x2)
  {
    double y2 = y0 * (x2 - x1) / (x0 - x1) + y1 * (x2 - x0) / (x1 - x0);
    return y2;
  }

  // --------------------------------------------------------------------------------------------
  /**
   * Bounded Linear Interpolation : Limit returned value to be between Y0 and Y1
   * 
   * @author EA
   */
  public double BoundLerp(double y0, double y1, double x0, double x1, double x2)
  {
    double y2 = y0 * (x2 - x1) / (x0 - x1) + y1 * (x2 - x0) / (x1 - x0);
    if (y2 < y0)
      y2 = y0;
    else if (y2 > y1)
      y2 = y1;
    return y2;
  }

  /**********************************************************************************
   * InputSystem static class holds all control input states ReadAllInput() should be called at top of main loop to refresh states then all Update routines will pull from here to make their decisions.
   * 
   * @author EA
   *
   */
  public static class InputSystem
  {
    // Control stick buttons
    public static boolean Y_Button_Control_Stick = false; //
    public static boolean A_Button_Control_Stick = false;
    public static boolean X_Button_Control_Stick = false;
    public static boolean X_Button_Control_Stick_Prev = false;
    public static boolean B_Button_Control_Stick = false;
    public static boolean RB_Button_Control_Stick = false; // r
    public static boolean RB_Button_Control_Stick_Prev = false;
    // Driver stick buttons
    public static boolean Y_Button_Driver_Stick = false;
    public static boolean A_Button_Driver_Stick = false;
    public static boolean X_Button_Driver_Stick = false; //
    public static boolean B_Button_Driver_Stick = false;
    public static boolean B_Button_Driver_Stick_Prev = false;
    public static boolean RB_Button_Driver_Stick = false; //
    public static boolean RB_Button_Driver_Stick_Prev = false;

    public static void ReadAllInput(Joystick driverstk, Joystick controlstk)
    {
      Y_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_Y_BUTTON);
      A_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_A_BUTTON);
      X_Button_Control_Stick_Prev = X_Button_Control_Stick;
      X_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_X_BUTTON);
      B_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_B_BUTTON);
      RB_Button_Control_Stick_Prev = RB_Button_Control_Stick;
      RB_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_RB_BUTTON);

      Y_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_Y_BUTTON);
      A_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_A_BUTTON);
      X_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_X_BUTTON);
      B_Button_Driver_Stick_Prev = B_Button_Driver_Stick;
      B_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_B_BUTTON);
      RB_Button_Driver_Stick_Prev = RB_Button_Driver_Stick;
      RB_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_RB_BUTTON);
    } // ReadAllInput
  } // public static inputsystem class
} // class Robot
