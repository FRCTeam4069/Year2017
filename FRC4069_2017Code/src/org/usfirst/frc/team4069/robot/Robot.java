package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends SampleRobot
{
 private ShooterControl mShooterController;   //shooter functions
 private WinchControl mWinchController;        //winch functions
 private MoveControl mMoveController;        //ALL robot movement functions
 
  Preferences prefs = Preferences.getInstance();

  Joystick driverStick = new Joystick(0); // set to ID 1 in DriverStation
  Joystick controlStick = new Joystick(1); // set to ID 2 in DriverStation

  long mLastDashboardUpdateTime = 0;

  VideoCaptureThread video_capture_instance;
  Thread VideoCaptureThreadHandle;
  
  VisionThreadNew vision_processor_instance;
  Thread VisionProcessorThreadHandle;
  
  
  @Override
  public void robotInit()
  {
  }

  /********************************************************************************
   * Robot Constructor, init all variables, create and start Thread(s)
   */
  public Robot()
  {
    mShooterController = new ShooterControl(controlStick);
    mWinchController = new WinchControl();
    mMoveController = new MoveControl(driverStick); //pass joystick

    video_capture_instance = new VideoCaptureThread();
    VideoCaptureThreadHandle = new Thread(video_capture_instance);
    VideoCaptureThreadHandle.start();
    video_capture_instance.Enable();  //begin getting frames.
 
    vision_processor_instance = new VisionThreadNew(video_capture_instance,VideoCaptureThreadHandle);  //pass in refs to video capture thread so it can grab frames
    
    VisionProcessorThreadHandle = new Thread(vision_processor_instance);
    VisionProcessorThreadHandle.start();
    
    mLastDashboardUpdateTime = System.currentTimeMillis();
  }// Robot()

  //---------------------------------------------------------------------------------------------

  @Override
  public void operatorControl()
  {
    mMoveController.mRobotDrive.setSafetyEnabled(true);
    SendDataToSmartDashboard();
    while (isOperatorControl() && isEnabled())
    {
      InputSystem.ReadAllInput(driverStick, controlStick); // Read all sensor/input devices

      // ALL UPDATE ROUTINES updating based on read/updated sensor values
      mShooterController.Tick(); // update shooter
      mWinchController.Tick();
      mMoveController.Tick();   //Give move functions a tick (unrelated to updatedriverinputs)
      
      double xc = vision_processor_instance.xcenter;
 
      SendDataToSmartDashboard();

      Timer.delay(0.005); // wait for a motor update time
    } // while isEnabled
  } // operatorControl

  public void autonomous()
  {
    mMoveController.mRobotDrive.setSafetyEnabled(false);
    mMoveController.leftEncoder.reset();
    mMoveController.rightEncoder.reset();
    mMoveController.MoveStraight(0.25, 2);
    while (isAutonomous() && isEnabled()) {
      mMoveController.Tick();
    }
    
  }
  
  
  /**
   * Update smart dashboard every 1 second
   */
  void SendDataToSmartDashboard()
  {
    long deltat = mLastDashboardUpdateTime - System.currentTimeMillis();
    if (deltat > 200) //5 times/sec
    {
      System.out.println("xcenter = " + vision_processor_instance.xcenter);
      SmartDashboard.putNumber("LEFTENCODER", mMoveController.leftEncoder.get());
      SmartDashboard.putNumber("RIGHTENCODER", mMoveController.rightEncoder.get());
      SmartDashboard.putNumber("SHOOTERENCODER", mShooterController.GetShooterPosition());
      SmartDashboard.putNumber("XCENTER", vision_processor_instance.xcenter);
      SmartDashboard.putNumber("CM Traveled:", mMoveController.AverageDistanceTraveledInCM());
      mLastDashboardUpdateTime = System.currentTimeMillis();
    }
  } // SendDataToSmartDashboard
  
  
  //--------------------------------------------------------------------------------------------
  /**
   * Linear Interpolation, given a value x2 between x0 and x1 calculate position between Y0 and Y1
   * @author EA
   */
  public double Lerp(double y0,double y1,double x0,double x1,double x2) 
  {
    double y2 = y0*(x2-x1) / (x0-x1)+y1*(x2-x0) / (x1-x0);
    return y2;
  }

  //--------------------------------------------------------------------------------------------
  /**
   * Bounded Linear Interpolation : Limit returned value to be between Y0 and Y1
   * @author EA
   */
  public double BoundLerp(double y0,double y1,double x0,double x1,double x2) 
  {
    double y2 = y0*(x2-x1) / (x0-x1)+y1*(x2-x0) / (x1-x0);
    if (y2 < y0) y2=y0;
    else
      if (y2 > y1) y2=y1;
    return y2;
  }

  
  

  /**********************************************************************************
   * InputSystem static class holds all control input states ReadAllInput()
   * should be called at top of main loop to refresh states then all Update
   * routines will pull from here to make their decisions.
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
