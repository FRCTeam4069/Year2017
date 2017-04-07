package org.usfirst.frc.team4069.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;

public class ControlMove
{
  // Objects MoveControl interfaces with...
  public RobotDrive mRobotDrive; // class that handles basic drive

  public Talon leftDriveMotor;
  public Talon rightDriveMotor;

  public Encoder leftEncoder;
  public Encoder rightEncoder;

  public LowPassFilter leftDriveMotorLowPassFilter;
  public LowPassFilter rightDriveMotorLowPassFilter;
  public Joystick driverStick;

  // Hard coded values from robot/encoder geometry
  public static final double TICKS_PER_ENCODER_ROTATION = 256;

  public static final double TICKS_PER_WHEEL_ROTATION = 150;

  public static final double WHEEL_CIRCUMFERENCE_IN_CM = 12.566; // 12.566cm = 4" diameter 2"radius= 2 * 6.28

  public static final double CM_PER_TICK = WHEEL_CIRCUMFERENCE_IN_CM / TICKS_PER_WHEEL_ROTATION;

  public static final double TICKS_PER_CM = TICKS_PER_WHEEL_ROTATION / WHEEL_CIRCUMFERENCE_IN_CM;

  // Robot Geometry
  public static final double mDistanceBetweenWheelCentersInCM = 24 * 2.54; // Diameter of Robot's drive base
  public static final double mDriveBaseRadius = mDistanceBetweenWheelCentersInCM / 2;
  public static final double mDriveBaseCircumference = mDistanceBetweenWheelCentersInCM * Math.PI; //
  public static final double mDriveBaseTicksInCircumference = TICKS_PER_CM * mDriveBaseCircumference;

  private Robot mRobot;

  // Current command executing, and a list of commands to be executed...
  private MoveCommand mCurrentCommand = null;
  ArrayList<MoveCommand> mCommandList = new ArrayList<MoveCommand>(); // Arrays.asList(new TurnOneWheelCommand(.25, 50, false), new StopCommand()));

  /**
   * Class Constructor MoveFunctions Constructor, given the driver stick, setup motors and encoders assign low pass filters to motors
   * 
   * @param driverstk
   */
  public ControlMove(Joystick driverstk, Robot robot)
  {
    driverStick = driverstk;

    mRobot = robot;

    leftDriveMotor = new Talon(IOMapping.LEFT_DRIVE_MOTOR_PWM_PORT);
    rightDriveMotor = new Talon(IOMapping.RIGHT_DRIVE_MOTOR_PWM_PORT);

    leftDriveMotorLowPassFilter = new LowPassFilter(125); // prevent motors tearing gears apart
    rightDriveMotorLowPassFilter = new LowPassFilter(125);

    leftEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
    rightEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);

    leftEncoder.setDistancePerPulse(CM_PER_TICK); // Tell encoders how far a tick is (in centimeters)
    rightEncoder.setDistancePerPulse(CM_PER_TICK);

    leftEncoder.setReverseDirection(true);

    mRobotDrive = new RobotDrive(leftDriveMotor, rightDriveMotor);
    mRobotDrive.setExpiration(0.1);
  } // MoveFunctions constructor

  public double AverageDistanceTraveledInCM()
  {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    double averageDistance = (leftDistance + rightDistance) / 2;
    return averageDistance;
  }

  /**
   * Checks mCommandList to see if any move commands are waiting, if so, pulls the next one and calls its initialization routine to execute it.
   * 
   * @return Number of commands waiting for execution -1 if none were waiting
   */
  public int DoNextCommand()
  {
    // System.out.println("DoNextCommand....");
    if (mCommandList.size() == 0) // when no commands left, stop.
    {
      System.out.println("Commandlist empty, adding stop command");
      addStopCMD();
      mCurrentCommand = null;
      return -1; // return -1 in the case where no commands were available
    }
    MoveCommand mc = mCommandList.get(0); // Trigger next command on next Tick()
    mCommandList.remove(0);
    mc.Init();
    mCurrentCommand = mc;
    // System.out.println("DoNextcommand returning with "+mCommandList.size());
    return mCommandList.size();
  }// doNextCommand

  /**
   * If no commands on list, and no current command, return 1, else return 0
   * 
   * @return
   */
  public int isControlMoveFinished()
  {
    if ((mCommandList.size() == 0) && (mCurrentCommand == null))
      return 1;
    else
      return 0;
  }

  public void Tick()
  {
    // System.out.println("MainTick list = "+mCommandList.size()+" mcurrentcommand="+mCurrentCommand);
    if ((mCurrentCommand == null) || (mCurrentCommand.Tick() == true)) // done?
    {
      // System.out.println("About to donextcommand num="+mCommandList.size());
      if (DoNextCommand() == -1)
      {
        mRobot.mMoveController.mRobotDrive.arcadeDrive(0, 0);
      }
    }
  }// Tick

  // -----------------------------------------------------------------------------
  // Add_commands below...

  public void addPivotCMD(double degrees, double speed)
  {
    MoveCommandPivot p = new MoveCommandPivot(this, degrees, speed);
    mCommandList.add(p);
  }// Pivot

  public void addStopCMD()
  {
    MoveCommandStop s = new MoveCommandStop(this);
    mCommandList.add(s);
  }// Stop

  public void addMoveStraightCMD(double speed, double distance)
  {
    mCommandList.add(new MoveCommandStraight(this, speed, distance));
  }

  public void addDoTurnCMD(boolean isRightWheel)
  {
    mCommandList.add(new MoveCommandTurnOneWheel(this, .80, 20, isRightWheel));
  }

  public void addDelayCMD(int milliseconds)
  {
    mCommandList.add(new MoveCommandDelay(this, milliseconds));
  }

  /**
   * Set current command to Operator Control (tele-operator mode) for human driving. There is no exit from this, it stays in effect until changed by something outside. NOTE: This command clears all pending commands and 'takes control' we may want to preserve list in future?
   */
  public void MoveOperatorControl()
  {
    mCommandList.clear(); // NOTE! Operator takes control clear command list, no more autonomous stuff
    OperatorControlCommand oc = new OperatorControlCommand(this, mRobot);
    oc.Init();
    mCurrentCommand = oc;
  }
} // class MoveFunctions