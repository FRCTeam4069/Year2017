package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class MoveControl
{
  private Talon leftDriveMotor;
  private Talon rightDriveMotor;

  public Encoder leftEncoder;
  public Encoder rightEncoder;

  private LowPassFilter leftDriveMotorLowPassFilter;
  private LowPassFilter rightDriveMotorLowPassFilter;
  private Joystick driverStick;

  private final double TICKS_PER_ENCODER_ROTATION = 256;

  private final double TICKS_PER_WHEEL_ROTATION = 150;
  private final double WHEEL_CIRCUMFERENCE_IN_CM = 12.566; // 12.566cm = 4" diameter 2"radius= 2 * 6.28

  private final double ERROR_SCALING_CONST_P = 10;
  private final double CM_PER_TICK = WHEEL_CIRCUMFERENCE_IN_CM / TICKS_PER_WHEEL_ROTATION;

  private final double TICKS_PER_CM = TICKS_PER_WHEEL_ROTATION / WHEEL_CIRCUMFERENCE_IN_CM;

  private double DRIVE_STRAIGHT_SPEED = 0.4;

  private double driveStrCM = 0;

  public RobotDrive mRobotDrive; // class that handles basic drive
  public double driverRobotSpeed = 0; // velocity of robot -1 full reverse, +1 = full
  public double driverRobotTurnDirection = 0; // x axis of drivers left joystick

  // Robot Geometry
  double mDistanceBetweenWheelCentersInCM = 30 * 2.54;

  double mLeftEncoderOrigValue = 0;
  double mRightEncoderOrigValue = 0;

  private MoveStatus mCurrentCommand = MoveStatus.DRIVE_STOPPED;
  
  double mSpeed = 0;
  double mDistance = 0;
  double mDirection = 0;  //-1 hard left, +1 hard right, 0 straight
  
  double mCurTime = System.currentTimeMillis();

  double mCurVelocity = 0.0;

  private enum MoveStatus
  {
    DRIVE_STOPPED, DRIVE_STRAIGHT, DRIVE_TRACE, DRIVE_OPERATOR_CONTROL, DRIVE_NEXT_COMMAND
  }

  // ALL class global vars go above here!

  /**
   * Class Constructor MoveFunctions Constructor, given the driver stick, setup motors and encoders assign low pass filters to motors
   * 
   * @param driverstk
   */
  public MoveControl(Joystick driverstk)
  {
    driverStick = driverstk;
    leftDriveMotor = new Talon(IOMapping.LEFT_DRIVE_MOTOR_PWM_PORT);
    rightDriveMotor = new Talon(IOMapping.RIGHT_DRIVE_MOTOR_PWM_PORT);

    leftDriveMotorLowPassFilter = new LowPassFilter(250); // prevent motors tearing gears apart
    rightDriveMotorLowPassFilter = new LowPassFilter(250);

    leftEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
    rightEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);

    leftEncoder.setDistancePerPulse(CM_PER_TICK); // Tell encoders how far a tick is (in meters)
    rightEncoder.setDistancePerPulse(CM_PER_TICK);

    mRobotDrive = new RobotDrive(leftDriveMotor, rightDriveMotor);
    mRobotDrive.setInvertedMotor(MotorType.kRearLeft, true);
    mRobotDrive.setInvertedMotor(MotorType.kRearRight, true);
    mRobotDrive.setExpiration(0.1);
  } // MoveFunctions constructor

  public double AverageDistanceTraveledInCM()
  {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    double averageDistance = (leftDistance + rightDistance) / 2;
    return averageDistance;
  }

  /******
   * NOTE Begin Tick parser and Tick Behavior Routines Tick is called from robot.java's main loop periodically Current Move function should update during this call.
   */
  public void Tick()
  {
    switch (mCurrentCommand)
    {
    case DRIVE_STRAIGHT:
      Drive_Straight_Tick();
    case DRIVE_TRACE:
      Drive_Trace_Tick();
      break;
    case DRIVE_STOPPED:
      Stop_Tick();
      break;
    case DRIVE_OPERATOR_CONTROL:
      Drive_Operator_Tick();
      break;
    case DRIVE_NEXT_COMMAND: // pull next drive command off list and execute it

      break;
    }
  }// Tick

  public void Stop()
  {
    mCurrentCommand = MoveStatus.DRIVE_STOPPED;
    leftDriveMotor.set(0); // usually would let tick do this, but stopping is serious biz
    rightDriveMotor.set(0);
  }// Stop

  private void Stop_Tick()
  {
    leftDriveMotor.set(0);
    rightDriveMotor.set(0);
  }

  private void Drive_Operator_Tick()
  {
    // Combined speed of the drive motors
    double combined = driverStick.getRawAxis(3) * -1 + driverStick.getAxis(AxisType.kZ);
    driverRobotSpeed = leftDriveMotorLowPassFilter.calculate(combined);
    // Calculate robot turn direction from the left drive joystick's x-axis
    // (left-right)
    driverRobotTurnDirection = rightDriveMotorLowPassFilter.calculate(driverStick.getAxis(AxisType.kX));

    // Last move robot. Let the robotdrive class handle the driving aspect of
    // the robot
    mRobotDrive.arcadeDrive(driverRobotSpeed, driverRobotTurnDirection); // move robot
  }// Drive_Operator_Tick

  public void MoveStraight(double speed, double distance)
  {
    mCurrentCommand = MoveStatus.DRIVE_STRAIGHT;
    DRIVE_STRAIGHT_SPEED = speed;
    driveStrCM = distance;
    leftEncoder.reset();
    rightEncoder.reset(); // just zero encoders for straight moves
  }

  private void Drive_Straight_Tick()
  {
    double leftDistance = leftEncoder.getDistance();
    double rightDistance = rightEncoder.getDistance();
    double averageDistance = (leftDistance + rightDistance) / 2;

    if (averageDistance >= driveStrCM)
    {
      Stop();
      mCurrentCommand = MoveStatus.DRIVE_NEXT_COMMAND;
      return;
    }

    double error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
                                                 // if error < 0 right is ahead add -error to right
    double correctionFactor = error * ERROR_SCALING_CONST_P; // amplify error for effect

    leftDriveMotor.set(DRIVE_STRAIGHT_SPEED - correctionFactor); // +err means left ahead, subtract from left speed
    rightDriveMotor.set(-(DRIVE_STRAIGHT_SPEED + correctionFactor)); // -err means right ahead, add -err to right speed
  }// Drive_Straight_Tick

  
  /**
   * MoveTrace direction is -1 to 1, -1 means drive hard left, 1 means drive hard right
   * @param dir -1 left, +1 right 0 straight
   * @param distance  distance in CM to travel
   * @param speed  base speed to travel, scaled by direction
   */
  public void MoveTrace(double dir,double distance,double speed)
  {
    mCurrentCommand=MoveStatus.DRIVE_TRACE;
    mSpeed = speed;
    mDistance = distance;
    mDirection = dir;
    leftEncoder.reset();
    rightEncoder.reset(); //clear encoder distances/ticks
  }
  
  private void Drive_Trace_Tick()
  {
    if (AverageDistanceTraveledInCM() < mDistance)
    {
      double leftSpeed = mSpeed + (mSpeed * (-1 * mDirection));
      double rightSpeed= mSpeed + (mSpeed * (mDirection));
      leftDriveMotor.set(leftSpeed);
      rightDriveMotor.set(rightSpeed);
    }
    else
    {
      Stop();
      mCurrentCommand = MoveStatus.DRIVE_NEXT_COMMAND;
    }
    
  }// Drive_Curve_Tick

  
  
  
} // class MoveFunctions