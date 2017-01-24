package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class MoveFunctions
{
  public static final int DRIVE_STOPPED = 0;
  public  static final int DRIVE_STRAIGHT= 1;
  public static final int DRIVE_CURVE = 2;
  public static final int DRIVE_TRACE = 3;
  
  private Talon leftDriveMotor;
  private Talon rightDriveMotor;
  public  Encoder leftEncoder;
  public Encoder rightEncoder;
  private LowPassFilter leftDriveMotorLowPassFilter;
  private LowPassFilter rightDriveMotorLowPassFilter;
  private Joystick driverStick;
  
  
  public RobotDrive mRobotDrive; // class that handles basic drive
  public double driverRobotSpeed = 0; // velocity of robot -1 full reverse, +1 = full
  public double driverRobotTurnDirection = 0; // x axis of drivers left joystick
  
  //Robot Geometry
  double mDistanceBetweenWheelCentersInCM = 30*2.54;
  int mEncoderTicksPerRevolution=256; //one turn of the encoder gives how many ticks?
  
  double mEncoderTicksPerCM = mEncoderTicksPerRevolution / (3*2.54 * 3.14159*2);  //fill this in, this is just a weird 6" guesstimate no gearing 2piR 
  double mCMPerEncoderTick = 1/mEncoderTicksPerCM;
  
  
  double mLeftEncoderOrigValue=0;
  double mRightEncoderOrigValue = 0;
  
  int mType=0;
  double mSpeed=0;
  double mDistanceWanted=0;
  double mDirection = 0;
  double mCurTime=System.currentTimeMillis();

  double mCurVelocity = 0.0;
  
  
  
  public MoveFunctions(Joystick driverstk)
  {
    driverStick = driverstk;
    leftDriveMotor = new Talon(IOMapping.LEFT_DRIVE_MOTOR_PWM_PORT);
    rightDriveMotor = new Talon(IOMapping.RIGHT_DRIVE_MOTOR_PWM_PORT);
    leftDriveMotorLowPassFilter = new LowPassFilter(350);
    rightDriveMotorLowPassFilter = new LowPassFilter(250);

    leftEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
    rightEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);

    
    mRobotDrive = new RobotDrive(leftDriveMotor, rightDriveMotor);
    mRobotDrive.setInvertedMotor(MotorType.kRearLeft, true);
    mRobotDrive.setInvertedMotor(MotorType.kRearRight, true);
    mRobotDrive.setExpiration(0.1);
  }
  
  
  /**************************
   * This is ugly, but its what the drive team wanted
   * 4069 drivers like their drive buttons configured in a interesting manner.
   * This routine sets the robot speed and direction vars for update to arcade
   * drive
   * 
   * @author EA
   */
  public boolean UpdateDriverInputs()
  {
    // Combined speed of the drive motors
    double combined = driverStick.getRawAxis(3) * -1 + driverStick.getAxis(AxisType.kZ);
    driverRobotSpeed = leftDriveMotorLowPassFilter.calculate(combined);
    // Calculate robot turn direction from the left drive joystick's x-axis
    // (left-right)
    driverRobotTurnDirection = rightDriveMotorLowPassFilter.calculate(driverStick.getAxis(AxisType.kX));
    return true;
  } //updatedrivemotors
  
  
  /**
   * Have robot move from current location and direction to a location distance 
   */
  public void Move(double speed,double direction,double distance)
  {
    mLeftEncoderOrigValue = leftEncoder.get();
    mRightEncoderOrigValue= rightEncoder.get();
    mSpeed =speed;
    mDirection = direction;
    mDistanceWanted = distance;
  }//Move

  public void MoveToRelativePosition(double dx,double dy,double speed)
  {
    
  } 
  
  /**
   * Tick is called from robot.java's main loop periodically Current Move function should update during this
   * call.
   */
  public void Tick()
  {
    switch(mType)
    {
      case DRIVE_STRAIGHT:
        break;
      case DRIVE_CURVE :
        break;
      case DRIVE_STOPPED:
        break;
    }
  }//Tick
  
  private void Drive_Operator_Tick()
  {
    // Last move robot. Let the robotdrive class handle the driving aspect of
    // the robot
    mRobotDrive.arcadeDrive(driverRobotSpeed, driverRobotTurnDirection); // move robot

  }
  
  
  private void Drive_Straight_Tick()
  {
  }
  private void Drive_Curve_Tick()
  {
    
  }
  private void Drive_Stopped_Tick()
  {
    
  }
  
  
}