package org.usfirst.frc.team4069.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Vector;

import org.usfirst.frc.team4069.robot.ControlMove.TurnOneWheelCommand;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;

public class ControlMove
{
  // Objects MoveControl interfaces with...
  public RobotDrive mRobotDrive; // class that handles basic drive

  private Talon leftDriveMotor;
  private Talon rightDriveMotor;

  public Encoder leftEncoder;
  public Encoder rightEncoder;
  public int TickCounter = 0;
  private LowPassFilter leftDriveMotorLowPassFilter;
  private LowPassFilter rightDriveMotorLowPassFilter;
  private Joystick driverStick;

  // Hard coded values from robot/encoder geometry
  private final double TICKS_PER_ENCODER_ROTATION = 256;
  
  private final double TICKS_PER_WHEEL_ROTATION = 150;
  
  private final double WHEEL_CIRCUMFERENCE_IN_CM = 12.566; // 12.566cm = 4" diameter 2"radius= 2 * 6.28
  private final double ERROR_SCALING_CONST_P = .400;
  private final double CM_PER_TICK = WHEEL_CIRCUMFERENCE_IN_CM / TICKS_PER_WHEEL_ROTATION;
  
  private final double TICKS_PER_CM = TICKS_PER_WHEEL_ROTATION / WHEEL_CIRCUMFERENCE_IN_CM;
  
  public double error = 0.0;
  public double correctionFactor = 0.0;
  public double resultantleftspeed = 0.0;
  public double resultantrightspeed = 0.0;
  // Robot Geometry
  double mDistanceBetweenWheelCentersInCM = 24 * 2.54; // Diameter of Robot's drive base
  double mDriveBaseRadius = mDistanceBetweenWheelCentersInCM / 2;
  double mDriveBaseCircumference = mDistanceBetweenWheelCentersInCM * Math.PI; //
  double mDriveBaseTicksInCircumference = TICKS_PER_CM * mDriveBaseCircumference;

  // Current command executing, and a list of commands to be executed...
  private MoveCommand mCurrentCommand = new StopCommand();
  ArrayList<MoveCommand> mCommandList = new ArrayList<MoveCommand>(); //Arrays.asList(new TurnOneWheelCommand(.25, 50, false), new StopCommand()));

  /**
   * Class Constructor MoveFunctions Constructor, given the driver stick, setup motors and encoders assign low pass filters to motors
   * 
   * @param driverstk
   */
  public ControlMove(Joystick driverstk)
  {
    driverStick = driverstk;
    TickCounter = 0;

    leftDriveMotor = new Talon(IOMapping.LEFT_DRIVE_MOTOR_PWM_PORT);
    rightDriveMotor = new Talon(IOMapping.RIGHT_DRIVE_MOTOR_PWM_PORT);

    leftDriveMotorLowPassFilter = new LowPassFilter(250); // prevent motors tearing gears apart
    rightDriveMotorLowPassFilter = new LowPassFilter(250);

    leftEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
    rightEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);

    leftEncoder.setDistancePerPulse(CM_PER_TICK); // Tell encoders how far a tick is (in centimeters)
    rightEncoder.setDistancePerPulse(CM_PER_TICK);

    leftEncoder.setReverseDirection(true);
    
    mRobotDrive = new RobotDrive(leftDriveMotor, rightDriveMotor);
    mRobotDrive.setInvertedMotor(MotorType.kRearLeft, true);
    mRobotDrive.setInvertedMotor(MotorType.kRearRight, false);
    mRobotDrive.setExpiration(0.1);
    error = 0.0;
    correctionFactor = 0.0;
    
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
    if (mCommandList.size() == 0) // when no commands left, stop.
    {
      Stop();
      mCurrentCommand = null;
      return -1; // return -1 in the case where no commands were available
    }
    MoveCommand mc = mCommandList.get(0); // Trigger next command on next Tick()
    mCommandList.remove(0);
    mc.Init();
    mCurrentCommand = mc;

    return mCommandList.size();
  }// doNextCommand

  
  public int isControlMoveFinished()
  {
    if ((mCommandList.size()==0)&&(mCurrentCommand==null))
      return 1;
    else
      return 0;
  } 
  
  /**
   * Call proper tick() routine for mCurrentCommand
   */
  public void Tick()
  {
    if (mCurrentCommand.Tick() == true) // done?
    {
      DoNextCommand();
    }
  }// Tick

  /*
   * Given degrees and speed turn robot in place that many degrees
   * 
   */
  public void Pivot(double degrees, double speed)
  {
    PivotCommand p = new PivotCommand(degrees, speed);
    p.Init();
    mCurrentCommand = p;
  }// Pivot

  /**
   * Set current command to DRIVE_STOPPED, set motors to 0 speed
   */
  public void Stop()
  {
    StopCommand s = new StopCommand();
    s.Init();
    mCurrentCommand = s;
  }// Stop

  /**
   * Set current command to Operator Control (tele-operator mode) for human driving. There is no exit from this, it stays in effect until changed by something outside.
   */
  public void MoveOperatorControl()
  {
    OperatorControlCommand oc = new OperatorControlCommand();
    oc.Init();
    mCurrentCommand = oc;
  }

  /*
   * Setup MoveStraight mode, given speed and distance to travel. Switches to DRIVE_NEXT_COMMAND when reaches distance requested.
   */
  public void MoveStraight(double speed, double distance)
  {
    StraightCommand strcmd = new StraightCommand(speed, distance);
    strcmd.Init();
    mCurrentCommand = strcmd;
  }

  public void DoTurn()
  {
    mCommandList.add(new TurnOneWheelCommand(.55, 50, false));
  }
  
  
  // ----------------------------------------------------------------------------------------
  // CLASS MoveCommand and all Move related Command Classes Derived from it.
  //

  abstract class MoveCommand
  {

    public MoveCommand()
    {
    }

    public abstract boolean Tick();
    
    public abstract void Done();

    public abstract void Init();
  }// class MoveCommand

  /**
   * Operator control, never ends once set.
   * 
   * @author EA
   *
   */
  public class OperatorControlCommand extends MoveCommand
  {
    double mDriverRobotSpeed = 0.0;
    double mDriverRobotTurnDirection = 0.0;

    public OperatorControlCommand()
    {
    	
    }

    @Override
    public void Init()
    {

    }

    @Override
    public boolean Tick()
    {
      // Combined speed of the drive motors
      double combined = driverStick.getRawAxis(3) * -1 + driverStick.getAxis(AxisType.kZ);
      mDriverRobotSpeed = leftDriveMotorLowPassFilter.calculate(combined);
      // Calculate robot turn direction from the left drive joystick's x-axis
      // (left-right)
      mDriverRobotTurnDirection = rightDriveMotorLowPassFilter.calculate(driverStick.getAxis(AxisType.kX));

      // Last move robot. Let the robotdrive class handle the driving aspect of
      // the robot
      mRobotDrive.arcadeDrive(mDriverRobotSpeed, mDriverRobotTurnDirection); // move robot
      return false;
    }
    
    @Override
    public void Done() 
    {
    }
  }// OperatorControlCommand

  /*
   * Given a number of degrees (0-360) and speed rotates robot in place that number of degrees NOTE: + degrees turns clockwise - degrees turns anticlockwise
   */

  public class PivotCommand extends MoveCommand
  {
    double mDistance = 0.0;
    double mSpeed;
    double mDegrees = 0;

    public PivotCommand(double degrees, double speed)
    {
      mSpeed = speed;
      mDegrees = degrees;

      degrees %= 360;
      double frac = degrees / 360; // fractional amount
      double bothwheeldist = -1 * frac * mDriveBaseCircumference;
      mDistance = bothwheeldist;
    }

    @Override
    public void Init()
    {
      leftEncoder.reset();
      rightEncoder.reset(); // zero counts
    }

    @Override
    public void Done()
    {

    }

    // For + degrees left wheel goes forward, right wheel goes backwards
    @Override
    public boolean Tick()
    {
      double leftDistance = leftEncoder.getDistance(); // get left encoder as normal distance
      double rightDistance = rightEncoder.getDistance() * -1; // right wheel is going backwards so distance is * -1

      double averageDistance = (leftDistance + rightDistance) / 2;

      if (averageDistance >= mDistance)
      {
        leftDriveMotor.set(0);
        rightDriveMotor.set(0);
        Done();
        return true;
      }

      error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
                                            // if error < 0 right is ahead add -error to right
      correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

      leftDriveMotor.set(mSpeed - correctionFactor); // +err means left ahead, subtract from left speed
      rightDriveMotor.set(mSpeed + correctionFactor); // -err means right ahead, add -err to right speed
      return false; // not done yet
    }// Tick
  }// PivotCommand

  /*
   * Stops all Drive motors.
   */
  public class StopCommand extends MoveCommand
  {
    StopCommand()
    {
    }

    @Override
    public void Init()
    {
      leftDriveMotor.set(0);
      rightDriveMotor.set(0);
    }

    @Override
    public boolean Tick()
    {
      leftDriveMotor.set(0);
      rightDriveMotor.set(0);
      return true;
    }
    
    @Override
    public void Done()
    {
    }
  } // StopCommand

  /**
   * Passed a speed and distance, uses encoders to travel straight line at speed for distance
   */
  public class StraightCommand extends MoveCommand
  {
    double mSpeed;
    double mDistance;

    StraightCommand(double speed, double distance)
    {
      mSpeed = speed;
      mDistance = distance;
    }

    @Override
    public void Init()
    {
      leftEncoder.reset();
      rightEncoder.reset();
    }

    @Override
    public void Done()
    {

    }

    @Override
    public boolean Tick()
    {
      double leftDistance = leftEncoder.getDistance();
      double rightDistance = rightEncoder.getDistance();
      double averageDistance = (leftDistance + rightDistance) / 2;

      if (Math.abs(averageDistance) >= mDistance)
      {
        leftDriveMotor.set(0);
        rightDriveMotor.set(0);
        Done();
        return true;
      }
      TickCounter++;
      error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
                                            // if error < 0 right is ahead add -error to right
      correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

      resultantleftspeed = mSpeed - correctionFactor;
      resultantrightspeed = mSpeed + correctionFactor;
      if (resultantleftspeed > 1.0)
        resultantleftspeed = 1.0;
      if (resultantrightspeed > 1.0)
        resultantrightspeed = 1.0;
      if (resultantleftspeed < -1.0)
        resultantleftspeed = -1.0;
      if (resultantrightspeed < -1.0)
        resultantrightspeed = -1.0;

      leftDriveMotor.set(resultantleftspeed); // +err means left ahead, subtract from left speed
      rightDriveMotor.set(resultantrightspeed); // -err means right ahead, add -err to right speed
      return false; // not done yet
    }// Tick
  }// StraightCommand class

  /**
   * Turn one wheel for distance at speed
   * 
   * @author BM
   */
  public class TurnOneWheelCommand extends MoveCommand
  {
    double speedCMPerSec, distCM;
    boolean isRightWheel;

    public TurnOneWheelCommand(double speedCMPerSec, double distCM, boolean isRightWheel)
    {
      this.speedCMPerSec = speedCMPerSec;
      this.distCM = distCM;
      this.isRightWheel = isRightWheel;
    }

    @Override
    public void Init()
    {
      leftEncoder.reset();
      rightEncoder.reset();
      if (isRightWheel)
      {
        rightDriveMotor.set(speedCMPerSec);
        leftDriveMotor.set(0);
      }
      else
      {
        leftDriveMotor.set(speedCMPerSec);
        rightDriveMotor.set(0);
      }

    }

    @Override
    public boolean Tick()
    {
      double distanceTraveledByWheel;
      if (isRightWheel)
        distanceTraveledByWheel = rightEncoder.getDistance();
      else
        distanceTraveledByWheel = leftEncoder.getDistance();
      if (Math.abs(distanceTraveledByWheel) >= distCM)
        return true;
      else
        return false;
    }
    
    @Override
    public void Done()
    {
    
    }
  }

  /**
   * Follow path at speed until end.
   * 
   * @author EA
   *
   */
  public class TraceCommand extends MoveCommand
  {
    ArrayList<Vector2> mPath;
    double mSpeed = 0.0;
    int index = 0;
    double currentDegrees = 0;
    double wheelBaseWidthCM = 100;
    boolean rotationMode = true;
    Vector2 currentPos;
    double driveDist = 0;
    boolean doneDriving = true;

    public TraceCommand(ArrayList<Vector2> path, double speed)
    {
      mSpeed = speed;
      mPath = path;
    }
    
    @Override
    public boolean Tick()
    {
    	if (index == mPath.size() - 2) return true;
    	float mDistance = 0f;
    	Vector2 a = mPath.get(index);
    	Vector2 b = mPath.get(index + 1);
    	double degrees = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    	if (doneDriving && !rotationMode) {
    		rotationMode = true;
    		doneDriving = false;
    		double rot = degrees %= 360;
    	    double frac = rot / 360; // fractional amount
    	    index++;
    	    mDistance = (float) (-1 * frac * mDriveBaseCircumference);
    	} else if (Math.abs(currentDegrees - degrees) < 4) {
    		if (rotationMode) {
    			driveDist = Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
    			rotationMode = false;
    		}
    	}
    		
    		rotationMode = false; 
    	
    	if (rotationMode) {
    	  double leftDistance = leftEncoder.getDistance(); // get left encoder as normal distance
		  double rightDistance = rightEncoder.getDistance() * -1; // right wheel is going backwards so distance is * -1
		
		  double averageDistance = (leftDistance + rightDistance) / 2;
		
		  if (averageDistance >= mDistance)
		  {
		    leftDriveMotor.set(0);
		    rightDriveMotor.set(0);
		    currentDegrees = degrees;
		  }
		
		  error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
		                        // if error < 0 right is ahead add -error to right
		  correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error
		
		  leftDriveMotor.set(mSpeed - correctionFactor); // +err means left ahead, subtract from left speed
		  rightDriveMotor.set(mSpeed + correctionFactor); // -err means right ahead, add -err to right speed
    	} else {
    		// MOVE MODE
    		double leftDistance = leftEncoder.getDistance();
    	      double rightDistance = rightEncoder.getDistance();
    	      double averageDistance = (leftDistance + rightDistance) / 2;

    	      if (Math.abs(averageDistance) >= driveDist)
    	      {
    	        leftDriveMotor.set(0);
    	        rightDriveMotor.set(0);
    	        doneDriving = true;
    	      }
    	      TickCounter++;
    	      error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
    	                                            // if error < 0 right is ahead add -error to right
    	      correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

    	      resultantleftspeed = mSpeed - correctionFactor;
    	      resultantrightspeed = mSpeed + correctionFactor;
    	      if (resultantleftspeed > 1.0)
    	        resultantleftspeed = 1.0;
    	      if (resultantrightspeed > 1.0)
    	        resultantrightspeed = 1.0;
    	      if (resultantleftspeed < -1.0)
    	        resultantleftspeed = -1.0;
    	      if (resultantrightspeed < -1.0)
    	        resultantrightspeed = -1.0;

    	      leftDriveMotor.set(resultantleftspeed); // +err means left ahead, subtract from left speed
    	      rightDriveMotor.set(resultantrightspeed); // -err means right ahead, add -err to right speed
    	}
    	return false;
    }
    

    @Override
    public void Init()
    {
      leftEncoder.reset();
      rightEncoder.reset(); // clear encoder distances/ticks
    }

    @Override
    public void Done()
    {
    }

  }// TraceCommand

} // class MoveFunctions