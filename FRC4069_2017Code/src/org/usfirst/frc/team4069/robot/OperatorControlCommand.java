package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Joystick.AxisType;

public class OperatorControlCommand extends MoveCommand
{
  double mDriverRobotSpeed = 0.0;
  double mDriverRobotTurnDirection = 0.0;
  ControlMove mControlMove;
  
  public OperatorControlCommand(ControlMove cntmv)
  {
    mControlMove = cntmv;
  }

  @Override
  public void Init()
  {

  }

  @Override
  public boolean Tick()
  {
    // Combined speed of the drive motors
    double combined = mControlMove.driverStick.getRawAxis(3) * -1 + mControlMove.driverStick.getAxis(AxisType.kZ);
    mDriverRobotSpeed = mControlMove.leftDriveMotorLowPassFilter.calculate(combined);
    // Calculate robot turn direction from the left drive joystick's x-axis
    // (left-right)
    mDriverRobotTurnDirection = mControlMove.rightDriveMotorLowPassFilter.calculate(mControlMove.driverStick.getAxis(AxisType.kX));

    // Last move robot. Let the robotdrive class handle the driving aspect of
    // the robot
    mControlMove.mRobotDrive.arcadeDrive(mDriverRobotSpeed, mDriverRobotTurnDirection); // move robot
    return false;
  }

  @Override
  public void Done()
  {
  }
}//class OperatorControlCommand
