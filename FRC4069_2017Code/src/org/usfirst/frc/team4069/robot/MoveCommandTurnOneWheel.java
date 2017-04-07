package org.usfirst.frc.team4069.robot;

public class MoveCommandTurnOneWheel extends MoveCommand
{
  double speedCMPerSec, distCM;
  boolean isRightWheel;
  private ControlMove mControlMove;

  public MoveCommandTurnOneWheel(ControlMove ctrlmove, double speedCMPerSec, double dist, boolean isRightWheel)
  {
    mControlMove = ctrlmove;
    this.speedCMPerSec = speedCMPerSec;
    distCM = dist;
    this.isRightWheel = isRightWheel;
  }

  @Override
  public void Init()
  {
    mControlMove.leftEncoder.reset();
    mControlMove.rightEncoder.reset();
    if (isRightWheel)
    {
      mControlMove.rightDriveMotor.set(speedCMPerSec);
      mControlMove.leftDriveMotor.set(0);
    }
    else
    {
      mControlMove.leftDriveMotor.set(speedCMPerSec);
      mControlMove.rightDriveMotor.set(0);
    }

  }

  @Override
  public boolean Tick()
  {
    //Log.Log("Turnonewheel tick");
    double distanceTraveledByWheel;
    if (isRightWheel)
      distanceTraveledByWheel = mControlMove.rightEncoder.getDistance();
    else
      distanceTraveledByWheel = mControlMove.leftEncoder.getDistance();
    System.out.println(distCM + " and " + distanceTraveledByWheel);
    if (isRightWheel)
    {
      mControlMove.rightDriveMotor.set(speedCMPerSec);
      mControlMove.leftDriveMotor.set(0);
    }
    else
    {
      mControlMove.leftDriveMotor.set(speedCMPerSec);
      mControlMove.rightDriveMotor.set(0);
    }
    if (Math.abs(distanceTraveledByWheel) >= distCM)
      return true;
    else
      return false;
  }
}
