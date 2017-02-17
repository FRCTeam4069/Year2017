package org.usfirst.frc.team4069.robot;

public class MoveCommandTurnOneWheel extends MoveCommand
{
  double speedCMPerSec, distCM;
  boolean isRightWheel;
  private ControlMove mControlMove;
  
  public MoveCommandTurnOneWheel(ControlMove ctrlmove,double speedCMPerSec, double dist, boolean isRightWheel)
  {
    mControlMove = ctrlmove;
    speedCMPerSec = speedCMPerSec;
    distCM = dist;
    isRightWheel = isRightWheel;
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
    System.out.println("Turnonewheel tick");
    double distanceTraveledByWheel;
    if (isRightWheel)
      distanceTraveledByWheel = mControlMove.rightEncoder.getDistance();
    else
      distanceTraveledByWheel = mControlMove.leftEncoder.getDistance();
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
