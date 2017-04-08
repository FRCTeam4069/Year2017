package org.usfirst.frc.team4069.robot;

public class MoveCommandStraightBadly extends MoveCommand
{
  double mSpeed;
  double mDistance;
  private ControlMove mControlMove;

  MoveCommandStraightBadly(ControlMove ctrlmove, double speed, double distance)
  {
	System.out.println("Created move straight command badly");
    mControlMove = ctrlmove;
    mSpeed = speed;
    mDistance = distance;
  }

  @Override
  public void Init()
  {
	System.out.println("Initialized move straight command badly");
    mControlMove.leftEncoder.reset();
    mControlMove.rightEncoder.reset();
  }

  @Override
  public boolean Tick()
  {
	System.out.println("Ticked move straight command badly, speed: " + mSpeed);
    double leftDistance = ((mControlMove.leftEncoder.get() * ControlMove.CM_PER_TICK) / ControlMove.CM_PER_INCH) / 12;
    double rightDistance = ((mControlMove.rightEncoder.get() * ControlMove.CM_PER_TICK) / ControlMove.CM_PER_INCH) / 12;
    double averageDistance = (leftDistance + rightDistance) / 2;
    if (Math.abs(averageDistance) >= mDistance)
    {
      mControlMove.leftDriveMotor.set(0);
      mControlMove.rightDriveMotor.set(0);
      return true;
    }
    mControlMove.leftDriveMotor.set(-mSpeed); // +err means left ahead, subtract from left speed
    mControlMove.rightDriveMotor.set(mSpeed*1.35); // -err means right ahead, add -err to right speed
    return false; // not done yet
  }// Tick
}
