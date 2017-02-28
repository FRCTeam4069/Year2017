package org.usfirst.frc.team4069.robot;

public class MoveCommandStop extends MoveCommand
{
  private ControlMove mControlMove;

  MoveCommandStop(ControlMove ctrlmove)
  {
    mControlMove = ctrlmove;
  }

  @Override
  public void Init()
  {
    mControlMove.leftDriveMotor.set(0);
    mControlMove.rightDriveMotor.set(0);
  }

  @Override
  public boolean Tick()
  {
    System.out.println("move stop tick");
    mControlMove.leftDriveMotor.set(0);
    mControlMove.rightDriveMotor.set(0);
    return true;
  }
}
