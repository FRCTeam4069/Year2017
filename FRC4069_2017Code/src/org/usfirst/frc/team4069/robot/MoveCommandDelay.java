package org.usfirst.frc.team4069.robot;

public class MoveCommandDelay extends MoveCommand
{

  private int milliseconds;

  private long startTime;
  ControlMove mControlMove;

  public MoveCommandDelay(ControlMove ctrlmove, int milliseconds)
  {
    mControlMove = ctrlmove;
    this.milliseconds = milliseconds;
  }

  @Override
  public void Init()
  {
    startTime = System.currentTimeMillis();
  }

  // For + degrees left wheel goes forward, right wheel goes backwards
  @Override
  public boolean Tick()
  {
    System.out.println("Delay tick");
    if ((int) (System.currentTimeMillis() - startTime) >= milliseconds)
    {
      return true;
    }
    return false;
  }// Tick

}
