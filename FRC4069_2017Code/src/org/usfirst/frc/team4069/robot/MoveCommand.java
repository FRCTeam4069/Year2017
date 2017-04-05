package org.usfirst.frc.team4069.robot;

public abstract class MoveCommand
{
  public MoveCommand()
  {
  }

  public abstract boolean Tick();

  public abstract void Init();
}
