package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class ControlElevator
{
  private Talon elevatorTalon;
  private int mEnabled = 0;
  private int mDebug = 0;
  private double mSpeed=0.0;
  
  public ControlElevator()
  {
    elevatorTalon = new Talon(IOMapping.ELEVATOR_PWM_PORT);
    elevatorTalon.set(0);
  } // controlElevator


  public void setElevatorSpeed(double spd)
  {
    mSpeed=spd;
    elevatorTalon.set(mSpeed);
  }
  
  public void EnableDebug()
  {
    mDebug = 1;
  }

  public void DisableDebug()
  {
    mDebug = 0;
  }

  public void Enable()
  {
    mEnabled = 1;
    elevatorTalon.set(mSpeed);
  }

  public void Disable()
  {
    mEnabled = 0;
    elevatorTalon.set(0);
  }

  public void Tick()
  {
    if (mEnabled==1)
    {
      elevatorTalon.set(mSpeed);
    }
    else
      elevatorTalon.set(0);
  }
}
