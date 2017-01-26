package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class WinchControl
{
  private Talon winchTalon;
  private StringBuilder sc_debug_info = new StringBuilder();
  private long mlastUpdateTime = 0;
  private double mWantedRPM = 3600;
  private int mEnabled = 0;
  private int mDebug = 0;

  public WinchControl()
  {
    winchTalon = new Talon(9);
    mlastUpdateTime = System.currentTimeMillis();
  } // ShooterControl init

  public double GetShooterPosition()
  {
    return winchTalon.getPosition();
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
  }

  public void Disable()
  {
    mEnabled = 0;
  }

  public void Tick()
  {
    if (Robot.InputSystem.B_Button_Control_Stick && !Robot.InputSystem.Y_Button_Control_Stick)
    {
      winchTalon.set(-1);
    }
    else if (!Robot.InputSystem.B_Button_Control_Stick && Robot.InputSystem.Y_Button_Control_Stick)
    {
      winchTalon.set(1);
    }
    else
    {
      winchTalon.set(0);
    }
  }
}
