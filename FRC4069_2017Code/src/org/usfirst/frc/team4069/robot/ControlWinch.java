package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class ControlWinch
{
  private Talon winchTalon;
  private int mEnabled = 0;
  private int mDebug = 0;
  private double mSpeed = 1;

  public ControlWinch()
  {
    winchTalon = new Talon(IOMapping.WINCH_PWN_PORT);
    winchTalon.set(0);
  } // ControlWinch

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
    winchTalon.set(mSpeed);
  }

  public void Disable()
  {
    mEnabled = 0;
    winchTalon.set(0);
  }

  public void Tick()
  {
    if (mEnabled == 1)
    {
      if (Robot.InputSystem.Y_Button_Driver_Stick){
        winchTalon.set(mSpeed);
      }
      else if(Robot.InputSystem.A_Button_Driver_Stick){
    	  winchTalon.set(-mSpeed);
      }
      else
      {
        winchTalon.set(0);
      }
    }
    else
    {
      winchTalon.set(0);
    }
  }
}
