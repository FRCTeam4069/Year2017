package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class ControlElevator
{
  private Talon elevatorTalon;
  private int mEnabled = 0;
  private int mDebug = 0;
  private double mSpeed=0.0; // default speed used by elevator
  private double mSecondSpeed=0.0; // secondary speed which can be toggled
  
  private boolean useSecondSpeed = false; // should second or first speed be used
  
  public ControlElevator()
  {
    elevatorTalon = new Talon(IOMapping.ELEVATOR_PWM_PORT);
    elevatorTalon.set(0);
  } // controlElevator

  /**
   * Set main speed of elevator
   * @param spd
   */
  public void setElevatorSpeed(double spd)
  {
    mSpeed=spd;
  }
  
  /**
   * Set secondary speed of elevator
   * @param spd
   */
  public void setElevatorSecondSpeed(double spd){
	  mSecondSpeed=spd;
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
    	// if back button is pressed once, toggle between main/second speed
    	if(Robot.InputSystem.Back_Button_Control_Stick_Once){
    		useSecondSpeed = !useSecondSpeed;
    	}
    	// if second speed is toggled use second instead of main speed
    	if(useSecondSpeed){
    		elevatorTalon.set(mSecondSpeed);
    	}
    	else{ // else just use main speed
    		elevatorTalon.set(mSpeed);
    	}
    }
    else
      elevatorTalon.set(0);
  }
}
