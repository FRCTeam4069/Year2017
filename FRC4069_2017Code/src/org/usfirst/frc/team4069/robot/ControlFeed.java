package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class ControlFeed
{
  private Talon feedTalon;
  private int mEnabled = 0;
  private int mDebug = 0;
  private Robot mRobot;
  private double mFeedSpeed = 0.8;

  public ControlFeed(Robot robot)
  {
    feedTalon = new Talon(IOMapping.FEED_PWM_PORT);
    mRobot = robot;
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
    feedTalon.set(mFeedSpeed);
  }

  public void Disable()
  {
    mEnabled = 0;
    feedTalon.set(0);
  }

  public void setFeedSpeed(double fs)
  {
    mFeedSpeed = fs;
    if (mEnabled == 1) // if enabled, change right away
    {
      feedTalon.set(mFeedSpeed);
    }
  }

  public void Tick()
  {
	// Enable feed if shooter is moving, otherwise disable feed
	
    if (mEnabled == 1)
    {
      // if(Robot.InputSystem.Y_Button_Driver_Stick || Robot.InputSystem.Y_Button_Control_Stick){
	  	if(Math.abs(mRobot.mElevatorSpeed) > 0){
	    	if(mRobot.mShooterTargetRPM > 0){
				feedTalon.set(mFeedSpeed);
			}
			else if(mRobot.mShooterTargetRPM == 0){
				feedTalon.set(-mFeedSpeed);
			}
	  	}
	  	else{
	  		feedTalon.set(0);
	  	}
      // }
      // else{
      // feedTalon.set(0);
      // }
    }
    else
    {
      feedTalon.set(0);
    }
  }
}
