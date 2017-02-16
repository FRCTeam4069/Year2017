package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class ControlFeed
{
  private Talon feedTalon;
  private StringBuilder sc_debug_info = new StringBuilder();
  private long mlastUpdateTime = 0;
  private double mWantedRPM = 3600;
  private int mEnabled = 0;
  private int mDebug = 0;
  private Robot mRobot;

  public ControlFeed(Robot robot)
  {
    feedTalon = new Talon(IOMapping.FEED_PWM_PORT);
    mlastUpdateTime = System.currentTimeMillis();
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
  }

  public void Disable()
  {
    mEnabled = 0;
  }

  public void Tick()
  {
    if (mRobot.mShooterController.runFeed) {
      feedTalon.set(1);
    }
  }
}
