package org.usfirst.frc.team4069.robot;

import org.usfirst.frc.team4069.robot.Robot.InputSystem;

import edu.wpi.first.wpilibj.Talon;

public class ControlElevator
{
  private Talon elevatorTalon;
  private int mEnabled = 0;
  private int mDebug = 0;
  private int mDirection = 1; // 1 for forwards, -1 for backwards
  private double mSpeed = 0.0; // default speed used by elevator
  private double mSecondSpeed = 0.0; // secondary speed which can be toggled
  private boolean useSecondSpeed = false; // should second or first speed be
  // used
  private Robot mRobot;

  public ControlElevator(Robot robot)
  {
    mRobot = robot;
    elevatorTalon = new Talon(IOMapping.ELEVATOR_PWM_PORT);
    elevatorTalon.set(0);
  } // controlElevator

  /**
   * Set main speed of elevator
   * 
   * @param spd
   */

  public void setElevatorSpeed(double spd)
  {
    mSpeed = spd;
  }

  /**
   * Set secondary speed of elevator
   * 
   * @param spd
   */
  public void setElevatorSecondSpeed(double spd)
  {
    mSecondSpeed = spd;
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
    elevatorTalon.set(getElevatorSpeed());
  }

  public void Disable()
  {
    mEnabled = 0;
    elevatorTalon.set(0);
  }

  /**
   * true = run backwards, false = run forwards
   * 
   * @param reverseDirection
   */
  public void setReverseDirection(boolean reverseDirection)
  {
    mDirection = reverseDirection ? -1 : 1;
  }

  /**
   * Calculates elevator speed based on direction
   * 
   * @return
   */
  private double getElevatorSpeed()
  {
    // if second speed is toggled use second instead of main speed
    if (useSecondSpeed)
    {
      return mSecondSpeed * mDirection * -1;
    }
    else
    { // else just use main speed
      return mSpeed * mDirection * -1;
    }
  }

  /**
   * Updates elevator direction based on dpad
   */
  private void updateDirection()
  {
    if (Math.abs(InputSystem.LT_Button_Control_Stick) > 0.1)
    {
      // if lt button is pressed, reverse direction of elevator
      setReverseDirection(true);
    }
    else
    {
      setReverseDirection(false);
    }
  }

  public void Tick()
  {

    updateDirection(); // if back button is pressed once, toggle between
    // main/second speed
    if (Robot.InputSystem.Back_Button_Control_Stick_Once)
    {
      useSecondSpeed = !useSecondSpeed;
    }
    if (InputSystem.RT_Button_Control_Stick_Pressed_Once)
    {
      if (mEnabled == 1)
      {
        Disable();
      }
      else
      {
        Enable();
      }
    }

    double speed = 0;

    if (mEnabled == 1)
    {
      speed = getElevatorSpeed();
    }

    mRobot.mElevatorSpeed = speed;

    elevatorTalon.set(speed);
  }
}
