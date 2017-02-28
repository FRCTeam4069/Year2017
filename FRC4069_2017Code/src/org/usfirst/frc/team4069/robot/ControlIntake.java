package org.usfirst.frc.team4069.robot;

import org.usfirst.frc.team4069.robot.Robot.InputSystem;

import edu.wpi.first.wpilibj.Talon;

public class ControlIntake
{
  private Talon intakeTalonFront;
  private Talon intakeTalonBack;

  private double frontSpeed = 0.0;
  private double backSpeed = 0.0;
  private int mEnabled = 0;
  private int mDebug = 0;
  private int mDirection = 1;

  private Robot mRobot;

  public ControlIntake(Robot robot)
  {
    mRobot = robot;
    intakeTalonFront = new Talon(IOMapping.INTAKE_FRONT_PWM_PORT);
    intakeTalonBack = new Talon(IOMapping.INTAKE_BACK_PWM_PORT);
    intakeTalonFront.set(0);
    intakeTalonBack.set(0);
  } // ControlIntake

  public void setIntakeSpeed(double speed)
  {
    frontSpeed = backSpeed = speed;
  }

  public void setBackIntakeSpeed(double spd)
  {
    backSpeed = spd;
  }

  public void setFrontIntakeSpeed(double spd)
  {
    frontSpeed = spd;
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
    intakeTalonFront.set(getFrontSpeed());
    intakeTalonBack.set(getBackSpeed());
  }

  public void Disable()
  {
    mEnabled = 0;
    intakeTalonFront.set(0);
    intakeTalonBack.set(0);
  }

  /**
   * true = run backward, false = run forward
   * 
   * @param reverseDirection
   */
  public void setReverseDirection(boolean reverseDirection)
  {
    mDirection = reverseDirection ? -1 : 1;
  }

  /**
   * Calculates front speed based on direction
   * 
   * @return
   */
  private double getFrontSpeed()
  {
    return frontSpeed * mDirection;
  }

  /**
   * Calculates back speed based on direction
   * 
   * @return
   */
  private double getBackSpeed()
  {
    return backSpeed * mDirection;
  }

  /**
   * Updates intake direction based on dpad
   */
  private void updateDirection()
  {
    if (InputSystem.Dpad_Down_Control_Stick_Pressed_Once)
    {
      // if dpad down is pressed once, reverse direction of intake
      setReverseDirection(true);
    }
    if (InputSystem.Dpad_Down_Control_Stick_Released_Once)
    {
      // if dpad down is released once, un-reverse direction of intake
      setReverseDirection(false);
    }
  }

  public void Tick()
  {
    updateDirection();
    if (Math.abs(mRobot.mRobotSpeed) >= 0.05 || InputSystem.Dpad_Up_Control_Stick || InputSystem.Dpad_Down_Control_Stick)
    {
      // if robot is moving or dpad up/down is pressed, enable intake
      if (mEnabled == 0)
      {
        Enable();
      }
    }
    else
    {
      // if robot is stationary and dpad up/down aren't pressed, disable intake
      if (mEnabled == 1)
      {
        Disable();
      }
    }
    if (mEnabled == 1)
    {
      intakeTalonFront.set(getFrontSpeed());
      intakeTalonBack.set(getBackSpeed());
    }
    else
    {
      intakeTalonFront.set(0);
      intakeTalonBack.set(0);
    }
  }
}
