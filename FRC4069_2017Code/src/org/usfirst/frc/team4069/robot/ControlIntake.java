package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Talon;

public class ControlIntake
{
  private Talon intakeTalonFront;
  private Talon intakeTalonBack;

  private double frontSpeed = 0.0;
  private double backSpeed = 0.0;
  private int mEnabled = 0;
  private int mDebug = 0;

  public ControlIntake()
  {
    intakeTalonFront = new Talon(IOMapping.INTAKE_FRONT_PWM_PORT);
    intakeTalonBack = new Talon(IOMapping.INTAKE_BACK_PWM_PORT);
    intakeTalonFront.set(0);
    intakeTalonBack.set(0);
  } //ControlIntake

  public void setIntakeSpeed(double speed)
  {
    frontSpeed = backSpeed = speed;
    intakeTalonFront.set(frontSpeed);
    intakeTalonBack.set(backSpeed);
  }

  public void setBackIntakeSpeed(double spd)
  {
    backSpeed = spd;
    intakeTalonBack.set(backSpeed);
  }

  public void setFrontIntakeSpeed(double spd)
  {
    frontSpeed = spd;
    intakeTalonFront.set(frontSpeed);
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
    intakeTalonFront.set(frontSpeed);
    intakeTalonBack.set(backSpeed);
  }

  public void Disable()
  {
    mEnabled = 0;
    intakeTalonFront.set(0);
    intakeTalonBack.set(0);
  }

  public void Tick()
  {
    if (mEnabled == 1)
    {
      intakeTalonFront.set(frontSpeed);
      intakeTalonBack.set(backSpeed);
    }
    else
    {
      intakeTalonFront.set(0);
      intakeTalonBack.set(0);
    }
  }
}
