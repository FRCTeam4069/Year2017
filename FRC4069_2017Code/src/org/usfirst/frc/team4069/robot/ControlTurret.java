package org.usfirst.frc.team4069.robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.DigitalInput;

public class ControlTurret
{
  private CANTalon turretTalon;
  public DigitalInput turretLimitSwitch;

  private StringBuilder sc_debug_info = new StringBuilder();
  private long mlastUpdateTime = 0;
  private double mWantedRPM = 3600;
  private int mEnabled = 0;
  private int mDebug = 0;
  private Robot mRobot;
  private LowPassFilter lpf = new LowPassFilter(100);

  public static final int turretEncoderMax = 8466;

  public static final int turretEncoderMin = -50;

  public static final int turretEncoderMidpoint = 4000;

  private int targetEncoderPosition;

  private double maxTargetPositionSpeed = 0.35;

  private boolean encoderTargetingEnabled = false;

  private boolean turretLimitSwitchEnabled;

  private boolean turretEncoderZeroed = false;
  private double turretEncoderPosition;

  private boolean autoTargetingEnabled = true; // true;

  public ControlTurret(Robot robot)
  {
    mRobot = robot;
    turretTalon = new CANTalon(IOMapping.TURRET_CANBUS_PORT);
    turretLimitSwitch = new DigitalInput(IOMapping.TURRET_LIMIT_SWITCH);
    mlastUpdateTime = System.currentTimeMillis();
  } // ShooterControl init

  public double GetShooterPosition()
  {
    return turretTalon.getPosition();
  }

  public void EnableDebug()
  {
    mDebug = 1;
  }

  public void DisableDebug()
  {
    mDebug = 0;
  }

  // --------------------------------------------------------------------------------------------
  /**
   * Linear Interpolation, given a value x2 between x0 and x1 calculate position between Y0 and Y1
   * 
   * @author EA
   */
  public double Lerp(double y0, double y1, double x0, double x1, double x2)
  {
    double y2 = y0 * (x2 - x1) / (x0 - x1) + y1 * (x2 - x0) / (x1 - x0);
    return y2;
  }

  public void Enable()
  {
    mEnabled = 1;
  }

  public void Disable()
  {
    mEnabled = 0;
  }

  public void setTargetEncoderPosition(int targetEncoderPosition)
  {
    this.targetEncoderPosition = targetEncoderPosition;
    encoderTargetingEnabled = true;
  }

  public int isTurretTargeted()
  {
    double xpos = mRobot.vision_processor_instance.cregions.mXGreenLine;
    if ((xpos >= 155) && (xpos <= 175))
      return 1;
    else
      return 0;
  }

  /**
   * Returns motor value for auto targeting
   * 
   * @return
   */
  private double getAutoTargetingMotorValue()
  {
    if (autoTargetingEnabled == false) // if auto-targeting is disabled, return 0.0 for speed
      return 0.0;

    double spd = 0;
    if (mRobot.vision_processor_instance.cregions.mTargetVisible == 1)
    {
      double xpos = mRobot.vision_processor_instance.cregions.mXGreenLine;
      // double humanOffset = mRobot.controlStick.getRawAxis(IOMapping.CONTROL_RIGHT_X_AXIS);
      //
      // double addABit = Lerp(-16, 16, -1, 1, humanOffset);
      //
      // double adjustedXPos = xpos; // + addABit; // allow user to change xposition of target by + or - 5% (-16 to +16) in a 320 x range

      if (xpos < 160)
      {
        spd = Lerp(.25, .025, 0, 160, xpos);
      }
      if (xpos > 160) // adjustedXPos)
      {
        spd = Lerp(-.25, -.025, 320, 160, xpos);
      }
    }
    return spd;
  }

  // NOTE: Motor movement < 0 means AWAY from switch
  // Motor movement > 0 means TOWARD switch
  // Note mv > 0 moves turret towards limit switch
  // mv < 0 moves away from limit switch
  // Position is backwards from this,
  // HIGH values are away from switch, low values are towards switch

  boolean isOkToMoveTurret(double mv)
  {
    if (mv < 0 && turretEncoderPosition >= turretEncoderMax)
    {
      return false;
    }
    if (mv > 0 && turretEncoderPosition <= turretEncoderMin)
    {
      return false;
    }
    return true; // false if at a limit and trying to move more past limit
  } // isOkToMoveTurret

  // read joystick and return a manual motor movement value
  double getManualTurretMovementValue()
  {
    double motorValue = 0.0; // return wanted manual turret movement

    double maxSpeed = 0.5; // scaling constant for motor speed
    double driverStick = mRobot.controlStick.getAxis(AxisType.kY) * maxSpeed * -1;

    if (Math.abs(driverStick) > 0.1)
    { // if joystick is down, exit out of encoder targeting routine
      encoderTargetingEnabled = false;
    }

    if (encoderTargetingEnabled)
    { // if encoder targeting is on, apply encoder targeting to motorValue
      double error = (turretEncoderPosition - targetEncoderPosition) / 2000;
      motorValue = error > 0 ? Math.min(maxTargetPositionSpeed, error) : Math.max(-maxTargetPositionSpeed, error);
      motorValue = lpf.calculate(motorValue); // apply lowpassfilter for smoother movement
    }
    else
    { // otherwise just let the turret be controlled manually
      motorValue = mRobot.controlStick.getAxis(AxisType.kY) * maxSpeed * -1;

      // decrease speed as turret gets closer to min/max so turret does not slide past limits
      /*if ((motorValue > 0 && turretEncoderPosition < turretEncoderMidpoint) || (motorValue < 0 && turretEncoderPosition > turretEncoderMidpoint))
      {
        motorValue *= Lerp(1, 0, 0, turretEncoderMidpoint - turretEncoderMin, Math.abs(turretEncoderPosition - turretEncoderMidpoint));
        motorValue = lpf.calculate(motorValue); // also apply lowpassfilter here
      }*/
      if (motorValue > 0 && turretEncoderPosition < turretEncoderMidpoint)
      {
        motorValue *= Lerp(1, 0, 0, turretEncoderMidpoint - turretEncoderMin, Math.abs(turretEncoderPosition - turretEncoderMidpoint));
        motorValue = lpf.calculate(motorValue); // also apply lowpassfilter here
      }
      else if (motorValue < 0 && turretEncoderPosition > turretEncoderMidpoint)
      {
        motorValue *= Lerp(1, 0, 0, turretEncoderMax - turretEncoderMidpoint, Math.abs(turretEncoderPosition - turretEncoderMidpoint));
        motorValue = lpf.calculate(motorValue); // also apply lowpassfilter here
      }
    }
    return motorValue;
  } // getManualTurretMovementValue

  // --------------------------------------------------------------------------
  // Main Tick for turret, lets keep this as few a lines as possible!
  //
  public void Tick()
  {
    turretLimitSwitchEnabled = turretLimitSwitch.get();
    turretEncoderPosition = turretTalon.getPosition();

    if (Robot.InputSystem.Start_Button_Control_Stick_Once)
    {
      autoTargetingEnabled = !autoTargetingEnabled;
    }

    if (turretEncoderZeroed == false)
    {
      DoTurretZeroStep();
      return;
    }

    double motorMovementValue = 0.0; // default to no movement of motor

    if (autoTargetingEnabled == true)
      motorMovementValue = getAutoTargetingMotorValue(); // if auto-target on, get value from auto-target
    else
      motorMovementValue = getManualTurretMovementValue(); // if auto-target Disabled, read manual joystick

    if (isOkToMoveTurret(motorMovementValue) == false) // FINAL check to verify its ok to move turret direction it wants to go...
      motorMovementValue = 0; // stop movement if not ok to move

    turretTalon.set(motorMovementValue); // Note: No Lowpass here! auto-target does not want one, do manual in manual calcs
  } // Tick()

  // -----------------------------------------------------------------------
  // DoTurretZeroStep : Called if turret is not yet zeroed at startup.
  // slowly moves turret 0.1 motor value towards limit switch >0==towards switch
  // once hits, slowly moves away from switch (-0.1) till switch opens
  //
  void DoTurretZeroStep()
  {
    if (turretLimitSwitchEnabled == false) // if limit NOT pressed
    {
      turretTalon.set(0.2); // crawl toward limit switch
    }
    else
    { // now lets move off limit switch...
      turretTalon.set(0); // turn off turret movement
      while (turretLimitSwitchEnabled == true) // while limit switch is pressed
      {
        turretTalon.set(-0.2); // go until off limit switch
        turretLimitSwitchEnabled = turretLimitSwitch.get();
      }
      turretTalon.set(0); // turn off turret
      turretTalon.setEncPosition(0); // reset encoder ticks
      turretEncoderZeroed = true;
    }
  }// DoTurretZeroStep

}// class ControlTurret