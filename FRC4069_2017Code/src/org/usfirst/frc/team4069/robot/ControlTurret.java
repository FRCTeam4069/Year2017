package org.usfirst.frc.team4069.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Talon;

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

  public static final int turretEncoderMax = 7780;

  public static final int turretEncoderMin = -50;

  
  public static final int turretEncoderMidpoint = 4000;

  private int targetEncoderPosition;

  private double maxTargetPositionSpeed = 0.35;

  private boolean encoderTargetingEnabled = false;

  private boolean turretLimitSwitchEnabled;

  private boolean turretEncoderZeroed = false;
  private double turretEncoderPosition;

  private boolean autoTargetingEnabled = false; // true;

  public ControlTurret(Robot robot)
  {
    mRobot = robot;
    turretTalon = new CANTalon(IOMapping.TURRET_CANBUS_PORT);
    turretLimitSwitch = new DigitalInput(IOMapping.TURRET_LIMIT_SWITCH);

    // turretTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    // turretTalon.reverseSensor(false);
    // turretTalon.configEncoderCodesPerRev(4096); // Magnetic encoder ticks per revolution
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
  private double handleAutoTargeting()
  {
    double spd = 0;
    if (mRobot.vision_processor_instance.cregions.mTargetVisible == 1)
    {
      double xpos = mRobot.vision_processor_instance.cregions.mXGreenLine;
      double humanOffset = mRobot.controlStick.getRawAxis(IOMapping.CONTROL_RIGHT_X_AXIS);

      double addABit = Lerp(-16, 16, -1, 1, humanOffset);

      double adjustedXPos = xpos; // + addABit; // allow user to change xposition of target by + or - 5% (-16 to +16) in a 320 x range

      if (adjustedXPos < 160)
      {
        spd = Lerp(.25, .025, 0, 160, adjustedXPos);
      }
      if (xpos > 160) //adjustedXPos)
      {
        spd = Lerp(-.25, -.025, 320, 160, adjustedXPos);
      }
    }
    return spd;
  }

  /**
   * Returns motorValue with safety limits applied
   * 
   * @param motorValue
   * @return
   */
  private double imposeEncoderSafetyLimits(double motorValue)
  {
    
    double safeMotorValue = 0.0;
    if (turretEncoderPosition >= turretEncoderMin || turretLimitSwitchEnabled)
    {
      if (motorValue > 0)
      {
        safeMotorValue = motorValue;
      }
      else
      {
        safeMotorValue = 0;
      }
    }
    else if (turretEncoderPosition <= turretEncoderMax)
    {
      if (motorValue < 0)
      {
        safeMotorValue = motorValue;
      }
      else
      {
        safeMotorValue = 0;
      }
    }
    else
    {
      safeMotorValue = motorValue;
    }
    return safeMotorValue;
  }

  // TODO Add safety checks for cantalons encoder position so we don't destroy wiring by doing 360's
  // Init with limit switch sensor to properly 'zero' out encoder
  public void Tick()
  {
    // toggle auto targeting with start button
    if (Robot.InputSystem.Start_Button_Control_Stick_Once)
    {
//      autoTargetingEnabled = !autoTargetingEnabled;
    }

    // test code for encoder targeting, moves turret to midpoint if a is pressed
    if (Robot.InputSystem.A_Button_Control_Stick)
    {
      //setTargetEncoderPosition(turretEncoderMidpoint);
    }

    // read some values
    turretLimitSwitchEnabled = turretLimitSwitch.get();
    turretEncoderPosition = turretTalon.getPosition();

    if (turretEncoderZeroed)
    {
      double mv = handleAutoTargeting();
      //mv = imposeEncoderSafetyLimits(mv);
      if ((turretEncoderPosition < 0 ) && (mv > 0))  //+mv means towards limit, - means encoder towards limit
      {
        mv = 0;
      }
      if ((turretEncoderPosition > turretEncoderMax)&&(mv < 0))
      {
        mv = 0;
      }
      turretTalon.set(mv);
    }/*
      
      
      double maxSpeed = 0.5; // scaling constant for motor speed
      double motorValue = 0; // input to the turret motor
      double driverStick = mRobot.controlStick.getAxis(AxisType.kY) * maxSpeed;
      if (Math.abs(driverStick) > 0.1)
      { // if joystick is down, exit out of encoder targeting routine
        encoderTargetingEnabled = false;
      }
      if (autoTargetingEnabled)
      { // if auto targeting enabled, apply auto targeting to motorValue
        motorValue = handleAutoTargeting();
      }
      else if (encoderTargetingEnabled)
      { // otherwise, if encoder targeting is on, apply encoder targeting to motorValue
        double error = (turretEncoderPosition - targetEncoderPosition) / 2000;
        motorValue = error > 0 ? Math.min(maxTargetPositionSpeed, error) : Math.max(-maxTargetPositionSpeed, error);
        motorValue = lpf.calculate(motorValue); // apply lowpassfilter for smoother movement
      }
      else
      { // otherwise just let the turret be controlled manually
        motorValue = mRobot.controlStick.getAxis(AxisType.kY) * maxSpeed;
        // decrease speed as turret gets closer to min/max so turret does not slide past limits
        if ((motorValue > 0 && turretEncoderPosition < turretEncoderMidpoint) || (motorValue < 0 && turretEncoderPosition > turretEncoderMidpoint))
        {
          motorValue *= Lerp(1, 0, 0, turretEncoderMin - turretEncoderMidpoint, Math.abs(turretEncoderPosition - turretEncoderMidpoint));
        }
        motorValue = lpf.calculate(motorValue); // also apply lowpassfilter here
      }
      motorValue = imposeEncoderSafetyLimits(motorValue); // finally impose hard safety limits on motorValue
      turretTalon.set(-motorValue);
    }*/
    else
    { // here if turret is not yet zeroed, must zero first
      DoTurretZeroStep();
    } // else turret not yet zeroed
  } // Tick()

  void DoTurretZeroStep()
  {
    if (turretLimitSwitchEnabled == false) // if limit NOT pressed
    {
      turretTalon.set(0.1); // crawl toward limit switch
    }
    else
    {
      turretTalon.set(0);
      while (turretLimitSwitchEnabled == true)
      {
        turretTalon.set(-0.1); // go until off limit switch
        turretLimitSwitchEnabled = turretLimitSwitch.get();
      }
      turretTalon.set(0); // turn off talon
      turretTalon.setEncPosition(0); // reset encoder ticks
      turretEncoderZeroed = true;
    }
  }

}// class ControlTurret