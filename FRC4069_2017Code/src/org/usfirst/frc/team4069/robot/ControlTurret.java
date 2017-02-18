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

  
  
  private boolean turretEncoderZeroed = true;

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

  public int isTurretTargeted()
  {
    double xpos = mRobot.vision_processor_instance.cregions.mXGreenLine;
    if ((xpos >= 155) && (xpos <= 175))
      return 1;
    else
      return 0;
  }

  // TODO Add safety checks for cantalons encoder position so we don't destroy wiring by doing 360's
  // Init with limit switch sensor to properly 'zero' out encoder
  public void Tick()
  {
    // return;
    /*
     * if (turretEncoderZeroed) { if (Robot.InputSystem.Turret_Limit_Switch) { turretTalon.set(0); } else {
     */
    if (mRobot.vision_processor_instance.cregions.mTargetVisible == 1)
    {
      double xpos = mRobot.vision_processor_instance.cregions.mXGreenLine;

      // if (mEnabled > 0)
      // {
      if (xpos < 160)
      {
        double spd = Lerp(.25, .025, 0, 160, xpos);
        turretTalon.set(spd); // .15);
      }
      if (xpos > 160)
      {
        double spd = Lerp(-.25, -.025, 320, 160, xpos);
        turretTalon.set(spd);
      }
      if ((xpos >= 155) && (xpos <= 165))
      {
        // turretTalon.set(0);
      }
    }
    else
    {
      turretTalon.set(mRobot.controlStick.getAxis(AxisType.kY)); //
    }
    // }
    // else
    /// {
    // turretTalon.set(0); // if not enabled, set turret speed to zero
    // }
    // } // if limit switch NOT hit
    // } // if turret zeroed
    // else
    // { // here if turret is not yet zeroed, must zero first
    // if (Robot.InputSystem.Turret_Limit_Switch == false) // if limit NOT pressed
    // {
    // turretTalon.set(0.015); // crawl toward limit switch
    // }
    // else
    // {
    // turretTalon.set(0);
    // while (Robot.InputSystem.Turret_Limit_Switch == true)
    // {
    // turretTalon.set(-0.01); // go until off limit switch
    // }
    // turretTalon.set(0); // turn off talon
    // turretTalon.reset(); // reset encoder ticks
    // }
    // } // else turret not yet zeroed
  } // Tick()
}// class ControlTurret