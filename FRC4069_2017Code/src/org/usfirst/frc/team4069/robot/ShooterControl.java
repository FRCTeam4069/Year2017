package org.usfirst.frc.team4069.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;


/**
 * ShooterControl : Uses a CANTalon with a encoder and a PID controller to
 * set the RPM's on the shooter motor to a fixed value and hold them there
 * @author EA
 * Create a instance of this class, then call its tick method periodically
 */
public class ShooterControl
{
  private CANTalon shooterCANTalon;
  private StringBuilder sc_debug_info = new StringBuilder();
  private long mlastUpdateTime = 0;
  private double mWantedRPM = 600;
  private int mEnabled = 0;
  private int mDebug = 1;

  public ShooterControl()
  {
    shooterCANTalon = new CANTalon(IOMapping.SHOOTER_CANBUS_PORT);
    shooterCANTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    shooterCANTalon.reverseSensor(false);
    shooterCANTalon.configEncoderCodesPerRev(4096); // Magnetic encoder ticks
                                                    // per revolution 2^12
    // Set peak and nominal outputs 12 volts means full power
    shooterCANTalon.configNominalOutputVoltage(+0.0f, -0.0f);
    shooterCANTalon.configPeakOutputVoltage(12.0f, 0.0f);

    // set closed loop gains in slot 0
    shooterCANTalon.setProfile(0);
    shooterCANTalon.setF(0.1097);
    shooterCANTalon.setP(0.22);
    shooterCANTalon.setI(0);
    shooterCANTalon.setD(0);
    shooterCANTalon.changeControlMode(TalonControlMode.Speed);
    mlastUpdateTime = System.currentTimeMillis();
  } // ShooterControl init

  public double GetShooterPosition()
  {
    return shooterCANTalon.getPosition();
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

  public void SetWantedRPM(double rpm)
  {
    mWantedRPM = rpm;
  }

  /**
   * ShooterTick : Should be called with all the other update functions after inputs have been read/updated
   * IF proper button held down, will call set speed to set to wanted RPM's
   */
  public void Tick(Joystick _joy)
  {
    long timesincelastupdate = System.currentTimeMillis() - mlastUpdateTime;
    /* get gamepad axis */
    double leftYstick = _joy.getAxis(AxisType.kY);
    double motorOutput = shooterCANTalon.getOutputVoltage() / shooterCANTalon.getBusVoltage();
    /* prepare line to print */
    sc_debug_info.append("\tout:");
    sc_debug_info.append(motorOutput);
    sc_debug_info.append("\tspd:");
    sc_debug_info.append(shooterCANTalon.getSpeed());

    if (_joy.getRawButton(1))
    {
      /* Speed mode */
      double targetSpeed = leftYstick * 1500.0; /* 1500 RPM in either direction */
      shooterCANTalon.changeControlMode(TalonControlMode.Speed);
      shooterCANTalon.set(targetSpeed); /* 1500 RPM in either direction */

      /* append more signals to print when in speed mode. */
      sc_debug_info.append("\terr:");
      sc_debug_info.append(shooterCANTalon.getClosedLoopError());
      sc_debug_info.append("\ttrg:");
      sc_debug_info.append(targetSpeed);
    }
    else
    {
      /* Percent voltage mode */
      shooterCANTalon.changeControlMode(TalonControlMode.PercentVbus);
      shooterCANTalon.set(leftYstick);
    }

    if (timesincelastupdate > 1000)
    {
      System.out.println(sc_debug_info.toString());
      mlastUpdateTime = System.currentTimeMillis();
    }
    sc_debug_info.setLength(0);
  }//ShooterTick
}//ShooterControl
