package org.usfirst.frc.team4069.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;


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
  private double mWantedRPM = 3600;
  private int mEnabled = 0;
  private int mDebug = 0;

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
  public void ShooterTick()
  {
    long timesincelastupdate = System.currentTimeMillis() - mlastUpdateTime;

    if (timesincelastupdate > 1000)
    {
      if (mDebug == 1)
      {
        System.out.println(sc_debug_info.toString());
        sc_debug_info.setLength(0);
      }
      mlastUpdateTime = System.currentTimeMillis();
    } //if time > 1 second

    if (Robot.InputSystem.A_Button_Control_Stick)
      mEnabled=1;
    else
      mEnabled=0;
        
    double motorOutput = shooterCANTalon.getOutputVoltage() / shooterCANTalon.getBusVoltage();
    if (mDebug == 1)
    {
      /* prepare line to print */
      sc_debug_info.append("\tout:");
      sc_debug_info.append(motorOutput);
      sc_debug_info.append("\tspd:");
      sc_debug_info.append(shooterCANTalon.getSpeed());
    }//if debug

    if (mEnabled == 1)
    {
    
      shooterCANTalon.set(mWantedRPM);  //set to neg for direction change 
      if (mDebug == 1)
      {
        /* append more signals to print when in speed mode. */
        sc_debug_info.append("\terr:");
        sc_debug_info.append(shooterCANTalon.getClosedLoopError());
        sc_debug_info.append("\ttrg:");
        sc_debug_info.append(mWantedRPM);
      } //if debug
    } //if enabled  
  }//ShooterTick
}//ShooterControl
