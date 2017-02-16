package org.usfirst.frc.team4069.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;

/**
 * ShooterControl : Uses a CANTalon with a encoder and a PID controller to set the RPM's on the shooter motor to a fixed value and hold them there
 * 
 * @author EA Create a instance of this class, then call its tick method periodically
 */
public class ControlShooter
{
  public CANTalon shooterCANTalon;
  private int mEnabled = 0;
  public double motorOutput = 0.0;
  public double targetRPM = 0.0;
  public LowPassFilter lpf = new LowPassFilter(1000);
  private Joystick _joy;
  private int mTesting=0;
  
  public ControlShooter(Joystick stk)
  {
    _joy = stk;
    shooterCANTalon = new CANTalon(IOMapping.SHOOTER_CANBUS_PORT);
    shooterCANTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    shooterCANTalon.reverseSensor(false);
    shooterCANTalon.configEncoderCodesPerRev(4096); // Magnetic encoder ticks
                                                    // per revolution 2^12
    // shooterCANTalon.setAllowableClosedLoopErr(0); //4096);
    // Set peak and nominal outputs 12 volts means full power
    shooterCANTalon.configNominalOutputVoltage(+0.0f, -0.0f);
    shooterCANTalon.configPeakOutputVoltage(12.0f, -12.0f);
    targetRPM = 0.0;

    // set closed loop gains in slot 0
    shooterCANTalon.setProfile(0);
    shooterCANTalon.setF(0.1097);
    shooterCANTalon.setP(0.15);
    shooterCANTalon.setI(0);
    shooterCANTalon.setD(0); //0.05);
    shooterCANTalon.changeControlMode(TalonControlMode.Speed);
  } // ShooterControl init

  public void setRPMWanted(double rpm)
  {
    targetRPM = rpm;
  }

  public double getCurrentRPM()
  {
    return shooterCANTalon.getSpeed();
  }

  /**
   * Passed a percentage 0-100, if the current rpm is within that percentage of the target rpm
   * returns 1, else returns 0
   * @return
   */
  public int isShooterWithingPercentage(double percentok)
  {
    double currpm = shooterCANTalon.getSpeed();
    double delta = Math.abs(currpm - targetRPM);
    double percentage = delta / targetRPM;
    if (percentage < percentok)
      return 1;
    else
      return 0;
  }//isShooterWithinPercentage
  
  
  /**
   * ShooterTick : Should be called with all the other update functions after inputs have been read/updated IF proper button held down, will call set speed to set to wanted RPM's
   */
  public void Tick()
  {
    if (mEnabled == 0)
    {
      shooterCANTalon.set(0);
      return;
    }
    motorOutput = shooterCANTalon.getOutputVoltage() / shooterCANTalon.getBusVoltage();

    if (mTesting == 1) //if testing, enable buttons, else just set rpm on tick
    {
      targetRPM = 0.0;

      if (_joy.getRawButton(IOMapping.CONTROL_A_BUTTON))
      {
        targetRPM = 1800; // 1300;
      }
      if (_joy.getRawButton(IOMapping.CONTROL_B_BUTTON))
      {
        targetRPM = 1900; // best spot 2800rpm output when set to this
      }
      else if (_joy.getRawButton(IOMapping.CONTROL_X_BUTTON))
      {
        targetRPM = 2000;
      }
      else if (_joy.getRawButton(IOMapping.CONTROL_Y_BUTTON))
      {
        targetRPM = 2100; // actual 2797rpm???
      }
      if ((_joy.getRawButton(5)) || (_joy.getRawButton(6)))
      {
        shooterCANTalon.changeControlMode(TalonControlMode.Speed);
        shooterCANTalon.set(0);
      }
    }

    if (targetRPM != 0.0)
    {
      shooterCANTalon.changeControlMode(TalonControlMode.Speed);
      shooterCANTalon.set(lpf.calculate(targetRPM)); //
    }
  }// ShooterTick


  public void Enable()
  {
    mEnabled = 1;
  }

  public void Disable()
  {
    mEnabled = 0;
  }

}// ShooterControl
