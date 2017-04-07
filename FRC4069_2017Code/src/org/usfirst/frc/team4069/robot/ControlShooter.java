package org.usfirst.frc.team4069.robot;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;

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
  private double targetRPMPercentChange = 0.0;
  public LowPassFilter lpf = new LowPassFilter(1000);
  private Joystick _joy;
  private int mTesting = 1;
  public boolean runFeed = false;
  private Robot mRobot;

  // private static final HashMap<Double, Integer> speedTableMetersToRPM;
  // private static final Double[] speedTableDistances;
  // private static final double[] speedTableSlopesAboveIndex;

  /*
   * static { HashMap<Double, Integer> t = new HashMap<>();
   * 
   * t.put(5.0, 800); t.put(7.5, 1700); t.put(10.8, 2500);
   * 
   * speedTableDistances = (Double[]) t.keySet().toArray(); Arrays.sort(speedTableDistances); Integer[] speedTableSpeeds = (Integer[]) t.values().toArray(); Arrays.sort(speedTableSpeeds); speedTableSlopesAboveIndex = new double[t.size()]; for (int i = 0; i <
   * speedTableDistances.length - 1; i++) { int y2 = speedTableSpeeds[i + 1]; int y1 = speedTableSpeeds[i]; double x2 = speedTableDistances[i + 1]; double x1 = speedTableDistances[i]; speedTableSlopesAboveIndex[i] = (double)(y2 - y1) / (x2 - x1); } speedTableMetersToRPM =
   * (HashMap<Double, Integer>) Collections.unmodifiableMap(t); }
   */

  public ControlShooter(Robot robot, Joystick stk)
  {
    _joy = stk;
    mRobot = robot;
    shooterCANTalon = new CANTalon(IOMapping.SHOOTER_CANBUS_PORT);
    shooterCANTalon.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
    shooterCANTalon.reverseSensor(true);
    shooterCANTalon.configEncoderCodesPerRev(4096); // Magnetic encoder ticks
                                                    // per revolution 2^12
    // shooterCANTalon.setAllowableClosedLoopErr(0); //4096);
    // Set peak and nominal outputs 12 volts means full power
    shooterCANTalon.configNominalOutputVoltage(+0.0f, -0.0f);
    shooterCANTalon.configPeakOutputVoltage(12.0f, -12.0f);
    targetRPM = 0.0;

    // set closed loop gains in slot 0
    shooterCANTalon.setProfile(0);
    shooterCANTalon.setF(0.0325); // 1097);
    shooterCANTalon.setP(0); // .15);
    shooterCANTalon.setI(0);
    shooterCANTalon.setD(0); // 0.05);
    shooterCANTalon.changeControlMode(TalonControlMode.Speed);
  } // ShooterControl init

  public void setRPMWanted(double rpm)
  {
    targetRPM = rpm;
  }

  // experimental
  /*
   * private double getRPMFromDistance (double distMeters) { int indexBelowDistMeters = -1; for (int i = 0; i < speedTableDistances.length; i++) { if (speedTableDistances[i] >= distMeters) { indexBelowDistMeters = i - 1; break; } } double slope = -1.0; double belowDistIterator =
   * indexBelowDistMeters; for (int i = 0; belowDistIterator >= 0; i++) { double highSlope = speedTableSlopesAboveIndex[indexBelowDistMeters + i + 1]; double lowSlope = speedTableSlopesAboveIndex[indexBelowDistMeters - i]; double averageSlope = (highSlope - lowSlope) / 2; double
   * weight = 1 / (i + 1); slope = (slope * (1 - weight)) + (averageSlope * weight); belowDistIterator--; } return (slope * distMeters) + speedTableMetersToRPM.get(speedTableDistances[indexBelowDistMeters]); }
   */

  public double getCurrentRPM()
  {
    return shooterCANTalon.getSpeed();
  }

  /**
   * Passed a percentage 0-100, if the current rpm is within that percentage of the target rpm returns 1, else returns 0
   * 
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
  }// isShooterWithinPercentage

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

    if (mTesting == 1) // if testing, enable buttons, else just set rpm on tick
    {
      targetRPM = 0.0;

      if (_joy.getRawButton(IOMapping.CONTROL_A_BUTTON))
      {
        targetRPM = 2850; // 1300;
      }
      else if (_joy.getRawButton(IOMapping.CONTROL_B_BUTTON))
      {
        targetRPM = 3050; // best spot 2800rpm output when set to this
      }
      else if (_joy.getRawButton(IOMapping.CONTROL_X_BUTTON))
      {
        targetRPM = 2000;
      }
      else if (_joy.getRawButton(IOMapping.CONTROL_Y_BUTTON))
      {
        targetRPM = 3250; // actual 2797rpm???
      }
      /*
       * if ((_joy.getRawButton(5)) || (_joy.getRawButton(6))) { shooterCANTalon.changeControlMode(TalonControlMode.Speed); shooterCANTalon.set(0); }
       */
    }

    mRobot.mShooterTargetRPM = targetRPM;

    // double humanInput = _joy.getRawAxis(IOMapping.CONTROL_RIGHT_Y_AXIS);
    // double tenPercentOfTarget = targetRPM * 0.1;
    shooterCANTalon.changeControlMode(TalonControlMode.Speed);
    shooterCANTalon.set(lpf.calculate(targetRPM));// + humanInput * tenPercentOfTarget)); //
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
