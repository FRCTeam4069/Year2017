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
  
  public static final int turretEncoderMax = -14100;
  
  public static final int turretEncoderMin = -39;
  
  public static final int turretEncoderMidpoint = -6153;

  private int targetEncoderPosition;
  
  private double maxTargetPositionSpeed = 0.35;
  
  private boolean shouldUseTargetPosition = false;
  
  private boolean turretLimitSwitchEnabled;
  
  private boolean turretEncoderZeroed = false;
  private double turretEncoderPosition;

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
  
  public void setTargetEncoderPosition(int targetEncoderPosition){
	  this.targetEncoderPosition = targetEncoderPosition;
	  shouldUseTargetPosition = true;
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
	  if(Robot.InputSystem.A_Button_Control_Stick){
		  setTargetEncoderPosition(turretEncoderMidpoint);
	  }
	  
    // return;
	  
	  turretLimitSwitchEnabled = turretLimitSwitch.get();
	  
	  turretEncoderPosition = turretTalon.getPosition();
	  
    if (turretEncoderZeroed) {
    	double maxSpeed = 0.5;
    	double motorValue = mRobot.controlStick.getAxis(AxisType.kY) * maxSpeed;
    	if((motorValue > 0 && turretEncoderPosition < turretEncoderMidpoint) || (motorValue < 0 && turretEncoderPosition > turretEncoderMidpoint)){
    		motorValue *= Lerp(1, 0, 0, turretEncoderMin - turretEncoderMidpoint, Math.abs(turretEncoderPosition - turretEncoderMidpoint));
    	}
    	if(Math.abs(motorValue) > 0.1){
    		shouldUseTargetPosition = false;
    	}
    	if(shouldUseTargetPosition){
    		double error = (turretEncoderPosition - targetEncoderPosition) / 2000;
    		System.out.println("Error: " + error);
    		motorValue = error > 0 ? Math.min(maxTargetPositionSpeed, error) : Math.max(-maxTargetPositionSpeed, error);
    	}
    	motorValue = lpf.calculate(motorValue);
    	if (turretEncoderPosition >= turretEncoderMin || turretLimitSwitchEnabled) {
    		if(motorValue > 0){
    			turretTalon.set(motorValue);
    		}
    		else{
    			turretTalon.set(0);
    		}
    	}
    	else if(turretEncoderPosition <= turretEncoderMax){
    		if(motorValue < 0){
    			turretTalon.set(motorValue);
    		}
    		else{
    			turretTalon.set(0);
    		}
    	}
    	else{
    		turretTalon.set(motorValue);
    	}
    }
    else
    { // here if turret is not yet zeroed, must zero first
    	if (turretLimitSwitchEnabled == false) // if limit NOT pressed
    	{
    		turretTalon.set(-0.1); // crawl toward limit switch
    	}
    	else{
    		turretTalon.set(0);
    		while (turretLimitSwitchEnabled == true)
    		{
    			turretTalon.set(0.1); // go until off limit switch
    			turretLimitSwitchEnabled = turretLimitSwitch.get();
    		}
    		turretTalon.set(0); // turn off talon
    		turretTalon.setEncPosition(0); // reset encoder ticks
    		turretEncoderZeroed = true;
    	}
    } // else turret not yet zeroed
    
    /*if (mRobot.vision_processor_instance.cregions.mTargetVisible == 1)
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
    }*/
  } // Tick()
}// class ControlTurret