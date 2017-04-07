package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends SampleRobot {
	
	// --------------------------------------------------------- //
	// CHANGE THIS DEPENDING ON WHAT SIDE OF THE FIELD WE ARE ON //
	// --------------------------------------------------------- //

	public boolean ON_RED_SIDE_OF_FIELD = false;
	private boolean SIMPLE_AUTONOMOUS_MODE = true;

	public ControlShooter mShooterController; // shooter functions
	public ControlWinch mWinchController; // winch functions
	public ControlMove mMoveController; // ALL robot movement functions
	public ControlTurret mTurretController;
	//public ControlIntake mIntakeController;
	public ControlElevator mElevatorController;
	private AutoControlElevator mAutoElevatorController;
	private AutoControlShooter mAutoShooterController;
	public ControlFeed mFeedController;

	public double mRobotSpeed = 0.0; // used only in teleop, gives all controls
										// access to robot speed
	public double mShooterTargetRPM = 0.0;
	public double mElevatorSpeed = 0.0;

	//private Control_MoveAimShoot mMoveAimShoot;

	Preferences prefs = Preferences.getInstance();

	Joystick driverStick = new Joystick(0); // set to ID 1 in DriverStation
	Joystick controlStick = new Joystick(1); // set to ID 2 in DriverStation

	long mLastDashboardUpdateTime = 0;

	ThreadVideoCapture video_capture_instance;
	Thread VideoCaptureThreadHandle;

	ThreadVisionProcessor vision_processor_instance;
	Thread VisionProcessorThreadHandle;

	ThreadArduinoGyro arduino_thread_instance;
	Thread arduinoThreadHandle;

	ThreadLIDAR lidar_instance;
	Thread lidarThreadHandle;

	public int ctr = 0;

	@Override
	public void robotInit() {
	}

	/********************************************************************************
	 * Robot Constructor, init all variables, create and start Thread(s)
	 */
	public Robot() {
		Log.mDebug = 1; // Enable logging output

		mShooterController = new ControlShooter(this, controlStick);
		mWinchController = new ControlWinch();
		mMoveController = new ControlMove(driverStick, this); // pass joystick
		mTurretController = new ControlTurret(this);
		//mIntakeController = new ControlIntake(this);
		mElevatorController = new ControlElevator(this);
		mAutoElevatorController = new AutoControlElevator(this);
		mAutoShooterController = new AutoControlShooter(this);
		mFeedController = new ControlFeed(this);

		lidar_instance = new ThreadLIDAR();
		lidarThreadHandle = new Thread(lidar_instance);
		lidarThreadHandle.start();

		video_capture_instance = new ThreadVideoCapture();
		VideoCaptureThreadHandle = new Thread(video_capture_instance);
		VideoCaptureThreadHandle.start();
		video_capture_instance.Enable(); // begin getting frames.

		vision_processor_instance = new ThreadVisionProcessor(video_capture_instance, VideoCaptureThreadHandle, this); // pass
																														// in
																														// refs
																														// to
																														// video
																														// capture
																														// thread
																														// so
																														// it
																														// can
																														// grab
																														// frames
		VisionProcessorThreadHandle = new Thread(vision_processor_instance);
		VisionProcessorThreadHandle.start();

		arduino_thread_instance = new ThreadArduinoGyro();
		arduinoThreadHandle = new Thread(arduino_thread_instance);
		//arduinoThreadHandle.start();

		mMoveController.leftEncoder.reset();
		mMoveController.rightEncoder.reset();

		mLastDashboardUpdateTime = System.currentTimeMillis();
	}// Robot()

	// ---------------------------------------------------------------------------------------------
	// OPERATORCONTROL :
	//

	@Override
	public void operatorControl() {
		mMoveController.mRobotDrive.setSafetyEnabled(false);
		// mShooterController.setRPMWanted(-50);
		mShooterController.Enable();
		mMoveController.MoveOperatorControl(); // human driving watch out!

		mTurretController.Enable();

		mWinchController.Enable();

		mElevatorController.setElevatorSpeed(0.8); // NOTE Elevaotr dir ok
		mElevatorController.setElevatorSecondSpeed(0.6); // 1.0); //0.8);
		//mElevatorController.Enable();

		// mIntakeController.setIntakeSpeed(0.8); //NOTE Intake direction ok
		// mIntakeController.Enable();

		// mFeedController.setFeedSpeed(0.9);
		mFeedController.Enable();

		while (isOperatorControl() && isEnabled()) {
			InputSystem.ReadAllInput(driverStick, controlStick); // Read all
																	// sensor/input
																	// devices

			// ALL UPDATE ROUTINES updating based on read/updated sensor values
			mShooterController.Tick();
			mWinchController.Tick();
			mMoveController.Tick();
			mTurretController.Tick();
			// mIntakeController.Tick();
			mElevatorController.Tick();
			mFeedController.Tick();
			SendDataToSmartDashboard();
			Timer.delay(0.005); // wait for a motor update time
		} // while isEnabled
	} // operatorControl

	// AUTONOMOUS AUTONOMOUS AUTONOMOUS
	// thanks for that...

	@Override
	public void autonomous() {
		
		mTurretController.Enable();
		//mShooterController.Enable(); // control_moveaimshoot will sequence
		// these

		mMoveController.mRobotDrive.setSafetyEnabled(false);
		mMoveController.leftEncoder.reset();
		mMoveController.rightEncoder.reset();
		mAutoElevatorController.setElevatorSpeed(0.8);
		mAutoElevatorController.Enable();
		mAutoShooterController.Enable();
	    mFeedController.Enable();
	    mFeedController.setFeedSpeed(-0.9);

		if (SIMPLE_AUTONOMOUS_MODE) {

			mMoveController.addMoveStraightCMD(-0.25, 350);

		} else {
			// mMoveAimShoot = new Control_MoveAimShoot(this);

			//mMoveController.addDelayCMD(2000);
			//mFeedController.Enable();

			mMoveController.addMoveStraightCMD(-0.25, 115);
			mMoveController.addDoTurnCMD(ON_RED_SIDE_OF_FIELD);
			mMoveController.addMoveStraightCMD(-0.25, 35);
			mMoveController.addMoveStraightCMD(0.25, 70);
			mMoveController.addMoveStraightCMD(-0.25, 35);
		}
		
		while (isAutonomous() && isEnabled()) {
			//mFeedController.Enable();
			
			mMoveController.mRobotDrive.arcadeDrive(0, 0);
			// SendDataToSmartDashboard();
			mMoveController.Tick();
			mAutoShooterController.Tick();
			mAutoElevatorController.Tick();
			
			//System.out.println("tick");
			mTurretController.Tick();
			//mShooterController.Tick();
			//mElevatorController.Tick();
			// mMoveAimShoot.Tick(); //master sequencer of the above, it will
			// enable/disable them as needed
			Timer.delay(0.005);
		}
	}// autonomous

	/**
	 * Update smart dashboard every 1 second
	 */
	void SendDataToSmartDashboard() {
		long deltat = System.currentTimeMillis() - mLastDashboardUpdateTime;
		if (deltat > 1000) {
			SmartDashboard.putNumber("AUTOTARGET XPOS: ", vision_processor_instance.cregions.mXGreenLine);
			SmartDashboard.putNumber("Auto TARGET Enabled: ", vision_processor_instance.cregions.mTargetVisible);
			SmartDashboard.putBoolean("TURRETLIMITSWITCH", mTurretController.turretLimitSwitch.get());
			SmartDashboard.putNumber("TURRETENCODER", mTurretController.GetShooterPosition());
			SmartDashboard.putNumber("LEFTENCODER", mMoveController.leftEncoder.get());
			SmartDashboard.putNumber("RIGHTENCODER", mMoveController.rightEncoder.get());
			SmartDashboard.putNumber("LEFTDISTANCE", mMoveController.leftEncoder.getDistance());
			SmartDashboard.putNumber("RIGHTDISTANCE", mMoveController.rightEncoder.getDistance());
			//SmartDashboard.putNumber("SHOOTER MotorOut:", mShooterController.motorOutput);
			//SmartDashboard.putNumber("SHOOTER RPM", mShooterController.shooterCANTalon.getSpeed());
			//SmartDashboard.putNumber("SHOOTER TARGET RPM:", mShooterController.targetRPM);
			//SmartDashboard.putNumber("SHOOTER ERROR:", mShooterController.shooterCANTalon.getClosedLoopError());
			SmartDashboard.putNumber("XCENTER", vision_processor_instance.cregions.mXGreenLine); // .lastXCenter);
			SmartDashboard.putNumber("NumContours:", vision_processor_instance.cregions.mContours.size());
			SmartDashboard.putNumber("MAPPED:", vision_processor_instance.lastMapped);
			SmartDashboard.putNumber("CM Traveled:", mMoveController.AverageDistanceTraveledInCM());
			mLastDashboardUpdateTime = System.currentTimeMillis();
			SmartDashboard.putNumber("LAST HEADING:", arduino_thread_instance.lastHeading);
			SmartDashboard.putNumber("LIDAR Angle:", lidar_instance.lastAngle);
			SmartDashboard.putNumber("LIDAR SS", lidar_instance.lastSignalStrength);
			SmartDashboard.putNumber("LIDAR distance", lidar_instance.lastDistance);
			SmartDashboard.putNumber("LIDAR status:", lidar_instance.lastStatus);
			SmartDashboard.putString("LIDAR LAST ERROR", lidar_instance.lastError);
			SmartDashboard.putString("LIDARMessage:", lidar_instance.lastMessage);
			SmartDashboard.putString("GYRO Last Error", arduino_thread_instance.lastError);
			SmartDashboard.putString("GYRO Message", arduino_thread_instance.lastMessage);
		}
	} // SendDataToSmartDashboard

	// --------------------------------------------------------------------------------------------
	/**
	 * Linear Interpolation, given a value x2 between x0 and x1 calculate
	 * position between Y0 and Y1
	 * 
	 * @author EA
	 */
	public double Lerp(double y0, double y1, double x0, double x1, double x2) {
		double y2 = y0 * (x2 - x1) / (x0 - x1) + y1 * (x2 - x0) / (x1 - x0);
		return y2;
	}

	// --------------------------------------------------------------------------------------------
	/**
	 * Bounded Linear Interpolation : Limit returned value to be between Y0 and
	 * Y1
	 * 
	 * @author EA
	 */
	public double BoundLerp(double y0, double y1, double x0, double x1, double x2) {
		double y2 = y0 * (x2 - x1) / (x0 - x1) + y1 * (x2 - x0) / (x1 - x0);
		if (y2 < y0)
			y2 = y0;
		else if (y2 > y1)
			y2 = y1;
		return y2;
	}

	/**********************************************************************************
	 * InputSystem static class holds all control input states ReadAllInput()
	 * should be called at top of main loop to refresh states then all Update
	 * routines will pull from here to make their decisions.
	 * 
	 * @author EA
	 *
	 */
	public static class InputSystem {
		// Control stick buttons
		public static boolean Y_Button_Control_Stick = false; //
		public static boolean A_Button_Control_Stick = false;
		public static boolean X_Button_Control_Stick = false;
		public static boolean X_Button_Control_Stick_Prev = false;
		public static boolean B_Button_Control_Stick = false;
		public static boolean RB_Button_Control_Stick = false; // r
		public static boolean RB_Button_Control_Stick_Prev = false;
		public static boolean Start_Button_Control_Stick = false;
		public static boolean Start_Button_Control_Stick_Prev = false;
		public static boolean Start_Button_Control_Stick_Once = false;
		public static boolean Dpad_Up_Control_Stick = false;
		public static boolean Dpad_Up_Control_Stick_Prev = false;
		public static boolean Dpad_Up_Control_Stick_Pressed_Once = false;
		public static boolean Dpad_Up_Control_Stick_Released_Once = false;
		public static boolean Dpad_Down_Control_Stick = false;
		public static boolean Dpad_Down_Control_Stick_Prev = false;
		public static boolean Dpad_Down_Control_Stick_Pressed_Once = false;
		public static boolean Dpad_Down_Control_Stick_Released_Once = false;
		public static int Dpad_Angle_Control_Stick = 0;
		public static double RT_Button_Control_Stick = 0.0;
		public static double RT_Button_Control_Stick_Prev = 0.0;
		public static boolean RT_Button_Control_Stick_Pressed_Once = false;
		public static double LT_Button_Control_Stick = 0.0;
		public static double LT_Button_Control_Stick_Prev = 0.0;
		public static boolean LT_Button_Control_Stick_Pressed_Once = false;

		// Driver stick buttons
		public static boolean Y_Button_Driver_Stick = false;
		public static boolean A_Button_Driver_Stick = false;
		public static boolean X_Button_Driver_Stick = false; //
		public static boolean B_Button_Driver_Stick = false;
		public static boolean B_Button_Driver_Stick_Prev = false;
		public static boolean RB_Button_Driver_Stick = false; //
		public static boolean RB_Button_Driver_Stick_Prev = false;
		public static boolean Start_Button_Driver_Stick = false; // small black
																	// button on
																	// front
		public static boolean Start_Button_Driver_Stick_Prev = false;
		public static boolean Start_Button_Driver_Stick_Once = false;
		public static boolean Back_Button_Control_Stick = false;
		public static boolean Back_Button_Control_Stick_Prev = false;
		public static boolean Back_Button_Control_Stick_Once = false;

		public static boolean Turret_Limit_Switch = false;

		public static void ReadAllInput(Joystick driverstk, Joystick controlstk) {
			Y_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_Y_BUTTON);
			A_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_A_BUTTON);
			X_Button_Control_Stick_Prev = X_Button_Control_Stick;
			X_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_X_BUTTON);
			B_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_B_BUTTON);
			RB_Button_Control_Stick_Prev = RB_Button_Control_Stick;
			RB_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_RIGHT_BACK_BUTTON);
			Start_Button_Control_Stick_Prev = Start_Button_Control_Stick;
			Start_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_SMALL_START_BUTTON);
			Start_Button_Control_Stick_Once = Start_Button_Control_Stick_Prev == false
					&& Start_Button_Control_Stick == true;
			Back_Button_Control_Stick_Prev = Back_Button_Control_Stick;
			Back_Button_Control_Stick = controlstk.getRawButton(IOMapping.CONTROL_SMALL_BACK_BUTTON);
			Back_Button_Control_Stick_Once = Back_Button_Control_Stick_Prev == false
					&& Back_Button_Control_Stick == true;
			Dpad_Angle_Control_Stick = controlstk.getPOV(IOMapping.CONTROL_DPAD);
			Dpad_Up_Control_Stick_Prev = Dpad_Up_Control_Stick;
			Dpad_Up_Control_Stick = Dpad_Angle_Control_Stick >= 315 || Dpad_Angle_Control_Stick <= 45;
			Dpad_Up_Control_Stick_Pressed_Once = Dpad_Up_Control_Stick_Prev == false && Dpad_Up_Control_Stick == true;
			Dpad_Up_Control_Stick_Released_Once = Dpad_Up_Control_Stick_Prev == true && Dpad_Up_Control_Stick == false;
			Dpad_Down_Control_Stick_Prev = Dpad_Down_Control_Stick;
			Dpad_Down_Control_Stick = Dpad_Angle_Control_Stick >= 135 && Dpad_Angle_Control_Stick <= 225;
			Dpad_Down_Control_Stick_Pressed_Once = Dpad_Down_Control_Stick_Prev == false
					&& Dpad_Down_Control_Stick == true;
			Dpad_Down_Control_Stick_Released_Once = Dpad_Down_Control_Stick_Prev == true
					&& Dpad_Down_Control_Stick == false;
			RT_Button_Control_Stick_Prev = RT_Button_Control_Stick;
			RT_Button_Control_Stick = controlstk.getRawAxis(3);
			RT_Button_Control_Stick_Pressed_Once = Math.abs(RT_Button_Control_Stick_Prev) < 0.1
					&& Math.abs(RT_Button_Control_Stick) >= 0.1;
			LT_Button_Control_Stick_Prev = LT_Button_Control_Stick;
			LT_Button_Control_Stick = controlstk.getAxis(AxisType.kZ);
			LT_Button_Control_Stick_Pressed_Once = Math.abs(LT_Button_Control_Stick_Prev) < 0.1
					&& Math.abs(LT_Button_Control_Stick) >= 0.1;

			Y_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_Y_BUTTON);
			A_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_A_BUTTON);
			X_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_X_BUTTON);
			B_Button_Driver_Stick_Prev = B_Button_Driver_Stick;
			B_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_B_BUTTON);
			RB_Button_Driver_Stick_Prev = RB_Button_Driver_Stick;
			RB_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_RIGHT_BACK_BUTTON);
			Start_Button_Driver_Stick_Prev = Start_Button_Driver_Stick;
			Start_Button_Driver_Stick = driverstk.getRawButton(IOMapping.DRIVER_SMALL_START_BUTTON);
			Start_Button_Driver_Stick_Once = Start_Button_Driver_Stick_Prev == false
					&& Start_Button_Driver_Stick == true;
		} // ReadAllInput
	} // public static inputsystem class

} // class Robot
