package org.usfirst.frc.team4069.robot;

import org.usfirst.frc.team4069.robot.Robot.InputSystem;

import edu.wpi.first.wpilibj.Talon;

public class AutoControlElevator {
	private Talon elevatorTalon;
	private int mEnabled = 0;
	private int mDebug = 0;
	private double mSpeed = 0.0; // default speed used by elevator
	private Robot mRobot;

	public AutoControlElevator(Robot robot) {
		mRobot = robot;
		elevatorTalon = new Talon(IOMapping.ELEVATOR_PWM_PORT);
		elevatorTalon.set(0);
	} // controlElevator

	/**
	 * Set main speed of elevator
	 * 
	 * @param spd
	 */

	public void setElevatorSpeed(double spd) {
		mSpeed = spd;
	}

	public void EnableDebug() {
		mDebug = 1;
	}

	public void DisableDebug() {
		mDebug = 0;
	}

	public void Enable() {
		mEnabled = 1;
		elevatorTalon.set(getElevatorSpeed());
	}

	public void Disable() {
		mEnabled = 0;
		elevatorTalon.set(0);
	}

	/**
	 * Calculates elevator speed based on direction
	 * 
	 * @return
	 */
	private double getElevatorSpeed() {
		return mSpeed * -1;
		
	}

	public void Tick() {
		
		//updateDirection(); // if back button is pressed once, toggle between
							// main/second speed
		
		double speed = 0;
		
		if (mEnabled == 1) {
			speed = getElevatorSpeed();
		}
		
		mRobot.mElevatorSpeed = speed;
		
		elevatorTalon.set(speed);
	}
}
