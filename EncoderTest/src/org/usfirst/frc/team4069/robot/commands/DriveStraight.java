package org.usfirst.frc.team4069.robot.commands;

import org.usfirst.frc.team4069.robot.Robot;
import org.usfirst.frc.team4069.robot.subsystems.Drivebase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveStraight extends Command {

	private double speed;
	private double distMeters;
	
	private final double ERROR_SCALING_CONST_P = 1;
	
	private Drivebase drivebase;
	
	private double initialLeftCount, initialRightCount;
	
    public DriveStraight(double speed, double distMeters) {
        requires(Robot.drivebase);
        drivebase = Robot.drivebase;
        this.speed = speed;
        this.distMeters = distMeters;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	initialLeftCount = drivebase.getLeftDist();
    	initialRightCount = drivebase.getRightDist();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double leftDelta = drivebase.getLeftDist() - initialLeftCount;
    	double rightDelta = drivebase.getRightDist() - initialRightCount;
    	double error = leftDelta - rightDelta;
    	double correctionFactor = error * ERROR_SCALING_CONST_P;
    	drivebase.tankDrive(speed + correctionFactor, speed - correctionFactor);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double leftDelta = drivebase.getLeftDist() - initialLeftCount;
    	double rightDelta = drivebase.getRightDist() - initialRightCount;
    	if ((leftDelta + rightDelta) / 2 > distMeters) {
    		return true;
    	}
    	return false;
    }
}
