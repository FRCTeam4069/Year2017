package org.usfirst.frc.team4069.robot.commands;

import java.util.ArrayList;
import java.util.List;

import org.usfirst.frc.team4069.robot.Robot;
import org.usfirst.frc.team4069.robot.subsystems.Drivebase;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class PlayBackActions extends Command {

	private double speed;
	
	private Drivebase drivebase;
	
	private List<Double> leftWheelValues = new ArrayList<Double>();
	private List<Double> rightWheelValues = new ArrayList<Double>();
	
	private final double accuracy = 0.1;
	
	
    public PlayBackActions() {
    	requires(Robot.drivebase);
    	drivebase = Robot.drivebase;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	// reset drivebase counters
    	// load wheel value lists from file
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Math.abs(drivebase.getLeftDist() - leftWheelValues.get(0)) < accuracy && Math.abs(drivebase.getRightDist() - rightWheelValues.get(0)) < accuracy) {
    		leftWheelValues.remove(0);
    		rightWheelValues.remove(0);
    	}
    	// drive towards left/rightWheelValues[0]
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return leftWheelValues.isEmpty();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
