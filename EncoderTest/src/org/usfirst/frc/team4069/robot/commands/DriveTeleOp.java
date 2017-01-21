package org.usfirst.frc.team4069.robot.commands;

import org.usfirst.frc.team4069.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveTeleOp extends Command {
	
    public DriveTeleOp() {
        requires(Robot.drivebase);
        
    }

    protected void execute() {
    	double joystickX = Robot.oi.driverStick.getX();
    	double joystickY = Robot.oi.driverStick.getY();
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
