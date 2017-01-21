package org.usfirst.frc.team4069.robot.subsystems;

import org.usfirst.frc.team4069.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Drivebase extends Subsystem {

	private final double METERS_PER_COUNT = 120;
	
	private VictorSP left_motor = new VictorSP(RobotMap.PWM_LEFT_MOTOR);
	private VictorSP right_motor = new VictorSP(RobotMap.PWM_RIGHT_MOTOR);
	private Encoder left_encoder = new Encoder(RobotMap.DIO_LEFT_ENC_A, RobotMap.DIO_LEFT_ENC_B);
	private Encoder right_encoder = new Encoder(RobotMap.DIO_RIGHT_ENC_A, RobotMap.DIO_RIGHT_ENC_B);

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    public Drivebase() {
    	left_encoder.reset();
    	right_encoder.reset();
    }
    
    public double getLeftDist() {
    	return METERS_PER_COUNT * left_encoder.get();
    }
    
    public double getRightDist() {
    	return METERS_PER_COUNT * right_encoder.get();
    }
    
    public void tankDrive(double leftSpeed, double rightSpeed) {
    	left_motor.setSpeed(leftSpeed);
    	right_motor.setSpeed(rightSpeed);
    }
}

