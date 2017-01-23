package org.usfirst.frc.team4069.robot;

import org.usfirst.frc.team4069.robot.Robot;
import java.util.ArrayList;
import java.util.List;

public class RecordMotions {

	private List<Double> leftWheelValues = new ArrayList<Double>();
	private List<Double> rightWheelValues = new ArrayList<Double>();
	private Robot robot = null;
	
	public RecordMotions(Robot robot)
	{
		this.robot = robot;
	}
	
	private void RecordActions () {
		// THIS METHOD IS NOT CALLED: IT MUST BE UPDATED TO WORK WITH THE FRC LIBRARIES
		leftWheelValues.add(robot.drivebase.getLeftDist());
		rightWheelValues.add(robot.drivebase.getRightDist());
	}
	  
}
