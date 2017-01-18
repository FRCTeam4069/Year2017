package org.usfirst.frc.team4069.vision;



import org.opencv.core.Point;
import org.opencv.core.Rect;

public class Target {
	
	public Rect HorizontalTarget;
	public Rect VerticalTarget;

	public double HorizontalAngle;
	public double VerticalAngle;
	public double Horizontal_W_H_Ratio;
	public double Horizontal_H_W_Ratio;
	public double Vertical_W_H_Ratio;
	public double Vertical_H_W_Ratio;

	public Point HorizontalCenter;
	public Point VerticalCenter;

	public boolean HorizGoal;
	public boolean VertGoal;
	public boolean HotGoal;
	public boolean matchStart;
	public boolean validFrame;

	//camera bool
	public boolean cameraConnected;

	public int targetLeftOrRight;
	public int lastTargerLorR;
	public int hotLeftOrRight;
	public double targetDistance;
	
}
