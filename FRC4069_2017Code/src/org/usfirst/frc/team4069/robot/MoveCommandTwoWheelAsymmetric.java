package org.usfirst.frc.team4069.robot;

public class MoveCommandTwoWheelAsymmetric extends MoveCommand {
	
	double mLeftSpeed, mLeftDistance;
	double mRightMultiplier;
	private ControlMove mControlMove;
	private final double ERROR_SCALING_CONST_P = .400;

	public MoveCommandTwoWheelAsymmetric(ControlMove ctrlmove, double averageSpeed, double leftDistance, double rightDistance)
	{
	  mControlMove = ctrlmove;
	  mLeftDistance = leftDistance;
	  mRightMultiplier = rightDistance / leftDistance;
	  double averageToLeftMultiplier = leftDistance / ((leftDistance + rightDistance) / 2);
	  mLeftSpeed = averageSpeed * averageToLeftMultiplier;
	}
	
	// Instead of distance for both wheels, takes radius of circle and angle
	public MoveCommandTwoWheelAsymmetric(ControlMove ctrlmove, double averageSpeed, double radius, double angle, boolean moveRight){
		mControlMove = ctrlmove;
		double halfRobotWidth = ControlMove.mDriveBaseRadius;
		double leftDistance, rightDistance;
		if(moveRight){
			leftDistance = radius + halfRobotWidth;
			rightDistance = radius - halfRobotWidth;
		}
		else{
			leftDistance = radius - halfRobotWidth;
			rightDistance = radius + halfRobotWidth;
		}
		leftDistance *= angle;
		rightDistance *= angle;
		mLeftDistance = leftDistance;
		mRightMultiplier = rightDistance / leftDistance;
		double averageToLeftMultiplier = leftDistance / ((leftDistance + rightDistance) / 2);
		mLeftSpeed = averageSpeed * averageToLeftMultiplier;
	}
	
	@Override
	public boolean Tick() {
		double leftDistance = mControlMove.leftEncoder.getDistance();
	    double rightDistance = mControlMove.rightEncoder.getDistance();

	    if (leftDistance >= mLeftDistance && rightDistance >= (mLeftDistance * mRightMultiplier))
	    {
	      mControlMove.leftDriveMotor.set(0);
	      mControlMove.rightDriveMotor.set(0);
	      return true;
	    }
	    double error = leftDistance - (rightDistance / mRightMultiplier); // if error > 0 left is ahead subtract error from left
	                                          // if error < 0 right is ahead add -error to right
	    double correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

	    double resultantLeftSpeed = -mLeftSpeed - correctionFactor;
	    double resultantRightSpeed = -(mLeftSpeed * mRightMultiplier) + correctionFactor;
	    if (resultantLeftSpeed > 1.0)
	      resultantLeftSpeed = 1.0;
	    if (resultantRightSpeed > 1.0)
	      resultantRightSpeed = 1.0;
	    if (resultantLeftSpeed < -1.0)
	      resultantLeftSpeed = -1.0;
	    if (resultantRightSpeed < -1.0)
	      resultantRightSpeed = -1.0;
	    System.out.println("leftdist=" + leftDistance + ",rightdist=" + rightDistance + ",error=" + error + ",correctionFactor=" + correctionFactor + ",resultleft=" + resultantLeftSpeed + ",resultsright=" + resultantRightSpeed);
	    mControlMove.leftDriveMotor.set(resultantLeftSpeed); // +err means left ahead, subtract from left speed
	    mControlMove.rightDriveMotor.set(resultantRightSpeed); // -err means right ahead, add -err to right speed
	    return false; // not done yet
	}

	@Override
	public void Init() {
		mControlMove.leftEncoder.reset();
	    mControlMove.rightEncoder.reset();
	}
}
