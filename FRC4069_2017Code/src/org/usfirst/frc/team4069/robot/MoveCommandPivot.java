package org.usfirst.frc.team4069.robot;

public class MoveCommandPivot extends MoveCommand
{
  double mDistance = 0.0;
  double mSpeed;
  double mDegrees = 0;
  ControlMove mControlMove;
  public double correctionFactor = 0.0;
  private final double ERROR_SCALING_CONST_P = .400;
  private double error = 0.0;

  public MoveCommandPivot(ControlMove ctrlmove, double degrees, double speed)
  {
    mControlMove = ctrlmove;

    mSpeed = speed;
    mDegrees = degrees;

    degrees %= 360;
    double frac = degrees / 360; // fractional amount
    double bothwheeldist = -1 * frac * mControlMove.mDriveBaseCircumference;
    mDistance = bothwheeldist;
  }

  @Override
  public void Init()
  {
    mControlMove.leftEncoder.reset();
    mControlMove.rightEncoder.reset(); // zero counts
  }

  // For + degrees left wheel goes forward, right wheel goes backwards
  @Override
  public boolean Tick()
  {
    double leftDistance = mControlMove.leftEncoder.getDistance(); // get left encoder as normal distance
    double rightDistance = mControlMove.rightEncoder.getDistance() * -1; // right wheel is going backwards so distance is * -1

    double averageDistance = (leftDistance + rightDistance) / 2;

    if (averageDistance >= mDistance)
    {
      mControlMove.leftDriveMotor.set(0);
      mControlMove.rightDriveMotor.set(0);
      return true;
    }

    error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
                                          // if error < 0 right is ahead add -error to right
    correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

    mControlMove.leftDriveMotor.set(mSpeed - correctionFactor); // +err means left ahead, subtract from left speed
    mControlMove.rightDriveMotor.set(mSpeed + correctionFactor); // -err means right ahead, add -err to right speed
    return false; // not done yet
  }// Tick
}
