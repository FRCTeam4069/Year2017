package org.usfirst.frc.team4069.robot;

import java.util.ArrayList;

public class MoveCommandTrace extends MoveCommand
{
  ArrayList<Vector2> mPath;
  double mSpeed = 0.0;
  int index = 0;
  double currentDegrees = 0;
  double wheelBaseWidthCM = 100;
  boolean rotationMode = true;
  Vector2 currentPos;
  double driveDist = 0;
  boolean doneDriving = true;
  private ControlMove mControlMove;

  public double error = 0.0;
  public double correctionFactor = 0.0;
  public double resultantleftspeed = 0.0;
  public double resultantrightspeed = 0.0;
  private final double ERROR_SCALING_CONST_P = .400;
  private int TickCounter = 0;

  public MoveCommandTrace(ControlMove ctrlmove, ArrayList<Vector2> path, double speed)
  {
    mControlMove = ctrlmove;
    mSpeed = speed;
    mPath = path;
  }

  @Override
  public boolean Tick()
  {
    if (index == mPath.size() - 2)
      return true;
    float mDistance = 0f;
    Vector2 a = mPath.get(index);
    Vector2 b = mPath.get(index + 1);
    double degrees = Math.toDegrees(Math.atan2(b.y - a.y, b.x - a.x));
    if (doneDriving && !rotationMode)
    {
      rotationMode = true;
      doneDriving = false;
      double rot = degrees %= 360;
      double frac = rot / 360; // fractional amount
      index++;
      mDistance = (float) (-1 * frac * mControlMove.mDriveBaseCircumference);
    }
    else if (Math.abs(currentDegrees - degrees) < 4)
    {
      if (rotationMode)
      {
        driveDist = Math.sqrt(Math.pow(a.x - b.x, 2) + Math.pow(a.y - b.y, 2));
        rotationMode = false;
      }
    }

    rotationMode = false;

    if (rotationMode)
    {
      double leftDistance = mControlMove.leftEncoder.getDistance(); // get left encoder as normal distance
      double rightDistance = mControlMove.rightEncoder.getDistance() * -1; // right wheel is going backwards so distance is * -1

      double averageDistance = (leftDistance + rightDistance) / 2;

      if (averageDistance >= mDistance)
      {
        mControlMove.leftDriveMotor.set(0);
        mControlMove.rightDriveMotor.set(0);
        currentDegrees = degrees;
      }

      error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
      // if error < 0 right is ahead add -error to right
      correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

      mControlMove.leftDriveMotor.set(mSpeed - correctionFactor); // +err means left ahead, subtract from left speed
      mControlMove.rightDriveMotor.set(mSpeed + correctionFactor); // -err means right ahead, add -err to right speed
    }
    else
    {
      // MOVE MODE
      double leftDistance = mControlMove.leftEncoder.getDistance();
      double rightDistance = mControlMove.rightEncoder.getDistance();
      double averageDistance = (leftDistance + rightDistance) / 2;

      if (Math.abs(averageDistance) >= driveDist)
      {
        mControlMove.leftDriveMotor.set(0);
        mControlMove.rightDriveMotor.set(0);
        doneDriving = true;
      }
      TickCounter++;
      error = leftDistance - rightDistance; // if error > 0 left is ahead subtract error from left
                                            // if error < 0 right is ahead add -error to right
      correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error

      resultantleftspeed = mSpeed - correctionFactor;
      resultantrightspeed = mSpeed + correctionFactor;
      if (resultantleftspeed > 1.0)
        resultantleftspeed = 1.0;
      if (resultantrightspeed > 1.0)
        resultantrightspeed = 1.0;
      if (resultantleftspeed < -1.0)
        resultantleftspeed = -1.0;
      if (resultantrightspeed < -1.0)
        resultantrightspeed = -1.0;

      mControlMove.leftDriveMotor.set(resultantleftspeed); // +err means left ahead, subtract from left speed
      mControlMove.rightDriveMotor.set(resultantrightspeed); // -err means right ahead, add -err to right speed
    }
    return false;
  }

  @Override
  public void Init()
  {
    mControlMove.leftEncoder.reset();
    mControlMove.rightEncoder.reset(); // clear encoder distances/ticks
  }

}
