package org.usfirst.frc.team4069.robot;

public class MoveCommandStraight extends MoveCommand
{
  double mSpeed;
  double mDistance;
  private ControlMove mControlMove;
  private double correctionFactor = 0.0;
  private double resultantleftspeed = 0.0;
  private double resultantrightspeed = 0.0;
  private double error = 0.0;
  private final double ERROR_SCALING_CONST_P = .4;

  MoveCommandStraight(ControlMove ctrlmove, double speed, double distance)
  {
    mControlMove = ctrlmove;
    mSpeed = speed;
    mDistance = distance;
  }

  @Override
  public void Init()
  {
    mControlMove.leftEncoder.reset();
    mControlMove.rightEncoder.reset();
  }

  @Override
  public boolean Tick()
  {
    System.out.println("Move Command straight tick");
    double leftDistance = mControlMove.leftEncoder.getDistance();
    double rightDistance = mControlMove.rightEncoder.getDistance();
    double averageDistance = (leftDistance + rightDistance) / 2;

    if (Math.abs(averageDistance) >= mDistance)
    {
      mControlMove.leftDriveMotor.set(0);
      mControlMove.rightDriveMotor.set(0);
      return true;
    }

    error = leftDistance - rightDistance; // if error > 0 left is ahead add - error to left
                                          // if error < 0 right is ahead add -error to right
    correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error should come from ticks/wheelrotation data
    double abscorrFactor = Math.abs(correctionFactor);

    if (mSpeed > 0)
    {
      if (error > 0)
      {
        resultantleftspeed = mSpeed - abscorrFactor; // left is ahead, slow it down by correctionFactor mspeed is >0
        resultantrightspeed = mSpeed;
      }
      else
      {
        resultantrightspeed = mSpeed - abscorrFactor; // right is ahead, slow it down by correctionFactor
        resultantleftspeed = mSpeed;
      }

      if (resultantleftspeed > 1.0)
        resultantleftspeed = 1.0;
      if (resultantrightspeed > 1.0)
        resultantrightspeed = 1.0;
      if (resultantleftspeed < -1.0)
        resultantleftspeed = -1.0;
      if (resultantrightspeed < -1.0)
        resultantrightspeed = -1.0;
    }
    else // her if moving backwards (mSpeed < 0)
    {
      if (error > 0)
      {
        resultantleftspeed = mSpeed + abscorrFactor; // left is ahead, slow it down by correctionFactor mspeed is < 0
        resultantrightspeed = mSpeed;
      }
      else
      {
        resultantrightspeed = mSpeed + abscorrFactor; // right is ahead, slow it down by correctionFactor
        resultantleftspeed = mSpeed;
      }

      if (resultantleftspeed > 1.0)
        resultantleftspeed = 1.0;
      if (resultantrightspeed > 1.0)
        resultantrightspeed = 1.0;
      if (resultantleftspeed < -1.0)
        resultantleftspeed = -1.0;
      if (resultantrightspeed < -1.0)
        resultantrightspeed = -1.0;
    }
    System.out.println("leftdist=" + leftDistance + ",rightdist=" + rightDistance + ",error=" + error + ",correctionFactor=" + correctionFactor + ",resultleft=" + resultantleftspeed + ",resultsright=" + resultantrightspeed);
    mControlMove.leftDriveMotor.set(resultantleftspeed); // +err means left ahead, subtract from left speed
    mControlMove.rightDriveMotor.set(resultantrightspeed); // -err means right ahead, add -err to right speed
    return false; // not done yet
  }// Tick
}
