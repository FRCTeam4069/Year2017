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
  private final double ERROR_SCALING_CONST_P = .1;

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
    double leftDistance = ((mControlMove.leftEncoder.get() * ControlMove.CM_PER_TICK) / ControlMove.CM_PER_INCH) / 12;
    double rightDistance = ((mControlMove.rightEncoder.get() * ControlMove.CM_PER_TICK) / ControlMove.CM_PER_INCH) / 12;
    double averageDistance = (leftDistance + rightDistance) / 2;

    if (Math.abs(averageDistance) >= mDistance)
    {
      mControlMove.leftDriveMotor.set(0);
      mControlMove.rightDriveMotor.set(0);
      return true;
    }

    /*
     * L        R        delta   fwd+       bkwd-
     * -10      0        -10     L++        R--(moreneg
     * 10      -10       20      R++        L--(moreneg 
     * -10     -5        -5      L++        R-- (moreneg
     * 10      5          5      R++        L--(moreneg
     */

    error = leftDistance - rightDistance; // if error > 0 left is ahead add - error to left
                                          // if error < 0 right is ahead add -error to right
    correctionFactor = error * ERROR_SCALING_CONST_P; // dampen error should come from ticks/wheelrotation data
    double abscorrFactor = Math.abs(correctionFactor) / 2;

    if (mSpeed > 0) //going forward
    {
      if (error > 0) //
      {
        resultantleftspeed = mSpeed - abscorrFactor; // left is ahead, slow it down by correctionFactor mspeed is >0
        resultantrightspeed = mSpeed + abscorrFactor; //R++
      }
      else
      {
        resultantrightspeed = mSpeed - abscorrFactor; // right is ahead, slow it down by correctionFactor
        resultantleftspeed = mSpeed + abscorrFactor; //L++
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
      if (error > 0) //bkwd(L--)
      {
        resultantleftspeed = mSpeed - abscorrFactor; // left is ahead, slow it down by correctionFactor mspeed is < 0
        resultantrightspeed = mSpeed + abscorrFactor;
      }
      else
      {
        resultantrightspeed = mSpeed - abscorrFactor; // right is ahead, slow it down by correctionFactor
        resultantleftspeed = mSpeed + abscorrFactor;
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
    mControlMove.leftDriveMotor.set(-resultantleftspeed); // +err means left ahead, subtract from left speed
    mControlMove.rightDriveMotor.set(resultantrightspeed); // -err means right ahead, add -err to right speed
    return false; // not done yet
  }// Tick
}
