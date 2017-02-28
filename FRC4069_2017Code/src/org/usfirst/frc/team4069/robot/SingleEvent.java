package org.usfirst.frc.team4069.robot;

/*
 * SingleEvent is a single recorded event in a file.
 * It consists of 5 things.
 * 
 * a delta time, the amount of time since the last recorded event in milliseconds
 * a left and right encoder value, read from the wheels at this time.
 * a left and right motor power setting, also read at the time of recording.
 * 
 */

public class SingleEvent
{
  public long dTime;
  public int leftEncoderValue;
  public int rightEncoderValue;
  public double leftmotorsetting;
  public double rightmotorsetting;

  SingleEvent(long t, int le, int re, double leftmtrval, double rhtmtrval)
  {
    dTime = t;
    leftEncoderValue = le;
    rightEncoderValue = re;
    leftmotorsetting = leftmtrval;
    rightmotorsetting = rhtmtrval;

  }

  void setValues(long t, int le, int re, double leftmtr, double rhtmtr)
  {
    dTime = t;
    leftEncoderValue = le;
    rightEncoderValue = re;
    leftmotorsetting = leftmtr;
    rightmotorsetting = rhtmtr;
  }
}
