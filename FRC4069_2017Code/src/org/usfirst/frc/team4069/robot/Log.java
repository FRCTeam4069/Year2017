package org.usfirst.frc.team4069.robot;

//Generic logging class, turn off debug to stop all logging to console etc.
public class Log
{
  static int mDebug = 1;

  public static void Log(String msg)
  {
    if (mDebug == 1)
    {
      long curtime = System.currentTimeMillis();
      System.out.println(curtime + " : " + msg);
    }
  }
}
