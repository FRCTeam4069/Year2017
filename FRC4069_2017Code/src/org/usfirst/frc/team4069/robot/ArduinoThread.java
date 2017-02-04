package org.usfirst.frc.team4069.robot;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class ArduinoThread implements Runnable
{
  I2C mi2sdev;
  byte[] fromArduino = new byte[512];
  public int enabled = 1;

  public ArduinoThread()
  {
    mi2sdev = new I2C(I2C.Port.kMXP, 0x51); // 4);
  }

  public void run()
  {
    byte[] buff = "hellothere\n".getBytes();

    while (enabled == 1)
    {
      try
      {
        if (mi2sdev.transaction(buff, buff.length, fromArduino, 11) == false) // mi2sdev.readOnly(fromArduino, 3)==false) //mi2sdev.transaction(buff, 0, fromArduino, 11)==false)
        {
          String str = fromArduino.toString();
          System.out.println("From: " + str);
        }
        else
        {
          System.out.println("nothing?");
        }

      }
      catch (BufferUnderflowException be)
      {
        System.out.println("underflow?");
      }

      try
      {
        Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    } // while
  }// run()
}
