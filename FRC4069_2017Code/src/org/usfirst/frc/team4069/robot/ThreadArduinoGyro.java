package org.usfirst.frc.team4069.robot;

import java.io.UnsupportedEncodingException;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class ThreadArduinoGyro implements Runnable
{
  I2C mi2sdev;
  byte[] fromArduino = new byte[512];
  public int enabled = 1;
  byte[] toSend = new byte[1];
  byte[] toGet = new byte[10];
  public double lastHeading = -1;
  public String lastError = "";
  public String lastMessage = "";

  public ThreadArduinoGyro()
  {
    mi2sdev = new I2C(I2C.Port.kMXP, 0x51); // 4);
    lastMessage = "I2C Created";
  }

  public void run()
  {
    lastError = "Startup...";

    int bigctr = 0;

    while (enabled == 1)
    {
      toSend[0] = 1;
      toGet[0] = 111;
      mi2sdev.transaction(toSend, 1, toGet, 10);
      // toGet[12]=0;
      String rval;
      try
      {
        rval = new String(toGet, "UTF-8");
        lastMessage = rval;
        if (rval.isEmpty() == false)
        {
          double head = 0.0;
          try
          {
            head = Double.parseDouble(rval);
            if (head != lastHeading)
            {
              lastHeading = head;
              lastError = rval;
              // System.out.println("Heading CHANGE:" + head + " degrees");
            }
          }
          catch (Exception e)
          {
            // System.out.println("Exception: " + e.getMessage());
            lastError = "GyroERR(" + bigctr + "):" + e.getMessage();
            lastHeading = -1;
          }
        } // isempty?

      }
      catch (UnsupportedEncodingException e1)
      {
        // TODO Auto-generated catch block
        e1.printStackTrace();
      }
      try
      {
        Thread.sleep(100);
      }
      catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    } // while
  }// run()
}
