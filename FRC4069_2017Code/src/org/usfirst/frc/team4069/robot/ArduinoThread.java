package org.usfirst.frc.team4069.robot;

import java.io.UnsupportedEncodingException;
import java.nio.BufferUnderflowException;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;

public class ArduinoThread implements Runnable
{
  I2C mi2sdev;
  byte[] fromArduino = new byte[512];
  public int enabled = 1;
  byte[] toSend = new byte[1];
  byte[] toGet = new byte[10];

  public ArduinoThread()
  {
    mi2sdev = new I2C(I2C.Port.kMXP, 0x51); // 4);
  }

  public void run()
  {
    byte[] buff = "hellothere\n".getBytes();

    while (enabled == 1)
    {
      toSend[0] = 1;
      toGet[0] = 111;
      System.out.println("Before:" + toGet[0]);
      mi2sdev.transaction(toSend, 1, toGet, 10);
      // toGet[12]=0;
      String rval;
      try
      {
        rval = new String(toGet,"UTF-8");
        System.out.println("After:" + rval); //toGet[0]+","+toGet[1]+toGet[2]+toGet[3]+toGet[4]);
      }
      catch (UnsupportedEncodingException e1)
      {
        // TODO Auto-generated catch block
        e1.printStackTrace();
      }
     

      /*
       * try { if (mi2sdev.transaction(buff, buff.length, fromArduino, 11) == false) // mi2sdev.readOnly(fromArduino, 3)==false) //mi2sdev.transaction(buff, 0, fromArduino, 11)==false) { String str = fromArduino.toString(); System.out.println("From: " + str); } else {
       * System.out.println("nothing?"); }
       * 
       * } catch (BufferUnderflowException be) { System.out.println("underflow?"); }
       */
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
