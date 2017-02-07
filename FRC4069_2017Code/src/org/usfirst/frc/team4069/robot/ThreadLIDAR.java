package org.usfirst.frc.team4069.robot;

import java.io.UnsupportedEncodingException;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.FlowControl;
import edu.wpi.first.wpilibj.SerialPort.WriteBufferMode;




public class ThreadLIDAR implements Runnable
{
  public double lastAngle = 0.0;
  public int lastSignalStrength = 0;
  public double lastDistance = 0.0;
  public int lastStatus = 0;
  // Note status 00=no error, 22=stopped to verify error, 55 = hardware trouble, 99 = resuming operation
  SerialPort mSerpt; // = new SerialPort(115200,SerialPort.Port.kOnboard);
  public String mDSResponse = "";
  public String mDXResponse = "";
  public String mMSResponse = "";
  public String mLRResponse = "";
  public String mLIResponse = "";
  public String mMIResponse = "";
  public String mIVResponse = "";
  public String mIDResponse = "";
  public String lastError = "";
  public String lastMessage = "";

  int mState = 0;

  int[] ulastPacket = new int[32]; // will hold unsigned values for last packet

  byte[] lastPacket = new byte[32];
  public LidarSpot[] history = new LidarSpot[500];
  public int historyIndex = 0;

  public ThreadLIDAR()
  {
  }

  public void run()
  {
    lastError = "Startup...";
    lastMessage = "Startup...";
    connectToSerial();
    boolean xx = true;
    byte[] retvals = new byte[32];
    // System.out.println("Resetting...");
    // doRRCmd();
    // doSleep(2000);
    System.out.println("Done reset, setting motor speed...");
    doMSCmd("02"); //
    doSleep(2000);
    System.out.println("MOTOR: " + mMSResponse);

    // Now loop till we get a ID which makes sense
    while (mState == 0)
    {
      doIDCmd();
      if (mIDResponse.contains("ID115200"))
      {
        lastMessage = mIDResponse;
        mState = 1;
      }
      else
      {
        System.out.println("ID fail len=" + mIDResponse.length());
      }
      doSleep(500);
    } // while

    doDSCmd(); // Now trigger stream...

    lastMessage = mDSResponse;

    // System.out.println("GOT DS HEADER: " + mDSResponse);

    int bigctr = 0;
    while (mState == 1)
    {
      while (mSerpt.getBytesReceived() < 7) // Wait for 7 bytes to be in buffer
      {
        doSleep(4);
      } // while not 7 bytes waiting...

      lastPacket = mSerpt.read(7); // Grab 7 bytes
      mSerpt.readString();
      bigctr++;
      lastMessage = "Read packet(" + bigctr + ")";

      // System.out.print("PACKET(" + bigctr + ": ");
      bigctr++;
      for (int i = 0; i < lastPacket.length; i++)
      {
        int pval = lastPacket[i];
        if (pval < 0)
          pval += 256;
        ulastPacket[i] = pval;
        // System.out.print("," + pval);
      }
      // System.out.println();

      double az = 0.0;
      int dist = 0;
      int ss = 0;
      ;
      int stat = 0;

      if (checkDSChecksum(ulastPacket) == 1) // double azimuth, int distance, int signalstrength, int status)
      {
        stat = ulastPacket[0];
        dist = ulastPacket[3] + (ulastPacket[4] << 8);
        ss = ulastPacket[5];

        int ang = (ulastPacket[2] << 8) + ulastPacket[1];
        az = 1.0 * ((ang >> 4) + ((ang & 15) / 16.0));
        LidarSpot ls = new LidarSpot(az,dist,ss,stat);
        history[historyIndex] = ls;
        historyIndex++;
        historyIndex %= 500;
        
        lastMessage = "LIDAR: az:" + az + ", dist:" + dist + ",sig:" + ss + ", stat:" + stat;
        lastError = "";
        // System.out.println("LIDAR: az:" + az + ", dist:" + dist + ",sig:" + ss + ", stat:" + stat);
        lastAngle = az;
        lastDistance = dist;
        lastSignalStrength = ss;
        lastStatus = stat;
      }
      else
      {
        lastError = "ERR LIDAR CHECKSUM FAIL packet " + bigctr;
        // System.out.println("ERR LIDARPACKET");
      }

      try
      {
        Thread.sleep(5);
      }
      catch (InterruptedException e)
      {
      }
    } // while mState==1

    if (mState == 99) // exit?
    {
      doDXCmd();
      mSerpt.reset();
      mSerpt.free();
    }
  }// run

  /**
   * Passed in a unsigned int packet, checks the checksum, returns 0 if fail, 1 if success
   * 
   * @param upkt
   * @return
   */
  int checkDSChecksum(int[] upkt)
  {
    int sum = 0;
    for (int i = 0; i < 6; i++)
    {
      int aval = upkt[i];
      sum += aval;
    }
    int val = sum % 255;

    int want = upkt[6];

    if (val != want)
    {
      lastError = "ChecksumFail Want:" + want + ", got:" + val;
      // System.out.println("ChecksumFail Want:" + want + ", got:" + val);
      return 0; // fail checksum
    }
    return 1;
  }// checkDSPacket

  /**
   * Pass in command like 'IV\r\n', will return response from the command
   */
  String doLIDARCommand(String cmd)
  {
    int numlf = 1;
    if ((cmd.contains("MS")) || (cmd.contains("LR")))
    {
      numlf = 2;
    }
    System.out.println("Docmd:" + cmd + " numlf=" + numlf + " length:" + cmd.length());
    int gotrsp = 0;
    int numsent = mSerpt.write(cmd.getBytes(), cmd.length());
    if (numsent != cmd.length())
    {
      System.out.println("Error not sent all bytes!");
    }
    // else
    // System.out.println("Sent all " + numsent + " bytes");

    byte[] response = new byte[32];

    gotrsp = readNonStreamingResponse(response, numlf); // -1 is timeout
    String resp = "NoResponse";
    try
    {
      resp = new String(response, "UTF-8");
    }
    catch (UnsupportedEncodingException e)
    {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    // System.out.println("LIDAR gotrsp=" + gotrsp + " Response:" + resp);
    return resp;
  }// doLIDARCommand

  /**
   * All packets returned from lidar end with LF (0x10) Except 2 which have LF in the middle too. We want to pull a response upto the end of packet, in streaming mode there could be quite a few packets queued up in the recieve buffer, lets get one at a time.
   * 
   * @param tothis[]
   *          array to store packet too
   * 
   *          in byte[] array must be 32 in length minimum
   */
  // we only want well/formed packets, ones ending in LF
  int readNonStreamingResponse(byte[] tothis, int numlf)
  {
    long lastlooptime = System.currentTimeMillis();
    int finished = 0;
    int idx = 0;
    byte[] abyte = new byte[1];
    int maxsize = tothis.length;
    int numlffnd = 0;
    // System.out.println("readnonstream response numlf = " + numlf);

    while ((finished == 0) && (idx < maxsize))
    {
      if (System.currentTimeMillis() - lastlooptime > 1000)
      {
        return -1;
      }
      lastlooptime = System.currentTimeMillis();

      // Now LF is the termination char, except for a copule command which have TWO LF in them, sigh.

      abyte = mSerpt.read(1);
      if (abyte.length > 0)
      {
        tothis[idx] = abyte[0];
        idx++;
        if (abyte[0] == 10)
        {
          numlffnd++;
          // System.out.println("GOTLF num=" + numlffnd);
          if (numlffnd == numlf)
          {
            finished = 1;
          }
          else
          {
            // System.out.println("ADDING _");
            tothis[idx - 1] = '_';
          }
        } // if LF found
      } // if abyte.len . 0
    } // while
    return idx;
  }// readResponse

  /**
   * Start Data Acquisition Immediate response: DSssSSLF ss=status,SS=sum Datablocks follow and continue until stopped format: SAADDsC S=synch/Error, aa=azimuth, DD=distance cm, s=signal strength, C=checksum
   */
  String doDSCmd()
  {
    mDSResponse = doLIDARCommand("DS\r\n");
    return mDSResponse;
  }

  /**
   * Stop data acquisition Stops outputting data returns ex: DXssSSlf ss=status, SS=sum
   */
  String doDXCmd()
  {
    mDXResponse = doLIDARCommand("DX\r\n");
    return mDXResponse;
  }

  /**
   * Adjust Motor Speed Change rotation speed, valid values 00-10 must have 2 digits!
   * 
   * @param speed
   *          2 digit string 00-10 Return example: MShhLFssSSLF hh=speed hz, ss=status, SS=sum
   */
  String doMSCmd(String speed)
  {
    mMSResponse = doLIDARCommand("MS" + speed + "\r\n");
    return mMSResponse;
  }

  /**
   * Adjust Lidar Sample Rate Change sample rate valid values 01=500-600hz, 02=750-800hz, 03=1000-1075hz
   * 
   * @param val
   *          01,02 or 03 (500-600),(750-800),(1000-1075) respectively.
   * @return
   */
  String doLRCmd(String val)
  {
    mLRResponse = doLIDARCommand("LR" + val + "\r\n");
    return mLRResponse;
  }

  /**
   * LiDAR information Returns sample rate in ASCII returns: LIssLF ss=speed 01=500-600hz, 02=750-800hz 03-1000-1075hz
   */
  String doLICmd()
  {
    mLIResponse = doLIDARCommand("LI\r\n");
    return mLIResponse;
  }

  /**
   * Motor Information Returns: MIssLF ss=speed example: 05
   * 
   * @return
   */

  String doMICmd()
  {
    mMIResponse = doLIDARCommand("MI\r\n");
    return mMIResponse;
  }

  /**
   * Version Details Return: IVSWEEPppffhssssssssLF pp=protocol, ff=firmware version, h=hardwareversion, sssssss=serial number
   */
  String doIVCmd()
  {
    mIVResponse = doLIDARCommand("IV\r\n");
    return mIVResponse;
  }

  /**
   * Device Information Returns: ID115200LMDssrrLF L=laser state M=Mode D=diagnostic ss=motor speed (2 bytes) ex: 05 rr sample rate 4 bytes ex: 0500
   * 
   * @return
   */
  String doIDCmd()
  {
    mIDResponse = doLIDARCommand("ID\r\n");
    return mIDResponse;
  }

  /**
   * Reset scanner, has NO return value
   */
  String doRRCmd()
  {
    mSerpt.writeString("RR\r\n"); // reset device, no response
    return "no resp";
  }

  void connectToSerial()
  {
    SerialPort.Port lastUSBRetry = SerialPort.Port.kMXP;
    int success = 0;
    while (success == 0)
    {
      try
      {
        mSerpt = new SerialPort(115200, lastUSBRetry, 8, SerialPort.Parity.kNone, SerialPort.StopBits.kOne);
        success = 1;
      }
      catch (Exception e)
      {
        switch (lastUSBRetry)
        {
        case kMXP:
          lastUSBRetry = SerialPort.Port.kUSB;
          break;
        case kUSB:
          lastUSBRetry = SerialPort.Port.kUSB1;
          break;
        case kUSB1:
          lastUSBRetry = SerialPort.Port.kUSB2;
          break;

        case kUSB2:
          lastUSBRetry = SerialPort.Port.kMXP;
          break;
        } // switch
        lastError = e.getMessage() + ", retrying port " + lastUSBRetry;
        // System.out.println("SERIAL_ERR:" + e.getMessage() + ", retrying port " + lastUSBRetry);
        doSleep(1000);
      }
    }
    mSerpt.setFlowControl(FlowControl.kNone);
    // mSerpt.setWriteBufferMode(WriteBufferMode.kFlushOnAccess);
    mSerpt.setReadBufferSize(32767);
  }

  void disconnectFromSerial()
  {
    mSerpt.reset();
    mSerpt.flush();
    mSerpt.free();
  }

  void doSleep(long ms)
  {
    try
    {
      Thread.sleep(ms);
    }
    catch (InterruptedException e)
    {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

  }

  
  public class LidarSpot
  {
    public double az = 0.0;
    public int dist = 0;
    public int ss = 0;
    public int stat = 0;

    LidarSpot(double a,int dst,int sstr,int st)
    {
      az=a;
      dist =dst;
      ss = sstr;
      stat = st;
    }
  } //class Lidarspot
  
  
}// class
