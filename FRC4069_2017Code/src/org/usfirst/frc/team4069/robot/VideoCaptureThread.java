/**
 * VideoCaptureThread Class
 */

package org.usfirst.frc.team4069.robot;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;

public class VideoCaptureThread implements Runnable
{
  public static final int VCT_USB = 0;
  public static final int VCT_IP = 1;

  // OpenCV constants
  public static final int CV_CAP_PROP_BRIGHTNESS = 10;
  public static final int CV_CAP_PROP_CONTRAST = 11;
  public static final int CV_CAP_PROP_EXPOSURE_ABSOLUTE = 39;
  public static final int CV_CAP_PROP_FRAME_WIDTH = 3;
  public static final int CV_CAP_PROP_FRAME_HEIGHT = 4;

  public boolean cameraConnected = false;

  public Mat[] FrameQueue = new Mat[4];
  public int FrameInIndex = 0;
  public int FrameOutIndex = 0;

  private Object InLock;
  private Object OutLock;

  private Mat frame; // current frame being read, dont use this

  int mKeepRunning = 1;
  int mCameraConnectionType = VCT_USB;
  boolean mEnabled = true;

  private VideoCapture vcap;

  public void run()
  {
    InitCapture();

    while (mKeepRunning == 1)
    {
      Capture();
    }
  }// run

  public static void main(String args[])
  {
    (new Thread(new VideoCaptureThread())).start();
  }// main

  /**
   * Enable reading frames
   */
  public void Enable()
  {
    mEnabled = true;
  }

  /**
   * Disable reading frames, sleeps a lot
   */
  public void Disable()
  {
    mEnabled = false;
  }

  /*
   * InitCapture
   */
  private void InitCapture()
  {
    vcap = new VideoCapture();
    switch (mCameraConnectionType)
    {
    case VCT_USB:
      SetupUSBCamera();
      break;
    case VCT_IP:
      SetupIPCamera("10.40.69.44"); // ip address of Axis camera
      break;
    }
  }// InitCapture

  /*
   * Read a frame into framequeue, update indexes allows upto 4 frames to be stored. IF no frames have been removed, update last frame location with new frame
   * 
   */
  private void Capture()
  {
    while ((true) && (mKeepRunning == 1))
    {
      if (mEnabled)
      {
        boolean state = vcap.read(FrameQueue[FrameInIndex]); // FrameInIndex is always 'safe' to write too by definition
        if (state == false)
        {
          System.out.println("FAILED reading frame");
        }
        else
        {
          int getcurin = FrameInIndex; // always safe to read...
          getcurin++; // update locally to new values
          getcurin &= 3;
          if (getcurin != FrameOutIndex) // if we are not bumping up against frames waiting to be read...
          {
            synchronized (InLock) // only lock writes, reading is always ok
            {
              FrameInIndex = getcurin; // write in single statement
            } // synchronized
          } // if curin != out
        } // else frame read successful
        // outputStream.putFrame(frame);

        try
        {
          Thread.sleep(5);// yeild some time 5ms == 200fps, no worries there
        }
        catch (InterruptedException e)
        {
          System.out.println("Sleep fail in Capture");
          e.printStackTrace();
          mKeepRunning = 0;
        } // catch
      } // if enabled
      else
      {
        try
        {
          Thread.sleep(250);// 4 times a sec check if we should run
        }
        catch (InterruptedException e)
        {
          System.out.println("Sleep fail in Capture");
          e.printStackTrace();
          mKeepRunning = 0;
        } // catch

      } //else Enabled
    } // while true
  }// Capture

  /*
   * Get next available frame, returns clone
   */
  public Mat GetFrame()
  {
    if (FrameOutIndex != FrameInIndex)
    {
      Mat frameout = FrameQueue[FrameOutIndex].clone(); // NOTE copy!
      int frametosend = FrameOutIndex; // safe to read...
      frametosend++;
      frametosend &= 3; // do manipulations locally

      synchronized (OutLock) // only lock write, reading is always safe
      {
        FrameOutIndex = frametosend; // update in single statment
      }
      return frameout;
    }
    return null; // no frames available if queue has been drained,
                 // we could keep last frame sent and resend it, but calling routine could do that
  }// GetFrame

  /*
   * Init USB camera
   */
  private void SetupUSBCamera()
  {
    int videoStreamAddress = 0;
    System.out.println("Trying to connect to Camera stream... at: " + videoStreamAddress);

    int count = 1;

    while ((!vcap.open(videoStreamAddress)) && (mKeepRunning == 1))
    {
      System.out.println("Error connecting to camera stream, retrying " + count);
      count++;
      try
      {
        Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    } // while !open

    vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);

    vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);

    vcap.set(CV_CAP_PROP_CONTRAST, 0);

    System.out.println(vcap.get(CV_CAP_PROP_FRAME_WIDTH));

    System.out.println(vcap.get(CV_CAP_PROP_FRAME_HEIGHT));
    cameraConnected = true;
    System.out.println("Successfully connected to USB Camera Stream");

  }

  /*
   * Sets up IP camera, must pass IP address as string ex 192.168.1.44
   */
  private void SetupIPCamera(String ip)
  {
    String videoStreamAddress = "http://" + ip + "/mjpg/video.mjpg";

    System.out.println("Trying to connect to Camera stream... at: " + videoStreamAddress);

    int count = 1;

    while ((!vcap.open(videoStreamAddress)) && (mKeepRunning == 1))
    {
      System.out.println("Error connecting to camera stream, retrying " + count);
      count++;
      try
      {
        Thread.sleep(1000);
      }
      catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    } // while !vcap.open
    cameraConnected = true;
    System.out.println("Successfully connected to IP Camera Stream");
  }// SetupIPCamera

}// VideoCaptureThread class
