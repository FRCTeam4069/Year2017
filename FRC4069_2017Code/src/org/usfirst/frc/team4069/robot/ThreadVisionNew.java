package org.usfirst.frc.team4069.robot;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;
import org.usfirst.frc.team4069.robot.ThreadLIDAR.LidarSpot;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Iterator;

import javax.imageio.ImageIO;

import org.opencv.*;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

public class ThreadVisionNew implements Runnable
{
  public static final int MIN_WIDTH = 120;
  public static final int Y_IMAGE_RES = 240;
  public static final double VIEW_ANGLE = 34.8665269;
  public static final double AUTO_STEADY_STATE = 1.9;

  public static final int minR = 0, minG = 70, minB = 0;
  public static final int maxR = 95, maxG = 255, maxB = 255; // 75;

  public static final double minHRatio = 1.5, minVRatio = 1.5;
  public static final double maxHRatio = 6.6, maxVRatio = 8.5;

  public static final int MAX_SIZE = 255;
  public static final Scalar RED = new Scalar(0, 0, 255), BLUE = new Scalar(255, 0, 0), GREEN = new Scalar(0, 255, 0), ORANGE = new Scalar(0, 128, 255), YELLOW = new Scalar(0, 255, 255), PINK = new Scalar(255, 0, 255), WHITE = new Scalar(255, 255, 255);

  private boolean mDebug = false;
  private boolean mShowContours = true;

  private Target targets = new Target();

  private boolean mExitThread = false;
  private boolean mProcessFrames = true;

  private ThreadVideoCapture vcap_thread_instance;
  private Thread vcap_thread_handle;

  private static double xcenter = 0.0;
  private static double ycenter = 0.0;

  private double mapped = 0.0;
  public double lastHeadingTargetSeen = 0.0;

  public double lastYCenter = 0.0;
  public double lastXCenter = 0.0;
  public double lastMapped = 0.0;

  public double arrowangle=0.0;
  public int arrowdistance=0;
  
  // OpenCV constants
  public static final int CV_CAP_PROP_BRIGHTNESS = 10;
  public static final int CV_CAP_PROP_CONTRAST = 11;
  public static final int CV_CAP_PROP_EXPOSURE_ABSOLUTE = 39;
  public static final int CV_CAP_PROP_FRAME_WIDTH = 3;
  public static final int CV_CAP_PROP_FRAME_HEIGHT = 4;
  Preferences prefs = Preferences.getInstance();
  CvSource outputStream;
  public ColourRegions cregions = new ColourRegions();

  Robot robot;

  Scalar minRange, bminRange, cminRange;
  Scalar maxRange, bmaxRange, cmaxRange;
  LowPassFilter xLowPass;
  LowPassFilter yLowPass;

  public ThreadVisionNew(ThreadVideoCapture vidcapinstance, Thread vcap_handle, Robot irobot)
  {
    vcap_thread_instance = vidcapinstance; // to access getframe
    vcap_thread_handle = vcap_handle; // for thread control
    robot = irobot;

    minRange = new Scalar(240, 239, 70); // 119); //51,70,122);
    maxRange = new Scalar(255, 255, 133); // 242,255,212);
    bminRange = new Scalar(194, 236, 28); // 31, 245, 194);
    bmaxRange = new Scalar(204, 252, 32); //// 32, 252, 204);
    cminRange = new Scalar(240, 239, 22); // 51,70,122);
    cmaxRange = new Scalar(255, 255, 46); // 242,255,212);
  }

  public void run()
  {
    Mat img = new Mat();
    Mat lastvalidimg = null;

    outputStream = CameraServer.getInstance().putVideo("ProcessorOutput", 640, 480);

    xLowPass = new LowPassFilter(200);
    yLowPass = new LowPassFilter(200);

    //cregions.addRange(70, 239, 240, 133, 255, 255);
    cregions.addRange(30, 198, 81, 133, 255, 255);
    cregions.addRange(22, 230, 101, 32, 255, 204); // 28, 236, 194, 32, 252, 204);
    cregions.addRange(22, 239, 240, 46, 255, 255);
    // cregions.addRange(240, 240, 240, 255, 255, 255);
    while ((true) && (mExitThread == false))
    {
      if (mProcessFrames)
      {
        img = vcap_thread_instance.GetFrame();

        if (img != null)
        {
          cregions.CalcAll(img);
          cregions.DrawAll(img);
          lastvalidimg = img;

        } // if img!=null
        if (lastvalidimg != null)
        {
          outputStream.putFrame(lastvalidimg);
        }
      } // if processframes true
      try
      {
        Thread.sleep(50);
      }
      catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    } // while true
  } // run

  private void CalculateDist()
  {
    double targetHeight = 32;
    if (targets.VerticalTarget != null)
    {
      int height = targets.VerticalTarget.height;
      targets.targetDistance = Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
    }
  }

  private double Lerp(double a1, double a2, double b1, double b2, double num)
  {
    return ((num - a1) / (a2 - a1)) * (b2 - b1) - b2;
  }

  // ----------------------------------------------------------------

  private class ColourRange
  {
    public Scalar cmin;
    public Scalar cmax;

    public ColourRange(int minr, int ming, int minb, int maxr, int maxg, int maxb)
    {
      cmin = new Scalar(minb, ming, minr);
      cmax = new Scalar(maxb, maxg, maxr);
    }
  }// class ColourRange

  public class ColourRegions
  {
    double mMinAreaAllowed = 49.0;
    public ArrayList<ColourRange> mRange = new ArrayList<ColourRange>();
    public ArrayList<Mat> mThresholds = new ArrayList<Mat>();
    private MatOfInt4 mHierarchy = new MatOfInt4();
    public ArrayList<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

    public MatOfPoint mLargestContour = null;
    public double mLargestContourArea = 0.0;
    public double mXCenter = 0.0;
    public double mYCenter = 0.0;
    public double mXGreenLine = 160.0;
    public double mYGreenLine = 120.0;

    ColourRegions()
    {
    }

    /**
     * Add min/max colour range for finding target.  This function can be called multiple times to add
     * more ranges, though it costs a bit in time for each.
     * @param minr
     * @param ming
     * @param minb
     * @param maxr
     * @param maxg
     * @param maxb
     */
    public void addRange(int minr, int ming, int minb, int maxr, int maxg, int maxb)
    {
      mRange.add(new ColourRange(minr, ming, minb, maxr, maxg, maxb)); //Store range to threashold
      mThresholds.add(new Mat()); //each range needs a threshold Mat to store into...one for one here.
    }

    public void setMinAreaAllowed(double ma)
    {
      mMinAreaAllowed = ma;
    }

    // Returns number of contours
    public int CalcAll(Mat img)
    {
      mContours.clear();
      for (int i = 0; i < mRange.size(); i++)
      {
        ColourRange wrange = mRange.get(i);
        Mat wthreshold = mThresholds.get(i);
        Core.inRange(img, wrange.cmin, wrange.cmax, wthreshold);
        Imgproc.blur(wthreshold, wthreshold, new Size(3, 3));
        ArrayList<MatOfPoint> tmpcontours = new ArrayList<MatOfPoint>();
        Imgproc.findContours(wthreshold, tmpcontours, mHierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        mContours.addAll(tmpcontours); //keep all contours from all colour ranges
      }

      // now clear out any contours smaller than minAreaAllowed 6x6=36 etc 9x9=81
      //Not sure this is a good idea...
      mXCenter = 0.0;
      mYCenter = 0.0;
      mLargestContourArea = 0.0;
      mLargestContour = null;
      int numremoved=0;  //count number removed...
      Iterator<MatOfPoint> it = mContours.iterator();
      while (it.hasNext())
      {
        MatOfPoint mop = (MatOfPoint) it.next();
        double area = Imgproc.contourArea(mop);
        if (area < mMinAreaAllowed)
        {
          it.remove();
          numremoved++;
        }
        else
        {
          //Area larger than min, calc center points, keep largeest area found
          mXCenter += mop.get(0, 0)[0] + mop.size().width / 2; // don't add to xcenter for removed contours
          mYCenter += mop.get(0, 0)[0] + mop.size().height / 2;
          if (area > mLargestContourArea)
          {
            mLargestContourArea = area; //if this is the largest found so far, remember its size
            mLargestContour = mop;      //and remember it.
          } // if
        } // else
      } // for mop

      int numContours = mContours.size(); // might have gotten rid of all!

      if (numContours == 0)
      {
        if (numremoved > 0)
        {
       //   System.out.println("CHANGE??? REMOVED ALL CONTOURS! Maybe lower min Currently:"+mMinAreaAllowed); //warn us, we may want to change min area
        }
        return 0;
      }

      mXCenter /= numContours; // average center
      mYCenter /= numContours;
      return numContours;
    }// CalcAll

    
    
    
    /**
     * DrawAll mContours is populated, now eliminate ones below area threshold, and draw outline of all contours which have an area greater than 50% of the largest contour Calculate a lowpass filter x coordinate
     * 
     * @param original
     */

    public void DrawAll(Mat original)
    {
      double xAverage = 0.0;
      double yAverage = 0.0;
      int avgCtr = 0;
      RotatedRect[] minRect = new RotatedRect[mContours.size()];

      avgCtr = 0;
      xAverage = 0.0;
      yAverage = 0.0;
      for (int i = 0; i < mContours.size(); i++)
      {
        MatOfPoint2f mop2f = new MatOfPoint2f(mContours.get(i).toArray());
        minRect[i] = Imgproc.minAreaRect(mop2f);
        double curarea = Imgproc.contourArea(mContours.get(i));

        double contourWeight = curarea / mLargestContourArea; // 0-1.0 for weight of this contour

        if ((mShowContours) && (contourWeight > .5))
        {
          Point[] rect_points = new Point[4];
          minRect[i].points(rect_points);

          for (int j = 0; j < 4; j++)
          {
            Imgproc.line(original, rect_points[j], rect_points[(j + 1) % 4], BLUE, 3);
            xAverage += rect_points[j].x;
            yAverage += rect_points[j].y;
            avgCtr++;
          }
        } // if visualize

        // Rect box = minRect[i].boundingRect();
        // Point center = new Point(box.x + box.width / 2, box.y + box.height / 2);
        // Point center2 = new Point(3 + (box.x + box.width / 2), 3 + (box.y + box.height / 2));
        // Imgproc.line(original, center, center2, YELLOW, 3);
      } // for i <contours.size
      if (mContours.size() > 0)
      {
        xAverage /= avgCtr;
        yAverage /= avgCtr;
        double nx = xLowPass.calculate(xAverage);
        double ny = yLowPass.calculate(yAverage);

        Point avgpt = new Point(nx, 0); // yAverage);
        Point avgpt2 = new Point(nx, 230); // yAverage+2); //give some size

        Point ypt = new Point(0, ny);
        Point ypt1 = new Point(320, ny);

        mXGreenLine = nx;
        mYGreenLine = ny;
        Point center = new Point(nx, ny);
        Imgproc.line(original, avgpt, avgpt2, GREEN, 1);
        Imgproc.line(original, ypt, ypt1, GREEN, 1);
        double head = robot.arduino_thread_instance.lastHeading;
        if (head != -1.0)
          lastHeadingTargetSeen = robot.arduino_thread_instance.lastHeading;
      }
      else
      {
        Point avgpt = new Point(mXGreenLine, 0); // use last seen location
        Point avgpt2 = new Point(mXGreenLine, 230); //
        Imgproc.line(original, avgpt, avgpt2, RED, 1); // draw in read if not recent
        Point ypt = new Point(0, mYGreenLine);
        Point ypt1 = new Point(320, mYGreenLine);
        Imgproc.line(original, ypt, ypt1, RED, 1);

      }
      Point centerbtm = new Point(0, 20);
      double head = robot.arduino_thread_instance.lastHeading;
      if (head != -1.0)
      {
        Imgproc.putText(original, "HEADING:" + robot.arduino_thread_instance.lastHeading, centerbtm, 0, 0.5, GREEN);
      }
      else
        Imgproc.putText(original, "HEADING:Err No Lock", centerbtm, 0, 0.5, RED);

      Point lastsn = new Point(50, 230);
      Imgproc.putText(original, "Target Last Heading:" + lastHeadingTargetSeen, lastsn, 0, 0.5, YELLOW);
      DrawArrow(160,120,arrowangle,arrowdistance/4.3,original);
      DrawLIDAR(160,120,1,original);
  
      Point cpt=new Point(160,120);
      Imgproc.putText(original, ""+arrowdistance+"cm", cpt, 0,0.5, GREEN);
    }// DrawAll

  }// ColourRegions class
  
  
  private void DrawLIDAR(int xp,int yp,double scale,Mat original)
  {
    LidarSpot ls = null;
    for(int i=0;i<robot.lidar_instance.history.length;i++)
    {
      ls = robot.lidar_instance.getHistoryPoint();
      LidarSpot ls2 = robot.lidar_instance.getHistoryPoint();
      
      double rad = ls.az * (Math.PI / 180);
      double rad2= ls2.az * (Math.PI/180);
      
      Vector2 vec = new Vector2(-Math.cos(rad),Math.sin(rad));
      Vector2 vec2= new Vector2(-Math.cos(rad2),Math.sin(rad2));
      
      vec.scale(ls.dist/4);
      vec2.scale(ls2.dist/4);
      
      Point dpt = new Point(xp+vec.x,yp+vec.y);
      Point dpt2= new Point(xp+vec2.x,yp+vec2.y+1);
      
      if ((ls.az >=269.5)&&(ls.az <= 270.5))
      {
        arrowangle = ls.az;
        arrowdistance=ls.dist;
      }
      
      if ((ls.az > 270-5)&&(ls.az < 270+5))
      {
        Imgproc.line(original,dpt,dpt2,GREEN,1);
      }
      else
        Imgproc.line(original,dpt,dpt2,RED,1);
    }
  }//DrawLIDAR
  

  private void DrawArrow(int xp, int yp, double dir, double scale, Mat original)
  {
    double rad = dir * (Math.PI / 180);
    double lrad = rad + 3.14159 / 5;
    double rrad = rad - 3.14159 / 5;

    Vector2 degvec = new Vector2(Math.cos(rad), Math.sin(rad)); // get vector pointing right
    Vector2 lradv = new Vector2(Math.cos(lrad), Math.sin(lrad));
    Vector2 rradv = new Vector2(Math.cos(rrad), Math.sin(rrad));
    lradv.scale(scale / 4);
    rradv.scale(scale / 4);
    ;
    degvec.scale(scale);
    Vector2 vtip = new Vector2(xp, yp);
    vtip.add(degvec);

    Vector2 lbit = vtip.clone();
    lbit.sub(lradv);
    ;

    Vector2 rbit = vtip.clone();
    rbit.sub(rradv);
    ;

    Point start = new Point(xp, yp);
    Point tip = new Point(vtip.x, vtip.y);
    Point lefthead = new Point(lbit.x, lbit.y);
    Point righthead = new Point(rbit.x, rbit.y);

    double csc = scale + scale * 0.15;
    Imgproc.circle(original, start, (int) csc, BLUE);

    Imgproc.line(original, start, tip, YELLOW, 2);
    Imgproc.line(original, tip, lefthead, YELLOW, 2);
    Imgproc.line(original, tip, righthead, YELLOW, 2);

  }


}// class VisionThreadNew

// notes:
// check out canny call might be useful
// http://docs.opencv.org/2.4/doc/tutorials/imgproc/shapedescriptors/find_contours/find_contours.html?highlight=findcontours
