package org.usfirst.frc.team4069.robot;

import java.util.ArrayList;
import java.util.Iterator;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt4;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Preferences;

public class ThreadVisionProcessor implements Runnable
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
  private double mapped = 0.0;

  public double lastXCenter = 0.0;
  public double lastMapped = 0.0;

  // OpenCV constants
  public static final int CV_CAP_PROP_BRIGHTNESS = 10;
  public static final int CV_CAP_PROP_CONTRAST = 11;
  public static final int CV_CAP_PROP_EXPOSURE_ABSOLUTE = 39;
  public static final int CV_CAP_PROP_FRAME_WIDTH = 3;
  public static final int CV_CAP_PROP_FRAME_HEIGHT = 4;
  Preferences prefs = Preferences.getInstance();
  CvSource outputStream;
  public ColourRegions cregions = new ColourRegions();

  Scalar minRange, bminRange, cminRange;
  Scalar maxRange, bmaxRange, cmaxRange;
  LowPassFilter xLowPass;
  LowPassFilter yLowPass;
  
  public ThreadVisionProcessor(ThreadVideoCapture vidcapinstance, Thread vcap_handle)
  {
    vcap_thread_instance = vidcapinstance; // to access getframe
    vcap_thread_handle = vcap_handle; // for thread control

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
    
    cregions.addRange(70, 239, 240, 133, 255, 255);
    cregions.addRange(28, 236, 194, 32, 252, 204);
    cregions.addRange(12, 200, 140, 46, 255, 255);

    while ((true) && (mExitThread == false))
    {
      if (mProcessFrames)
      {
        img = vcap_thread_instance.GetFrame();

        if (img != null)
        {
          if (cregions.CalcAll(img) > 0)
          {
            cregions.DrawAll(img);
          }
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
    double mMinAreaAllowed = 80.0;
    public ArrayList<ColourRange> mRange = new ArrayList<ColourRange>();
    public ArrayList<Mat> mThresholds = new ArrayList<Mat>();
    private MatOfInt4 mHierarchy = new MatOfInt4();
    public ArrayList<MatOfPoint> mContours = new ArrayList<MatOfPoint>();

    public MatOfPoint mLargestContour = null;
    public double mLargestContourArea = 0.0;
    public double mXCenter = 0.0;
    public double mXGreenLine = 0.0;

    void ColourRegions()
    {
    }

    public void addRange(int minr, int ming, int minb, int maxr, int maxg, int maxb)
    {
      mRange.add(new ColourRange(minr, ming, minb, maxr, maxg, maxb));
      mThresholds.add(new Mat());
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
        mContours.addAll(tmpcontours);
      }

      // now clear out any contours smaller than minAreaAllowed 6x6=36 etc 9x9=81
      mXCenter = 0.0;
      mLargestContourArea = 0.0;
      mLargestContour = null;
      Iterator<MatOfPoint> it = mContours.iterator();
      while (it.hasNext())
      {
        MatOfPoint mop = (MatOfPoint) it.next();
        double area = Imgproc.contourArea(mop);
        if (area < mMinAreaAllowed)
        {
          it.remove();
        }
        else
        {
          mXCenter += mop.get(0, 0)[0] + mop.size().width / 2; // don't add to xcenter for removed contours
          if (area > mLargestContourArea)
          {
            mLargestContourArea = area;
            mLargestContour = mop;
          } // if
        } // else
      } // for mop

      int numContours = mContours.size(); // might have gotten rid of all!

      if (numContours == 0)
        return 0;

      mXCenter /= numContours; // average center
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
        Point avgpt = new Point(nx, 0); // yAverage);
        Point avgpt2 = new Point(nx, 240); // yAverage+2); //give some size
        mXGreenLine = nx;

        Imgproc.line(original, avgpt, avgpt2, GREEN, 1);
      }
      else
      {
        Point avgpt = new Point(mXGreenLine, 0); // use last seen location
        Point avgpt2 = new Point(mXGreenLine, 240); //
        Imgproc.line(original, avgpt, avgpt2, RED, 1); // draw in read if not recent

      }
    }// DrawAll

  }// ColourRegions class

}
