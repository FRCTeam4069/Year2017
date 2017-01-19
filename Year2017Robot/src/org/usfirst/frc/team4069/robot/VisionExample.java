package org.usfirst.frc.team4069.robot;


import java.util.List;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JLabel;

import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.util.ArrayList;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
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
import org.opencv.videoio.VideoCapture;

public class VisionExample {
	
	public static final int MIN_WIDTH = 120;
	
	public static final int Y_IMAGE_RES = 240;
	
	public static final double VIEW_ANGLE = 34.8665269;
	
	public static final double AUTO_STEADY_STATE = 1.9;
	
	public static final int minR = 0, minG = 80, minB = 0;
	
	public static final int maxR = 50, maxG = 255, maxB = 75;
	
	public static final double minHRatio = 1.5, minVRatio = 1.5;
	
	public static final double maxHRatio = 6.6, maxVRatio = 8.5;
	
	public static final int MAX_SIZE = 255;
	
	public static final Scalar 
			RED = new Scalar(0, 0, 255),
			BLUE = new Scalar(255, 0, 0),
			GREEN = new Scalar(0, 255, 0),
			ORANGE = new Scalar(0, 128, 255),
			YELLOW = new Scalar(0, 255, 255),
			PINK = new Scalar(255, 0,255),
			WHITE = new Scalar(255, 255, 255);
	
	private Target targets = new Target();
	
	private Mat frame = new Mat();
	
	private boolean progRun;
	
	private Thread videoCaptureThread;
	
	//private Window window;
	
	// OpenCV constantssss
	
    public static final int CV_CAP_PROP_BRIGHTNESS = 10;
    
    public static final int CV_CAP_PROP_CONTRAST = 11;
	
	public static final int CV_CAP_PROP_EXPOSURE_ABSOLUTE = 39;
	
    public static final int CV_CAP_PROP_FRAME_WIDTH = 3;
    
    public static final int CV_CAP_PROP_FRAME_HEIGHT = 4;
	
	public VisionExample(){ //Window window){
		
//		this.window = window;
		
		Mat img = new Mat(), thresholded = new Mat();
		
		videoCaptureThread = new Thread(new Runnable(){
			public void run(){
				videoCapture();
			}
		});
		
		videoCaptureThread.start();
		
		targets.matchStart = false;
		
		targets.validFrame = false;
		
		targets.hotLeftOrRight = 0;
		
		progRun = true;
		
		while(true){
			
			if(Params.Process && progRun){
				
				boolean frameEmpty;
				
				synchronized(frame){
					
					frameEmpty = frame.empty();
					
				}
				
				if(!frameEmpty){
					
					synchronized(frame){
						
						frame.copyTo(img);
						
					}
					
					thresholded = thresholdImage(img);
					
					synchronized(targets){
						
						findTarget(img, thresholded);
						
						CalculateDist();
						
						if(Params.Debug){
							
							System.out.println("Vert: " + targets.VertGoal);
							
							System.out.println("Horiz: " + targets.HorizGoal);
							
							System.out.println("Hot Goal: " + targets.HotGoal);
							
							System.out.println("Dist: " + targets.targetDistance);
							
							
						}
						
					}
					
				}
				
			}
			
			try{
				
				Thread.sleep(1);
				
			}
			catch(InterruptedException e){
				
				e.printStackTrace();
				
			}
			
		}
		
	}
	
	private void CalculateDist(){
		
		double targetHeight = 32;
		
		if(targets.VerticalTarget != null){
			
			int height = targets.VerticalTarget.height;
			
			targets.targetDistance = Y_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE * Math.PI / (180 * 2)));
			
		}
		
	}
	
	private double map(double a1, double a2, double b1, double b2, double num){
		
		return ((num - a1) / (a2 - a1)) * (b2 - b1) - b2;
		
	}
	
	private void findTarget(Mat original, Mat thresholded){
		
		MatOfInt4 hierarchy = new MatOfInt4();
		
		ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>();
		
		Imgproc.findContours(thresholded, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
		
		if(Params.Debug){
			
			System.out.println("Contours: " + contours.size());
			
			System.out.println("Hierarchy: " + hierarchy.size());
			
		}
		
		int contourMin = 6;
		
		ArrayList<MatOfPoint> contoursToRemove = new ArrayList<MatOfPoint>();
		
		double xcenter = 0;
		
		for(MatOfPoint mop: contours){
			
			xcenter += mop.get(0, 0)[0] + mop.size().width / 2;
			
			if(mop.size().width < contourMin && mop.size().height < contourMin){
				
				contoursToRemove.add(mop);
				
			}
			
		}
		
		xcenter /= contours.size();
		
		double mapped = map(0, 640, -1, 1, xcenter);
		
		System.out.println(mapped);
		
		for(MatOfPoint mop: contoursToRemove){
			
			contours.remove(mop);
			
		}
		
		RotatedRect[] minRect = new RotatedRect[contours.size()];
		
		//Mat drawing = Mat.zeros(original.size(), CvType.CV_8UC3);
		
		NullTargets();
		
		if(!contours.isEmpty() && !hierarchy.empty()){
			
			for(int i = 0; i < contours.size(); i++){
				
				MatOfPoint2f mop2f = new MatOfPoint2f(contours.get(i).toArray());
				
				minRect[i] = Imgproc.minAreaRect(mop2f);
				
				if(Params.Visualize){
					
					Point[] rect_points = new Point[4];
					
					minRect[i].points(rect_points);
					
					for(int j = 0; j < 4; j++){
						
						Imgproc.line(original, rect_points[j], rect_points[(j + 1) % 4], BLUE);
						
					}
					
				}
				
				Rect box = minRect[i].boundingRect();
				
				double WHRatio = box.width / ((double)box.height);
				
				double HWRatio = ((double)box.height) / box.width;
				
				//check if contour is vert, we use HWRatio because it is greater that 0 for vert target
				if ((HWRatio > minVRatio) && (HWRatio < maxVRatio))
				{
					targets.VertGoal = true;
					targets.VerticalTarget = box;
					targets.VerticalAngle = minRect[i].angle;
					targets.VerticalCenter = new Point(box.x + box.width / 2, box.y + box.height / 2);
					targets.Vertical_H_W_Ratio = HWRatio;
					targets.Vertical_W_H_Ratio = WHRatio;

				}
				//check if contour is horiz, we use WHRatio because it is greater that 0 for vert target
				else if ((WHRatio > minHRatio) && (WHRatio < maxHRatio))
				{
					targets.HorizGoal = true;
					targets.HorizontalTarget = box;
					targets.HorizontalAngle = minRect[i].angle;
					targets.HorizontalCenter = new Point(box.x + box.width / 2, box.y + box.height / 2);
					targets.Horizontal_H_W_Ratio = HWRatio;
					targets.Horizontal_W_H_Ratio = WHRatio;
				}

				if (targets.HorizGoal && targets.VertGoal)
				{
					targets.HotGoal = true;

					//determine left or right
					if (targets.VerticalCenter.x < targets.HorizontalCenter.x) //target is right
						targets.targetLeftOrRight = 1;
					else if (targets.VerticalCenter.x > targets.HorizontalCenter.x) //target is left
						targets.targetLeftOrRight = -1;

					targets.lastTargerLorR = targets.targetLeftOrRight;

				}
				
				if(Params.Debug){
					
					System.out.println("Contour: " + i);
					
					System.out.println("X: " + box.x);
					
					System.out.println("Y: " + box.y);
					
					System.out.println("Height: " + box.height);
					
					System.out.println("Width: " + box.width);
					
					System.out.println("Angle: " + minRect[i].angle);
					
					System.out.println("Ratio (W/H): " + WHRatio);
					
					System.out.println("Ratio (H/W): " + HWRatio);
					
					System.out.println("Area: " + (box.height * box.width));
					
				}
				
				Point center = new Point(box.x + box.width / 2, box.y + box.height / 2);
				
				Imgproc.line(original, center, center, YELLOW, 3);
				
				Imgproc.line(original, new Point(320/2, 240/2), new Point(320/2, 240/2), YELLOW, 3);
				
			}
			
		}
		else{
			
			System.out.println("No Contours");
			
			targets.targetLeftOrRight = 0;
			
		}
		
		if(Params.Visualize){
			
			BufferedImage toShow = matToImage(original);
			
			if(toShow != null){
				
				//window.getDisplayIcon().setImage(toShow);
				
				//window.repaint();
				
			}
			
		}
		
		synchronized(targets){
			
			if(!targets.matchStart){
				
				targets.hotLeftOrRight = targets.targetLeftOrRight;
				
			}
			
		}
		
	}
	
	private void NullTargets(){
		
		targets.HorizontalAngle = 0.0;
		targets.VerticalAngle = 0.0;
		targets.Horizontal_W_H_Ratio = 0.0;
		targets.Horizontal_H_W_Ratio = 0.0;
		targets.Vertical_W_H_Ratio = 0.0;
		targets.Vertical_H_W_Ratio = 0.0;
		targets.targetDistance = 0.0;
		targets.targetLeftOrRight = 0;
		targets.lastTargerLorR = 0;
		
		targets.HorizGoal = false;
		targets.VertGoal = false;
		targets.HotGoal = false;
		
	}
	
	private Mat thresholdImage(Mat original){
		
		Mat thresholded = new Mat();
		
		Core.inRange(original, new Scalar(minB, minG, minR), new Scalar(maxB, maxG, maxR), thresholded);
		
		Imgproc.blur(thresholded, thresholded, new Size(3, 3));
		
		return thresholded;
		
	}
	
	private void videoCapture(){
		
		if(Params.From_File){
			
			System.out.println("Java opencv doesn't have imread, sorry!");
			
		}
		else if(Params.From_Camera){
			
			VideoCapture vcap = new VideoCapture();
			
			if(Params.USB_Cam){
				
				int videoStreamAddress = 0;
				
				System.out.println("Trying to connect to Camera stream... at: " + videoStreamAddress);
				
				int count = 1;
				
				while(!vcap.open(videoStreamAddress)){
					
					System.out.println("Error connecting to camera stream, retrying " + count);
					
					count++;
					
					try{
						
						Thread.sleep(1000);
						
					}
					catch(InterruptedException e){
						
						e.printStackTrace();
						
					}
				}
				
				vcap.set(CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);
				
				vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);
				
				vcap.set(CV_CAP_PROP_CONTRAST, 0);
				
				System.out.println(vcap.get(CV_CAP_PROP_FRAME_WIDTH));
				
				System.out.println(vcap.get(CV_CAP_PROP_FRAME_HEIGHT));
				
			}
			else{
				
				String videoStreamAddress = "http://" + Params.CAMERA_IP + "/mjpg/video.mjpg";
				
				System.out.println("Trying to connect to Camera stream... at: " + videoStreamAddress);
				
				int count = 1;
				
				while(!vcap.open(videoStreamAddress)){
					
					System.out.println("Error connecting to camera stream, retrying " + count);
					
					count++;
					
					try{
						
						Thread.sleep(1000);
						
					}
					catch(InterruptedException e){
						
						e.printStackTrace();
						
					}
					
				}
				
			}
			
			System.out.println("Successfully connected to Camera Stream");
			
			synchronized(targets){
				
				targets.cameraConnected = true;
				
			}
			
			while(true){
				
				synchronized(frame){
					
					vcap.read(frame);
					
				}
				
				try{
					
					Thread.sleep(5);
					
				}
				catch(InterruptedException e){
					
					e.printStackTrace();
					
				}
				
			}
			
		}
		
	}
	
	private BufferedImage matToImage(Mat mat){
		
		MatOfByte buffer = new MatOfByte();
		
		Imgcodecs.imencode(".png", mat, buffer);
		
		BufferedImage image = null;
		
		try{
			image = ImageIO.read(new ByteArrayInputStream(buffer.toArray()));
		}
		catch(IOException e){
			e.printStackTrace();
		}
		
		return image;
		
	}
	
}
