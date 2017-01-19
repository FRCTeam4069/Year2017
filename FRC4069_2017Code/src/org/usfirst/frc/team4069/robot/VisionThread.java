package org.usfirst.frc.team4069.robot;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

public class VisionThread implements Runnable {

    public void run() {
        System.out.println("Hello from a thread!");
		VideoCapture vcap = new VideoCapture();
		
	   	
		while (!vcap.open(0)) //(videoStreamAddress, 320,240,7.5))
		{
			//std::cout << "Error connecting to camera stream, retrying " << count<< std::endl;
			//count++;
			//usleep(1000000);
		}
		
		vcap.set(39, 0.1);
		vcap.set(10,1); //, value)
		vcap.set(11, 0);
		Mat frame = new Mat();
		while (true)
		{
			vcap.read(frame);
			try 
			{
			    Thread.sleep(1000);                 //1000 milliseconds is one second.
			} catch(InterruptedException ex) 
			{
			    Thread.currentThread().interrupt();
			}		
		}
		//vcap.set( CV_CAP_PROP_EXPOSURE_ABSOLUTE, 0.1);
//		vcap.set(CV_CAP_PROP_BRIGHTNESS, 1);
		//vcap.set(CV_CAP_PROP_CONTRAST, 0);

		
    }

   
    
    public static void main(String args[]) {
        (new Thread(new VisionThread())).start();
    }

}

