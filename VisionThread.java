package org.usfirst.frc.team4069.robot;

import org.opencv.videoio.VideoCapture;

public class VisionThread implements Runnable {

    public void run() {
        System.out.println("Hello from a thread!");
		VideoCapture vcap;
    }

    public static void main(String args[]) {
        (new Thread(new VisionThread())).start();
    }

}

