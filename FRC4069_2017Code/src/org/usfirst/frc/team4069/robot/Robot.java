package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.RobotDrive;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.vision.*;

public class Robot extends SampleRobot
{
  RobotDrive myRobot = new RobotDrive(0, 1); // class that handles basic drive
  private Encoder leftDriveEncoder;
  private Encoder rightDriveEncoder;
  
  //private CANTalon shooterEncoder;
  
  
  Preferences prefs= Preferences.getInstance();
  double ming=0.0;
  // operations
  Joystick leftStick = new Joystick(0); // set to ID 1 in DriverStation
  Joystick rightStick = new Joystick(1); // set to ID 2 in DriverStation
  VideoCapture vcap;
  // vcap.set(CV_CAP_PROP_CONTRAST, 0);

  public void robotInit()
  {
    ming = prefs.getDouble("minG", 1.0);
    
  }
  
  
  public Robot()
  {
    //CameraServer camera = CameraServer.getInstance(); //CameraServer();
    //camera.startAutomaticCapture(0);
   // camera.startAutomaticCapture();
    
    ming = prefs.getDouble("minG", 1.0);
    System.out.println("MING = "+ming);
    
    // vcap.set(propId, value)
    myRobot.setExpiration(0.1);
    leftDriveEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
    rightDriveEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);
    //shooterEncoder = new CANTalon(IOMapping.SHOOTER_ENCODER_DIO_1, IOMapping.SHOOTER_ENCODER_DIO_2);

    Thread thread = new Thread(new VisionThread());
    thread.start();

  }

  void SendDataToSmartDashboard()
  {
    System.out.println("xcenter = "+VisionThread.xcenter);
    //System.out.println("shooter encoder = "+shooterEncoder.get());
    SmartDashboard.putNumber("LEFTENCODER", leftDriveEncoder.get());
    SmartDashboard.putNumber("RIGHTENCODER", rightDriveEncoder.get());
    SmartDashboard.putNumber("SHOOTERENCODER", 33); //ooterEncoder.get());
    SmartDashboard.putNumber("XCENTER",44); //VisionThread.xcenter);
    
  }

  /**
   * Runs the motors with tank steering.
   */
  @Override
  public void operatorControl()
  {
    myRobot.setSafetyEnabled(true);
    while (isOperatorControl() && isEnabled())
    {
      SendDataToSmartDashboard();
      //myRobot.tankDrive(leftStick, rightStick);
      myRobot.drive(0.8, VisionThread.xcenter);
      
      Timer.delay(0.005); // wait for a motor update time
    }
 
  }
  
  public double getLeftEncoder() {
	  return leftDriveEncoder.get();
  }
  
  public double getRightEncoder() {
	  return rightDriveEncoder.get();
  }
  

}
