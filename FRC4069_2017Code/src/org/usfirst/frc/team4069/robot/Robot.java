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

  private ShooterControl mShooterController = new ShooterControl();

  Preferences prefs = Preferences.getInstance();
  double ming = 0.0;
  // operations
  Joystick driverStick = new Joystick(0); // set to ID 1 in DriverStation
  Joystick controlStick = new Joystick(1); // set to ID 2 in DriverStation
  
  
  @Override
  public void robotInit()
  {
    ming = prefs.getDouble("minG", 1.0);
  }

  public Robot()
  {
    // CameraServer camera = CameraServer.getInstance(); //CameraServer();
    // camera.startAutomaticCapture(0);
    // camera.startAutomaticCapture();

    ming = prefs.getDouble("minG", 1.0);
    System.out.println("MING = " + ming);

    // vcap.set(propId, value)
    myRobot.setExpiration(0.1);
    leftDriveEncoder = new Encoder(IOMapping.LEFT_DRIVE_ENCODER_1, IOMapping.LEFT_DRIVE_ENCODER_2);
    rightDriveEncoder = new Encoder(IOMapping.RIGHT_DRIVE_ENCODER_1, IOMapping.RIGHT_DRIVE_ENCODER_2);
   
    mShooterController.SetWantedRPM(1500);
    

    Thread thread = new Thread(new VisionThread());
    thread.start();

  }//Robot()



  @Override
  public void operatorControl()
  {
    myRobot.setSafetyEnabled(true);
    while (isOperatorControl() && isEnabled())
    {
      SendDataToSmartDashboard();
      // myRobot.tankDrive(leftStick, rightStick);
      myRobot.drive(0.8, VisionThread.xcenter);

      if (driverStick.getRawButton(1)) //only while button pressed
      {
        mShooterController.Enable();
      }
      else
        mShooterController.Disable();
      
      mShooterController.ShooterTick();  //give some love to the shooter
      
      Timer.delay(0.005); // wait for a motor update time
    } //while isEnabled
  } //operatorControl





  void SendDataToSmartDashboard()
  {
    System.out.println("xcenter = " + VisionThread.xcenter);
    // System.out.println("shooter encoder = "+shooterEncoder.get());
    SmartDashboard.putNumber("LEFTENCODER", leftDriveEncoder.get());
    SmartDashboard.putNumber("RIGHTENCODER", rightDriveEncoder.get());
    SmartDashboard.putNumber("SHOOTERENCODER", mShooterController.GetShooterPosition());
    SmartDashboard.putNumber("XCENTER",  VisionThread.xcenter);
  } //SendDataToSmartDashboard


} //class Robot
