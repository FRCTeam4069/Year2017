package org.usfirst.frc.team4069.robot;

import java.io.DataOutputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Date;

//good site for io stuff http://wpilib.screenstepslive.com/s/4485/m/13810/l/292213-c-java-porting-guide-2014-to-2015#FileStorage
///media/sda1 for usb flash drive also heard /U/ and /V/
///home/lvuser/Output.txt  to put file in  home directory


public class RecordEvents
{
  /*DataOutputStream mDataStream=null;
  FileOutputStream mFileStream=null;
  Robot mRobot=null;
  long mLastRecTime=0;
  String mFileName="nonedefined";
  
  //Constructor for class RecordEvents
  
  public RecordEvents(String filename,Robot rbt)
  {
    mRobot=rbt;
    mFileName=filename;      
  } //RecordEvents constructor
  
  
  //Read encoders and timestamp and save out to file.
  public boolean SaveEvent() 
  {
    if ((mFileStream==null)||(mDataStream==null))
    {
      return false;
    }
    
    
    long curtime = System.currentTimeMillis();
    
    long deltaT = curtime - mLastRecTime; 
  
    int leftcount = mRobot.leftSideEncoder.get(); //.getRaw();
    int rightcount = mRobot.rightSideEncoder.get(); //.getRaw();
    double leftmotorvalue = mRobot.leftDriveMotorTalon.get();
    double rightmotorvalue= mRobot.rightDriveMotorTalon.get();
    
    try
    {      
      mDataStream.writeLong(deltaT);
      mDataStream.writeInt(leftcount);
      mDataStream.writeInt(rightcount);
      mDataStream.writeDouble(leftmotorvalue);
      mDataStream.writeDouble(rightmotorvalue);
      
    }
    catch(IOException e)
    {
    }
    

    mLastRecTime=curtime;
    return true;
  }//RecordEvent  
  
  
  public boolean OpenRecording()
  {
    if ((mFileStream!=null)||(mDataStream!=null))
    {
        return false;  //fail, already open?!     
    }   
    
    mLastRecTime = System.currentTimeMillis();  //initialize this
    
    try
    {
      mFileStream = new FileOutputStream(mFileName);    
      mDataStream = new DataOutputStream(mFileStream);
    }
    catch(IOException e)
    {     	
    } 
    return true;
  }//OpenRecording
    
  
  
  public void CloseRecording()
  {
    try
    {
      mFileStream.close();
      mDataStream.close();
    }
    catch(IOException e)
    {}
    mFileStream=null;
    mDataStream=null;
  } //CloseRecording*/
} //RecordEvents
  