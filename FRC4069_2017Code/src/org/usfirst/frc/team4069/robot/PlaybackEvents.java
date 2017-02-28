package org.usfirst.frc.team4069.robot;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.Date;

/*
 * PlaybackEvents class
 * 
 * This class reads events from a file every time GetPlayEvent() is called, it will return a SingleEvent
 * filled with the 5 values for the event.
 */

public class PlaybackEvents
{
  DataInputStream mDataStream = null;
  FileInputStream mFileStream = null;

  long mLastRecTime = 0;
  String mFileName = "nonedefined";

  /*
   * PlaybackEvents Constructor Passed filename to play events from
   */

  public PlaybackEvents(String filename)
  {
    mFileName = filename;
  } // PlaybackEvents constructor

  // Get a SingleEvent object from playback file

  public SingleEvent GetPlayEvent()
  {
    if ((mFileStream == null) || (mDataStream == null))
    {
      return null;
    }

    long curtime = System.currentTimeMillis();
    SingleEvent se = new SingleEvent(0, 0, 0, 0, 0);
    try
    {
      long deltaT = mDataStream.readLong();
      int leftcount = mDataStream.readInt();
      int rightcount = mDataStream.readInt();
      double lefttalonval = mDataStream.readDouble();
      double righttalonval = mDataStream.readDouble();

      se.setValues(deltaT, leftcount, rightcount, lefttalonval, righttalonval);

      // Now add code to move motors till the encoders match left/right count

    }
    catch (IOException e)
    {
    }

    mLastRecTime = curtime;

    return se; // return the SingleEvent. It will be filled with 0's if an error was encountered.
  }// RecordEvent

  /*
   * OpenRecording opens the file, this must be called before calling GetPlayEvent
   */

  public boolean OpenRecording()
  {
    if ((mFileStream != null) || (mDataStream != null))
    {
      return false; // fail, already open?!
    }

    mLastRecTime = System.currentTimeMillis(); // initialzie

    try
    {
      mFileStream = new FileInputStream(mFileName);
      mDataStream = new DataInputStream(mFileStream);
    }
    catch (IOException e)
    {
    }
    return true;
  }// OpenRecording

  public void CloseRecording()
  {
    try
    {
      mFileStream.close();
      mDataStream.close();
    }
    catch (IOException e)
    {
    }
    mFileStream = null;
    mDataStream = null;
  } // CloseRecording
}
