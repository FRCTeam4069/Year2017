package org.usfirst.frc.team4069.robot;
import edu.wpi.first.wpilibj.SerialPort;

public class SerialIO
{
  byte[] Enable = {'D','S',10};
  byte[] Disable = {'D','X',10};
  
  public SerialIO()
  {
    SerialPort s = new SerialPort(115200,SerialPort.Port.kOnboard,8,SerialPort.Parity.kNone,SerialPort.StopBits.kOne);
    s.setFlowControl(SerialPort.FlowControl.kNone);
    int angle_i=3;
    
    double angle_f = 1.0f * ((float)(angle_i >> 4) + ((angle_i & 15) / 16.0f));
    
    
    
  }//SerialIO constructor
  
}//SerialIO
