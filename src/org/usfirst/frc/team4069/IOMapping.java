package org.usfirst.frc.team4069.robot;

import edu.wpi.first.wpilibj.Joystick;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public interface IOMapping
{
  
  // Drive motor PWM ports
  public static final int LEFT_DRIVE_MOTOR_PWM_PORT = 0;
  public static final int RIGHT_DRIVE_MOTOR_PWM_PORT = 1;
  
  // Left and right arm canbus ports
  public static final int LEFT_ARM_CANBUS_PORT = 2;
  public static final int RIGHT_ARM_CANBUS_PORT = 1;
  
  // Shooter PWM port
  public static final int SHOOTER_PWM_PORT = 5;
  
  // Roller wheels PWM port
  public static final int ROLLERWHEELS_PWM_PORT = 6;
  
  // Intake PWM port
  public static final int INTAKE_PWM_PORT = 4;
  
  // Shooter encoder digital IO ports
  public static final int SHOOTER_ENCODER_DIO_1 = 4;
  public static final int SHOOTER_ENCODER_DIO_2 = 5;
  
  // Intake sensor digital IO
  public static final int INTAKE_SENSOR_DIO = 8;
  
  // Ready to shoot sensor digital IO
  public static final int READY_TO_SHOOT_DIO = 6;
  
  // Arm digital IO
  public static final int ARM_DIO = 7;
  
  // Compressor port
  public static final int COMPRESSOR_PORT = 0;
  
  // Compressor digital IO
  public static final int COMPRESSOR_DIO = 9;
  
  /**
   * Left encoder Digital I/O ports.
   */
  public static final int LEFT_DRIVE_ENCODER_1 = 0;
  public static final int LEFT_DRIVE_ENCODER_2 = 1;
  /**
   * Right encoder Digital I/O ports.
   */
  public static final int RIGHT_DRIVE_ENCODER_1 = 2;
  public static final int RIGHT_DRIVE_ENCODER_2 = 3;
  
  /**
   * Drive joystick port.
   */
  public static final int DRIVE_JOYSTICK_PORT = 0;
  /**
   * Shoot joystick port.
   */
  public static final int CONTROL_JOYSTICK_PORT = 1;

public static final int DRIVER_Y_BUTTON = 4;
public static final int DRIVER_A_BUTTON = 1;
public static final int DRIVER_X_BUTTON = 3;
public static final int DRIVER_B_BUTTON = 2;
public static final int DRIVER_RB_BUTTON = 6;

  
 public static final int CONTROL_Y_BUTTON = 4;
 public static final int CONTROL_A_BUTTON = 1;
 public static final int CONTROL_X_BUTTON = 3;
 public static final int CONTROL_B_BUTTON = 2;
 public static final int CONTROL_RB_BUTTON = 6;

 
}