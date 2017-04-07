package org.usfirst.frc.team4069.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into to a variable name. This provides flexibility changing wiring, makes checking the wiring easier and significantly reduces the number of magic numbers floating around.
 */
public class IOMapping
{
  // Drive motor PWM ports
  public static final int LEFT_DRIVE_MOTOR_PWM_PORT = 8; // red
  public static final int RIGHT_DRIVE_MOTOR_PWM_PORT = 9; // green

  public static final int ELEVATOR_PWM_PORT = 2; // green/blue band

  public static final int FEED_PWM_PORT = 3; // yellow

  public static final int INTAKE_BACK_PWM_PORT = 4;
  public static final int INTAKE_FRONT_PWM_PORT = 5;

  public static final int WINCH_PWN_PORT = 6; // blue

  public static final int SHOOTER_CANBUS_PORT = 0;
  public static final int TURRET_CANBUS_PORT = 1;

  public static final int TURRET_LIMIT_SWITCH = 8;

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
  public static final int DRIVER_LEFT_BACK_BUTTON = 5;
  public static final int DRIVER_RIGHT_BACK_BUTTON = 6;
  public static final int DRIVER_SMALL_BACK_BUTTON = 7; // on front of controller, small black button labeled back
  public static final int DRIVER_SMALL_START_BUTTON = 8; // on front of controller small black start button
  public static final int DRIVER_RIGHT_Y_AXIS = 5;
  public static final int DRIVER_RIGHT_X_AXIS = 4;
  public static final int DRIVER_LEFT_Y_AXIS = 1;
  public static final int DRIVER_LEFT_X_AXIS = 0;
  public static final int DRIVER_DPAD = 0;

  public static final int CONTROL_Y_BUTTON = 4;
  public static final int CONTROL_A_BUTTON = 1;
  public static final int CONTROL_X_BUTTON = 3;
  public static final int CONTROL_B_BUTTON = 2;
  public static final int CONTROL_LEFT_BACK_BUTTON = 5;
  public static final int CONTROL_RIGHT_BACK_BUTTON = 6;
  public static final int CONTROL_SMALL_BACK_BUTTON = 7; // small black button on front of controller labeled 'back'
  public static final int CONTROL_SMALL_START_BUTTON = 8; // small black button on front of controller labeled start
  public static final int CONTROL_RIGHT_Y_AXIS = 5;
  public static final int CONTROL_RIGHT_X_AXIS = 4;
  public static final int CONTROL_LEFT_Y_AXIS = 1;
  public static final int CONTROL_LEFT_X_AXIS = 0;
  public static final int CONTROL_DPAD = 0;

}