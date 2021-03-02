// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  //drive train motor CAN addresses
  public final class DriveTrain {
    public static final int LEFT_MOTOR1_ID = 1;
    public static final int LEFT_MOTOR2_ID = 2;
    public static final int LEFT_MOTOR3_ID = 3;
    public static final int RIGHT_MOTOR1_ID = 4;
    public static final int RIGHT_MOTOR2_ID = 5;
    public static final int RIGHT_MOTOR3_ID = 6;
    public static final double SPEED = 1;
    public static final double SLOW_SPEED = .5;
    public static final double AUTONOMOUS_SPEED = 1; 
    public static final double DRIVE_FORWARD_TIME = 3;
  }
  public final class XboxController {
    public static final int LEFT_Y_AXIS = 0;
    public static final int LEFT_X_AXIS = 1;
    public static final int JOYSTICK_NUMBER = 0;
    public static final int ROLLER_AXIS = 3;
  }

  public final class Intake {
    public static final double EXTEND_ROLLER_SPEED = .25;
    public static final int SOLENOID_FORWARD = 2;
    public static final int SOLENOID_REVERSE = 5;
    public static final double DISLODGE_SPIN_EXTEND_TIME =.5;
    public static final double DISLODGE_ROLLERS_SPEED = 0.5;
    public static final double ROLLER_SPEED = 1;
    public static final int ROLLER_MOTOR_PORT = 3;
    public static final double DISLODGE_SPIN_REVERSE_TIME = .5;
    public static final double DISLODGE_SPIN_RETRACT_TIME = 1;
  }
  public final class Accumulator {
    public static final int ACCUMULATOR_MOTOR_PORT = 4;
    public static final double ACCUMULATOR_MOTOR_SPEED = 1;
  }
  public final class Kicker {
    public static final int KICKER_MOTOR_PORT = 5;
    public static final double KICKER_MOTOR_ADVANCE_SPEED = .25;
    public static final int KICKER_SENSOR_PORT = 3;
    public static final double KICKER_MOTOR_NUDGE_SPEED = 1;
  }
  public final class Shooter {
    public static final int SHOOTER_MOTOR_1_ID = 7;
    public static final int SHOOTER_MOTOR_2_ID = 8;
    public static final double SHOOTER_TARGET_SPEED = 4000;
      public class ShooterPID {
        public static final double P = 0.00105;
        public static final double I = 0;
        public static final double D = 0.06;
        public static final double FF = 0.00018;
      }
  }
  public final class Hood {
    public static final int HOOD_MOTOR_PORT = 6;
    public static final int HOOD_BOTTOM_STOP = 1;
    public static final int HOOD_TOP_STOP = 5;
    public static final int HOOD_POTENTIOMETER = 1;
  }
  public final class Limelight {
    public static final double LIMELIGHT_MOUNTING_ANGLE = 30; // degrees
    public static final double LIMELIGHT_MOUNTING_HEIGHT = 18; // inches
    public static final double LIMELIGHT_TARGET_HEIGHT = 98.25; // inches
  }
}