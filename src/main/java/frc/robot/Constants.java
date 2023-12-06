// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// wpilib imports
import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * The DrivetrainConstants class is used for the drivetrain. For example,
   * maximum speed
   */
  public static class DrivetrainConstants {
    // units are in meters and radians
    public static final double WHEEL_RADIUS = 0.0508;
    public static final double WHEEL_DIAMETER = 2 * WHEEL_RADIUS;
    public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    public static final double MAX_LINEAR_SPEED = 3.0;
    public static final double MAX_LINEAR_ACCELERATION = 6.0;
    public static final double MAX_LINEAR_JERK = 12.0;
    public static final double MAX_ANGULAR_SPEED = Math.PI;
    public static final double MAX_ANGULAR_ACCELERATION = 2 * Math.PI;
    public static final double MAX_ANGULAR_JERK = 4 * Math.PI;
    public static final Translation2d FL_SWERVE_LOCATION = new Translation2d(-0.381, 0.381);
    public static final Translation2d FR_SWERVE_LOCATION = new Translation2d(0.381, 0.381);
    public static final Translation2d BL_SWERVE_LOCATION = new Translation2d(-0.381, -0.381);
    public static final Translation2d BR_SWERVE_LOCATION = new Translation2d(0.381, -0.381);
  }

  /**
   * The OperatorConstants class contains constants used wherever driver control
   * is involved
   */
  public static class OperatorConstants {
    public static final int PRIMARY_CONTROLLER_PORT = 0;
  }

  /**
   * The CANIDs class contains CAN IDs used by devices on the CAN bus
   */
  public static class CANIDs {
    public static final String CANBUS_NAME = "rio";
    public static final int FL_DRIVE_ID = 3;
    public static final int FL_STEER_ID = 7;
    public static final int FL_ENCODER_ID = 11;
    public static final int FR_DRIVE_ID = 1;
    public static final int FR_STEER_ID = 5;
    public static final int FR_ENCODER_ID = 9;
    public static final int BL_DRIVE_ID = 2;
    public static final int BL_STEER_ID = 6;
    public static final int BL_ENCODER_ID = 10;
    public static final int BR_DRIVE_ID = 4;
    public static final int BR_STEER_ID = 8;
    public static final int BR_ENCODER_ID = 12;
  }
}
