package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
  public class Intake {
    // motor ids
    public static final int kUpperMotorId = 14;
    public static final int kLowerMotorId = 15;
    // motor CAN bus names
    public static final String kUpperBusName = "";
    public static final String kLowerBusName = "";
    // motor inversion
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kLowerControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kUpperSpeed = .8 * 12; // rotations per second
    public static final double kLowerSpeed = .8 * 12; // rotations per second
  }

  public class Flywheel {
    // motor ids
    public static final int kUpperMotorId = 16;
    public static final int kLowerMotorId = 17;
    // motor CAN bus names
    public static final String kUpperBusName = "";
    public static final String kLowerBusName = "";
    // motor inversion
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kLowerControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kUpperSpeed = .9 * 12; // rotations per second
    public static final double kLowerSpeed = .9 * 12; // rotations per second
    // tolerances
    public static final double kUpperTolerance = 5; // revolutions per second
    public static final double kLowerTolerance = 5; // revolutions per second
  }

  public class Indexer {
    // ids
    public static final int kMotorId = 18;
    public static final int kNoteDetectorPort = 0;
    // motor CAN bus names
    public static final String kMotorBusName = "";
    // motor inversion
    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
    // controller constants
    public static final Slot0Configs kMotorControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kSpeed = .9 * 12; // rotations per second
  }

  public class Drivetrain {
    // Feedforward/feedforward gains
    // lateral position controller
    public static final double kLateralPositionP = 10;
    public static final double kLateralPositionD = 0;
    // angular position controller
    public static final double kAngularPositionP = 0.5;
    public static final double kAngularPositionD = 0;
    // velocity/acceleration constraints
    public static final double kMaxLateralSpeed = 5; // meters per second
    public static final double kMaxLateralAcceleration = 2; // meters per second squared
    public static final double kMaxAngularSpeed = 9.42477796077; // radians per second
    public static final double kMaxAngularAcceleration = 3.06998012384; // radians per second
                                                                        // squared
    // movement tolerances
    public static final double kLateralTolerance = 0.2; // meters
    public static final double kAngularTolerance = 0.2; // radians

    public static final double kBotWidth = 0.7; // meters
    public static final double kBotLength = 1; // meters

    public static TrajectoryConfig K_TRAJECTORY_CONFIG =
        new TrajectoryConfig(kMaxLateralSpeed, kMaxLateralAcceleration)
            .setKinematics(RobotContainer.m_drivetrain.getKinematics()).setEndVelocity(0);
  }

  public class Field {
    // Blue Amp Pos
    public static final double kBlueAmpX = 5;
    public static final double kBlueAmpY = 5;
    // Blue Amp Lineup Pos
    public static final double kBlueAmpLineupX = 3;
    public static final double kBlueAmpLineupY = 5;
    public static final double kBlueAmpLineupTheta =
        new Rotation2d(kBlueAmpX - kBlueAmpLineupX, kBlueAmpY - kBlueAmpLineupY).getRadians();
    // Red Amp Pos
    public static final double kRedAmpX = 5;
    public static final double kRedAmpY = 5;
    // Red Amp Lineup Pos
    public static final double kRedAmpLineupX = 14;
    public static final double kRedAmpLineupY = 5;
    public static final double kRedAmpLineupTheta =
        new Rotation2d(kRedAmpX - kRedAmpLineupX, kRedAmpY - kRedAmpLineupY).getRadians();

    public static Translation2d getAmpPos() {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red)
        return new Translation2d(kRedAmpX, kRedAmpY);
      else
        return new Translation2d(kBlueAmpX, kBlueAmpY);
    }

    public static Pose2d getAmpLineupPose() {
      if (DriverStation.getAlliance().isPresent()
          && DriverStation.getAlliance().get() == Alliance.Red)
        return new Pose2d(kRedAmpLineupX, kRedAmpLineupY, new Rotation2d(kRedAmpLineupTheta));
      else
        return new Pose2d(kBlueAmpLineupX, kBlueAmpLineupY, new Rotation2d(kBlueAmpLineupTheta));
    }
  }
}
