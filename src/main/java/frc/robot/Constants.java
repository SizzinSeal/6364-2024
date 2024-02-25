package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
  public class Intake {
    // motor ids
    public static final int kIntakeId = 20;
    public static final int kDeployerId = 18;
    // CAN bus name
    public static final String kIntakeBus = "rio";
    public static final String kDeployerBus = "rio";
    // motor inversion
    public static final InvertedValue kIntakeInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kDeployerInverted = InvertedValue.CounterClockwise_Positive;
    // controller constants
    public static final Slot0Configs kIntakeControllerConstants = new Slot0Configs().withKP(0.01).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kDeployerControllerConstants = new Slot0Configs().withKP(0.01).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // speed
    public static final double kIntakeSpeed = .8 * 12; // rotations per second
    // position
    public static final double kUpPosition = 0;
    public static final double kDownPosition = 18;
  }

  public class Shooter {
    // motor ids
    public static final int kUpperId = 14;
    public static final int kLowerId = 15;
    public static final int kAnglerId = 17;
    // motor CAN bus names
    public static final String kUpperBus = "rio";
    public static final String kLowerBus = "rio";
    public static final String kAnglerBus = "rio";
    // motor inversion
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kAnglerInverted = InvertedValue.Clockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants = new Slot0Configs().withKP(0.01).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kLowerControllerConstants = new Slot0Configs().withKP(0.01).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kAnglerControllerConstants = new Slot0Configs().withKP(0.01).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kUpperSpeed = .9 * 12; // rotations per second
    public static final double kLowerSpeed = .9 * 12; // rotations per second
    public static final double kAnglerSpeed = .9 * 12; // rotations per second
    // tolerances
    public static final double kUpperTolerance = 5; // rotations per second
    public static final double kLowerTolerance = 5; // rotations per second
    public static final double kAnglerTolerance = 5; // rotations per second
  }

  public class Indexer {
    // motor ids
    public static final int kMotorId = 19;
    public static final int kNoteDetectorPort = 0;
    // motor CAN bus names
    public static final String kMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
    // controller constants
    public static final Slot0Configs kMotorControllerConstants = new Slot0Configs().withKP(0.01).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);
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

    public static TrajectoryConfig K_TRAJECTORY_CONFIG = new TrajectoryConfig(kMaxLateralSpeed, kMaxLateralAcceleration)
        .setKinematics(RobotContainer.m_drivetrain.getKinematics()).setEndVelocity(0);
  }

  public class PathPlanner {
    public static final PIDConstants kTranslationalPIDConstants = new PIDConstants(4.0, 0.0, 0.5);
    public static final PIDConstants kRotationalPIDConstants = new PIDConstants(4.0, 0.0, 0.5);
    public static final double kMaxModuleSpeed = 4.5; // 4.5 m/s.
    public static final double kDriveBaseRadius = 0.4; // 0.4 m. Distance from robot center to
                                                       // furthest module.
    public static final ReplanningConfig kReplanningConfig = new ReplanningConfig(true, true);
  }

  public class Field {
    // Blue Amp Pos
    public static final double kBlueAmpX = 5;
    public static final double kBlueAmpY = 5;
    // Blue Amp Lineup Pos
    public static final double kBlueAmpLineupX = 3;
    public static final double kBlueAmpLineupY = 5;
    public static final double kBlueAmpLineupTheta = new Rotation2d(kBlueAmpX - kBlueAmpLineupX,
        kBlueAmpY - kBlueAmpLineupY).getRadians();
    // Red Amp Pos
    public static final double kRedAmpX = 5;
    public static final double kRedAmpY = 5;
    // Red Amp Lineup Pos
    public static final double kRedAmpLineupX = 14;
    public static final double kRedAmpLineupY = 5;
    public static final double kRedAmpLineupTheta = new Rotation2d(kRedAmpX - kRedAmpLineupX, kRedAmpY - kRedAmpLineupY)
        .getRadians();

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
