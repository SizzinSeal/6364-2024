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
    public static final Slot0Configs kIntakeControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kDeployerControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // positions (rotations)
    public static final double kUpPosition = 0;
    public static final double kDownPosition = 0.2; // TODO: find this value
    // speeds (rotations per second)
    public static final double kIntakeSpeed = 0; // TODO: find this value
    public static final double kDeployerMaxSpeed = 0; // TODO: find this value
    // acceleration (rotations per second squared)
    public static final double kDeployerAcceleration = 0; // TODO: find this value
    // ratios (driven/driver)
    public static final double kDeployerRatio = 90;
    public static final double kIntakeRatio = 1;
  }

  public class Shooter {
    // motor ids
    public static final int kUpperId = 14;
    public static final int kLowerId = 15;
    public static final int kAnglerId = 17;
    // sensor ids
    public static final int kLimitPort = 0; // TODO: find this value
    // motor CAN bus names
    public static final String kUpperBus = "rio";
    public static final String kLowerBus = "rio";
    public static final String kAnglerBus = "rio";
    // motor inversion TODO: find these values
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kAnglerInverted = InvertedValue.Clockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants = // TODO: tune this
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kLowerControllerConstants = // TODO: tune this
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kAnglerControllerConstants = // TODO: tune this
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // positions (in rotations)
    public static final double kMaxPosition = 0; // TODO: find this value
    public static final double kMinPosition = 0;
    public static final double kShootingPosition = 0; // TODO: find this value
    public static final double kLoadingPosition = 0; // TODO: find this value
    // speeds (in rotations per second)
    public static final double kUpperSpeed = 0; // TODO: find this value
    public static final double kLowerSpeed = 0; // TODO: find this value
    public static final double kAnglerMaxSpeed = 0; // TODO: find this value
    public static final double kAnglerManualSpeed = 0; // TODO: find this value
    public static final double kAnglerProbeSpeed = 0; // TODO: find this value
    // acceleration (in rotations per second squared)
    public static final double kUpperAcceleration = 0; // TODO: find this value
    public static final double kLowerAcceleration = 0; // TODO: find this value
    public static final double kAnglerAcceleration = 0; // TODO: find this value
    public static final double kAnglerManualAcceleration = 0; // TODO: find this value
    public static final double kAnglerProbeAcceleration = 0; // TODO: find this value
    // tolerances (in rotations)
    public static final double kUpperTolerance = 5; // TODO: find this value
    public static final double kLowerTolerance = 5; // TODO: find this value
    public static final double kAnglerTolerance = 5; // TODO: find this value
    // ratios (driven/driver)
    public static final double kUpperRatio = 1;
    public static final double kLowerRatio = 1;
    public static final double kAnglerRatio = 1; // in reality this is a 1500 to 1 gear ratio, but
    // for some reason TalonFX doesn't support gear ratios higher than 1000 to 1
  }

  public class Indexer {
    // motor ids
    public static final int kMotorId = 19;
    // sensor ids
    public static final int kNoteDetectorPort = 0; // TODO: find this value
    // motor CAN bus names
    public static final String kMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive; // TODO: find
                                                                                    // this value
    // controller constants
    public static final Slot0Configs kMotorControllerConstants = // TODO: tune this
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds (rotations per second)
    public static final double kSpeed = 0; // TODO: find this value
    // ratios
    public static final double kRatio = 1;
  }

  public class Drivetrain {
    // feedforward gains
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
