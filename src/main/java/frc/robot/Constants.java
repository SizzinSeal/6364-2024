package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Constants {
  public class Drivetrain {
    // translation sysid constants
    public static final double kTranslationalRampRate = 0.5;
    public static final double kTranslationalStepVoltage = 7.0;
    public static final double kTranslationalTimeout = 15.0;
    // steer sysid constants
    public static final double kSteerRampRate = 0.5;
    public static final double kSteerStepVoltage = 7.0;
    public static final double kSteerTimeout = 15.0;
    // rotational sysid constants
    public static final double kRotationRampRate = 0.2;
    public static final double kRotationStepVoltage = 7.0;
    public static final double kRotationTimeout = 15.0;
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

  public class Angler {
    // motor ids
    public static final int kMotorId = 17;
    // motor CAN bus names
    public static final String kMotorBus = "rio";
    // sensor ports
    public static final int kLimitPort = 5;
    // motor inversion
    public static final InvertedValue kMotorInverted = InvertedValue.Clockwise_Positive;
    // gravity type
    public static final GravityTypeValue kGravityType = GravityTypeValue.Arm_Cosine;
    // controller constants
    public static final double kP = 50;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.28898;
    public static final double kV = 51.452;
    public static final double kA = 0.82497;
    public static final double kG = 0.20358;
    // positions (in rotations)
    public static final double kMaxPosition = 0.211111; // 76 degrees
    public static final double kMinPosition = 0.083333; // 30 degrees
    public static final double kZeroPosition = 0.2111111; // 76 degrees
    public static final double kShootingPosition = 0.16;
    public static final double kLoadingPosition = 0.09;
    // speeds (in rotations per second)
    public static final double kMaxSpeed = 0.2;
    public static final double kProbeFastSpeed = 3; // volts
    public static final double kProbeSlowSpeed = 1; // volts
    // acceleration (in rotations per second squared)
    public static final double kAcceleration = 1;
    // jerk (in rotations per second cubed)
    public static final double kJerk = 2;
    // tolerances (in rotations)
    public static final double kTolerance = 0.013889; // 5 degrees
    // ratios (driven/driver)
    public static final double kRatio = 500;
    // sysid constants
    public static final double kRampRate = 0.2;
    public static final double kStepVoltage = 8.0;
    public static final double kTimeout = 20.0;
  }

  public class Climber {
    // sim loop period (milliseconds)
    public static final double kSimLoopPeriod = 0.005;
    // motor ids
    public static final int kMotorId = 16;
    // CAN bus name
    public static final String kMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kMotorInverted = InvertedValue.Clockwise_Positive;
    // gravity type
    public static final GravityTypeValue kGravityType = GravityTypeValue.Elevator_Static;
    // controller constants
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    public static final double kG = 0;
    // positions (rotations) TODO: tune this so 90 degrees is up relative to the
    // ground
    public static final double kMaxPosition = 0;
    public static final double kMinPosition = 1;
    // speeds (rotations per second)
    public static final double kSpeed = 80; // TODO: find this value
    // acceleration (rotations per second squared)
    public static final double kAccel = 0; // TODO: find this value
    // jerk (rotations per second cubed)
    public static final double kJerk = 0; // TODO: find this value
    // tolerances (rotations)
    public static final double kTolerance = 0; // TODO: find this value
    // ratios (driven/driver)
    public static final double kRatio = 90;
    // SysId constants
    public static final double kRampRate = 5.0;
    public static final double kStepVoltage = 2.0;
    public static final double kTimeout = 5.0;
  }

  public class Deployer {
    // sim loop period (milliseconds)
    public static final double kSimLoopPeriod = 0.005;
    // motor ids
    public static final int kMotorId = 18;
    // CAN bus name
    public static final String kMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
    // gravity type
    public static final GravityTypeValue kGravityType = GravityTypeValue.Arm_Cosine;
    // controller constants
    public static final double kP = 20;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.33237;
    public static final double kV = 2.0077;
    public static final double kA = 1.4688;
    public static final double kG = 0.0;
    // positions (rotations) TODO: tune this so 90 degrees is up relative to the
    // ground
    public static final double kMaxPosition = 0.49;
    public static final double kMinPosition = 0;
    // speeds (rotations per second)
    public static final double kSpeed = 0.7; // TODO: find this value
    // acceleration (rotations per second squared)
    public static final double kAccel = 0.4; // TODO: find this value
    // jerk (rotations per second cubed)
    public static final double kJerk = 0; // TODO: find this value
    // tolerances (rotations)
    public static final double kTolerance = 0.1; // TODO: find this value
    // ratios (driven/driver)
    public static final double kRatio = 32.5;
    // current limit (we use this to limit accel)
    public static final double kCurrentLimit = 15;
    // SysId constants
    public static final double kRampRate = 0.2;
    public static final double kStepVoltage = 2.5;
    public static final double kTimeout = 20.0;
  }

  public class Flywheel {
    // sim loop period (seconds)
    public static final double kSimLoopPeriod = 0.005;
    // motor ids
    public static final int kUpperMotorId = 14;
    public static final int kLowerMotorId = 15;
    // motor CAN bus names
    public static final String kUpperMotorBus = "rio";
    public static final String kLowerMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kUpperMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kLowerMotorInverted = InvertedValue.CounterClockwise_Positive;
    // motor neutral modes
    public static final NeutralModeValue kUpperNeutralMode = NeutralModeValue.Brake;
    public static final NeutralModeValue kLowerNeutralMode = NeutralModeValue.Brake;
    // upper motor controller gains
    public static final double kUpperKP = 0;
    public static final double kUpperKI = 0;
    public static final double kUpperKD = 0;
    public static final double kUpperKS = 0.22748;
    public static final double kUpperKV = 0.11942;
    public static final double kUpperKA = 0.015106;
    // lower motor controller gains
    public static final double kLowerKP = 0;
    public static final double kLowerKI = 0;
    public static final double kLowerKD = 0;
    public static final double kLowerKS = 0.17641;
    public static final double kLowerKV = 0.10863;
    public static final double kLowerKA = 0.018805;
    // speeds (in rotations per second)
    public static final double kUpperSpeed = 100; // TODO: find this value
    public static final double kLowerSpeed = 100; // TODO: find this value
    // acceleration (in rotations per second squared)
    public static final double kUpperAccel = 0; // TODO: find this value
    public static final double kLowerAccel = 0; // TODO: find this value
    // tolerances (in rotations)
    public static final double kUpperTolerance = 20; // TODO: find this value
    public static final double kLowerTolerance = 20; // TODO: find this value
    // ratios (driven/driver)
    public static final double kUpperRatio = 1;
    public static final double kLowerRatio = 1;
    // sysid constants
    public static final double kRampRate = 0.2;
    public static final double kStepVoltage = 7.0;
    public static final double kTimeout = 15.0;
  }

  public class Indexer {
    // simulation loop period (seconds)
    public static final double kSimLoopPeriod = 0.005; // 5 ms
    // motor ids
    public static final int kMotorId = 19;
    // sensor ids
    public static final int kBeamBreakPort = 1;
    // motor CAN bus names
    public static final String kMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kInverted = InvertedValue.CounterClockwise_Positive;
    // controller gains
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    // speeds (rotations per second)
    public static final double kEjectSpeed = 3;
    public static final double kLoadSpeed = 3;
    public static final double kSlowLoadSpeed = 2.0;
    // ratios
    public static final double kRatio = 1;
  }

  public class Intake {
    // sim loop period (seconds)
    public static final double kSimLoopPeriod = 0.005;
    // sensor ids
    public static final int kBeamBreakPort = 0;
    // motor ids
    public static final int kMotorId = 20;
    // CAN bus name
    public static final String kMotorBus = "rio";
    // motor inversion
    public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
    // speeds (voltage)
    public static final double kIntakeSpeed = 8;
    public static final double kSlowIntakeSpeed = 5;
    public static final double kOuttakeSpeed = 8;
    // ratios (driven/driver)
    public static final double kRatio = 1;
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
