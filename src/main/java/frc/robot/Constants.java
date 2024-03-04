package frc.robot;

import com.ctre.phoenix6.signals.GravityTypeValue;
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
    public static final double kP = 1;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0.2539;
    public static final double kV = 0.11015;
    public static final double kA = 0.0010909;
    public static final double kG = 0;
    // positions (in rotations)
    public static final double kMaxPosition = 150;
    public static final double kMinPosition = 0;
    public static final double kShootingPosition = 130;
    public static final double kLoadingPosition = 20;
    // speeds (in rotations per second)
    public static final double kMaxSpeed = 120;
    public static final double kProbeInitialSpeed = 30;
    public static final double kProbeFinalSpeed = 10;
    // acceleration (in rotations per second squared)
    public static final double kAcceleration = 200;
    // jerk (in rotations per second cubed)
    public static final double kJerk = 800;
    // tolerances (in rotations)
    public static final double kTolerance = 0.013889; // 5 degrees
    // ratios (driven/driver)
    public static final double kRatio = 1; // in reality this is a 1500 to 1 gear ratio, but
    // for some reason TalonFX doesn't support gear ratios higher than 1000 to 1
    // sysid constants
    public static final double kRampRate = 0.2;
    public static final double kStepVoltage = 8.0;
    public static final double kTimeout = 20.0;
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
