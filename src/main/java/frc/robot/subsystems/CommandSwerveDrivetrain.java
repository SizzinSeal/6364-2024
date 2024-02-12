package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.proto.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.autonomous.TrajectoryGen;
import me.nabdev.pathfinding.structures.Path;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private Path TPath;

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }

  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
      // Configure AutoBuilder last
      AutoBuilder.configureHolonomic(
          this::getPose, // Robot pose supplier
          this::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
          this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
          new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants
                                           // class
              new PIDConstants(4.0, 0.0, 0.5), // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.5), // Rotation PID constants
              4.5, // Max module speed, in m/s
              0.4, // Drive base radius in meters. Distance from robot center to furthest module.
              new ReplanningConfig(true, true) // Default path replanning config. See the API for the options here
          ),
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this // Reference to this subsystem to set requirements
      );
    }

    // Configure AutoBuilder last
    AutoBuilder.configureHolonomic(
        this::getPose, // Robot pose supplier
        this::seedFieldRelative, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(3.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(3.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public List<Translation2d> generatebezierPoints(Path path) {
    return PathPlannerPath.bezierFromPoses(path.asPose2dList());
  }

  public List<Pose2d> generatePosesFromBezierPoints(List<Translation2d> bezierPoints) {
    List<Pose2d> poses = new ArrayList<>();
    // Assuming you want to set a default orientation (facing in the x-direction)
    Rotation2d defaultOrientation = new Rotation2d();

    // Convert each Translation2d point to a Pose2d with default orientation
    for (Translation2d point : bezierPoints) {
      poses.add(new Pose2d(point, defaultOrientation));
    }

    return poses;
  }

  public PathPlannerPath GetPath(List<Translation2d> bezierPoints) {
    PathPlannerPath Path = new PathPlannerPath(bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path.
                                                                 // If using a differential
                                                                 // drivetrain, the angular
                                                                 // constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // Goal end state. You can set a
                                                          // holonomic rotation here. If using a
                                                          // differential drivetrain, the rotation
                                                          // will have no effect.
    );
    Path.preventFlipping = true;
    return Path;
  }

  @Override
  public SwerveDriveState getState() {
    // TODO Auto-generated method stub
    return super.getState();
  }

  /**
   * @brief get the scalar distance between the robot and a position
   */
  public double getPoseDifference(Pose2d pos) {
    return m_odometry.getEstimatedPosition().getTranslation().getDistance(pos.getTranslation());
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    applyRequest(
        () -> new SwerveRequest.FieldCentric.ApplyChassisSpeeds().withSpeeds(chassisSpeeds)
            .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagic));
    setControl(new SwerveRequest.FieldCentric.ApplyChassisSpeeds().withSpeeds(chassisSpeeds)
        .withDriveRequestType(DriveRequestType.Velocity).withSteerRequestType(SteerRequestType.MotionMagic));
  }

  /**
   * @brief get the scalar speed of the robot in meters per second
   * 
   * @return double
   */

  public SwerveDriveKinematics getKinematics() {
    return m_kinematics;
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public Twist2d getTwist2d() {
    return m_kinematics.toTwist2d();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  /**
   * @brief update the odometry with a vision measurement
   * 
   * @param pos       the position of the robot
   * @param xyStds    the standard deviation of the x and y measurements
   * @param degStds   the standard deviation of the angle measurement
   * @param timestamp the timestamp of the measurement
   */
  public void updateVision(Pose2d pos, double xyStds, double degStds, double timestamp) {
    m_odometry.addVisionMeasurement(pos, timestamp,
        VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
  }

  public Command findandfollowPath(Pose2d targpos) {
    return AutoBuilder.pathfindToPose(targpos, new PathConstraints(5, 2.5, 6.28, 3.14));
  }

  public Command followPath(PathPlannerPath path) {
    return AutoBuilder.followPath(path);
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }
}
