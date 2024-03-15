package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.MotorLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.SimConstants;
import frc.robot.Constants.Drivetrain;
import frc.robot.Constants.PathPlanner;
import me.nabdev.pathfinding.structures.Path;

/**
 * Drivetrain Subsystem
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // sysid routine
  private final SwerveRequest.SysIdSwerveTranslation m_sysIdDrive =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SwerveRequest.SysIdSwerveSteerGains m_sysIdSteer =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation m_sysIdSwerveRotation =
      new SwerveRequest.SysIdSwerveRotation();
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  // sysid routine for characterizing swerve drive translation
  private final SysIdRoutine m_translationSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(Drivetrain.kTranslationalRampRate).per(Second),
          Volts.of(Drivetrain.kTranslationalStepVoltage),
          Seconds.of(Drivetrain.kTranslationalTimeout)),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        this.applyRequest(() -> m_sysIdDrive.withVolts(volts));
      }, log -> {
        getDriveMotorLog("Front Left", log, Modules[0]);
        getDriveMotorLog("Front Right", log, Modules[1]);
        getDriveMotorLog("Back Left", log, Modules[2]);
        getDriveMotorLog("Back Right", log, Modules[3]);
      }, this));
  // sysid routine for characterizing swerve module steer
  private final SysIdRoutine m_steerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(Drivetrain.kSteerRampRate).per(Second),
          Volts.of(Drivetrain.kSteerStepVoltage), Seconds.of(Drivetrain.kSteerTimeout)),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        this.applyRequest(() -> m_sysIdSteer.withVolts(volts));
      }, log -> {
        getSteerMotorLog("Front Left", log, Modules[0]);
        getSteerMotorLog("Front Right", log, Modules[1]);
        getSteerMotorLog("Back Left", log, Modules[2]);
        getSteerMotorLog("Back Right", log, Modules[3]);
      }, this));
  // sysid routine for characterizing swerve module steer
  private final SysIdRoutine m_rotationSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(Drivetrain.kSteerRampRate).per(Second),
          Volts.of(Drivetrain.kSteerStepVoltage), Seconds.of(Drivetrain.kSteerTimeout)),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        this.applyRequest(() -> m_sysIdSwerveRotation.withVolts(volts));
      }, log -> {
        getDriveMotorLog("Front Left", log, Modules[0]);
        getDriveMotorLog("Front Right", log, Modules[1]);
        getDriveMotorLog("Back Left", log, Modules[2]);
        getDriveMotorLog("Back Right", log, Modules[3]);
      }, this));

  /**
   * @brief Dynamic drive SysID routine
   * 
   * @param direction the direction of travel
   * @return Command
   */
  public final Command driveDynamic(SysIdRoutine.Direction direction) {
    return m_translationSysIdRoutine.dynamic(direction);
  }

  /**
   * @brief Quasistatic drive SysID routine
   * 
   * @param direction the direction of travel
   * @return Command
   */
  public final Command driveQuasistatic(SysIdRoutine.Direction direction) {
    return m_translationSysIdRoutine.quasistatic(direction);
  }

  /**
   * @brief Dynamic steer SysID routine
   * 
   * @param direction the direction of travel
   * @return Command
   */
  public final Command steerDynamic(SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutine.dynamic(direction);
  }

  /**
   * @brief Quasistatic steer SysID routine
   * 
   * @param direction the direction of travel
   * @return Command
   */
  public final Command steerQuasistatic(SysIdRoutine.Direction direction) {
    return m_steerSysIdRoutine.quasistatic(direction);
  }

  /**
   * @brief Dynamic rotation SysID routine
   * 
   * @param direction the direction of travel
   * @return Command
   */
  public final Command rotationDynamic(SysIdRoutine.Direction direction) {
    return m_rotationSysIdRoutine.dynamic(direction);
  }

  /**
   * @brief Quasistatic rotation SysID routine
   * 
   * @param direction the direction of travel
   * @return Command
   */
  public final Command rotationQuasistatic(SysIdRoutine.Direction direction) {
    return m_rotationSysIdRoutine.quasistatic(direction);
  }

  /**
   * @brief log the drive motor of a module
   * 
   * @param name Name of the module
   * @param log the log
   * @param module the module
   * @return MotorLog
   */
  private MotorLog getDriveMotorLog(String name, SysIdRoutineLog log, SwerveModule module) {
    return log.motor(name)
        .voltage(m_appliedVoltage
            .mut_replace(module.getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
        .angularPosition(
            m_angle.mut_replace(module.getDriveMotor().getPosition().getValueAsDouble(), Rotations))
        .angularVelocity(m_velocity.mut_replace(
            module.getDriveMotor().getVelocity().getValueAsDouble(), RotationsPerSecond));
  }

  /**
   * @brief log the drive motor of a module
   * 
   * @param name Name of the module
   * @param log the log
   * @param module the module
   * @return MotorLog
   */
  private MotorLog getSteerMotorLog(String name, SysIdRoutineLog log, SwerveModule module) {
    return log.motor(name)
        .voltage(m_appliedVoltage
            .mut_replace(module.getSteerMotor().get() * RobotController.getBatteryVoltage(), Volts))
        .angularPosition(
            m_angle.mut_replace(module.getSteerMotor().getPosition().getValueAsDouble(), Rotations))
        .angularVelocity(m_velocity.mut_replace(
            module.getSteerMotor().getVelocity().getValueAsDouble(), RotationsPerSecond));
  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    AutoBuilder.configureHolonomic(this::getPose, // Supply robot pose. See SwerveDrivetrain
                                                  // superclass.
        this::seedFieldRelative, // Reset odometry (only called if auto has a set starting
                                 // pose).
        this::getChassisSpeeds, // Supply robot relative chassis speed.
        this::driveRobotRelative, // Drive the robot given robot relative chassis speeds.
        new HolonomicPathFollowerConfig(PathPlanner.kTranslationalPIDConstants,
            PathPlanner.kRotationalPIDConstants, PathPlanner.kMaxModuleSpeed,
            PathPlanner.kDriveBaseRadius, PathPlanner.kReplanningConfig),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance.
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE.
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        }, this // Reference to this subsystem to set requirements.
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
    final PathPlannerPath path =
        new PathPlannerPath(bezierPoints, new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // constraints
            new GoalEndState(0.0, Rotation2d.fromDegrees(90)) // goal end state
        );
    return path;
  }

  @Override
  public SwerveDriveState getState() {
    // TODO Auto-generated method stub
    return super.getState();
  }

  /**
   * Calculate the scalar distance between the robot position and a given position.
   */
  public double getPoseDifference(final Pose2d pose) {
    try {
      return m_odometry.getEstimatedPosition().getTranslation().getDistance(pose.getTranslation());
    } catch (NullPointerException e) {
      System.out.println(e);
      return 0.0;
    }
  }

  public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
    applyRequest(() -> new SwerveRequest.FieldCentric.ApplyChassisSpeeds().withSpeeds(chassisSpeeds)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagic));
    setControl(new SwerveRequest.FieldCentric.ApplyChassisSpeeds().withSpeeds(chassisSpeeds)
        .withDriveRequestType(DriveRequestType.Velocity)
        .withSteerRequestType(SteerRequestType.MotionMagic));
  }

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
   * @param pos the position of the robot
   * @param xyStds the standard deviation of the x and y measurements
   * @param degStds the standard deviation of the angle measurement
   * @param timestamp the timestamp of the measurement
   */
  public void updateVision(Pose2d pos, double xyStds, double degStds, double timestamp) {
    m_odometry.addVisionMeasurement(pos, timestamp,
        VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
  }

  public Command findAndFollowPath(final Pose2d targetPose) {

    PathConstraints pathConstraints =
        new PathConstraints(Drivetrain.kMaxLateralSpeed, Drivetrain.kMaxLateralAcceleration,
            Drivetrain.kMaxAngularSpeed, Drivetrain.kMaxAngularAcceleration);


    if (DriverStation.getAlliance().equals(Alliance.Blue))
      return AutoBuilder.pathfindToPose(targetPose, pathConstraints);
    else
      return AutoBuilder.pathfindToPoseFlipped(targetPose, pathConstraints);
  }

  public Command followPath(final PathPlannerPath path, boolean fromfile) {

    if (DriverStation.getAlliance().isPresent() == false)
      return new Command() {};

    if (DriverStation.getAlliance().equals(Alliance.Blue))
      return AutoBuilder.followPath(path);
    else
      return AutoBuilder.followPath(path.flipPath());

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
    m_simNotifier.startPeriodic(SimConstants.kSimLoopPeriod);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }
}
