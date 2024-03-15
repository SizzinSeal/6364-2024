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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.SimConstants;
import me.nabdev.pathfinding.structures.Path;

/**
 * Drivetrain Subsystem
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {

  // Simulation.
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  // Sysid constants.
  public static final Measure<Velocity<Voltage>> kRampRate = Volts.of(0.2).per(Second);
  public static final Measure<Voltage> kStepVoltage = Volts.of(7.0);
  public static final Measure<Time> kTimeout = Seconds.of(15.0);

  // Sysid routine.
  private final VoltageOut m_sysIdOutput = new VoltageOut(0);
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(new SysIdRoutine.Config(kRampRate, kStepVoltage, kTimeout),
          new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
            for (int i = 0; i < Modules.length; i++) {
              Modules[i].getDriveMotor().setControl(m_sysIdOutput.withOutput(volts.in(Volts)));
            }
          }, log -> {
            log.motor("Front Left")
                .voltage(m_appliedVoltage.mut_replace(
                    Modules[0].getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(
                    Modules[0].getDriveMotor().getPosition().getValueAsDouble(), Rotations))
                .angularVelocity(m_velocity.mut_replace(
                    Modules[0].getDriveMotor().getVelocity().getValueAsDouble(),
                    RotationsPerSecond));
            log.motor("Front Right")
                .voltage(m_appliedVoltage.mut_replace(
                    Modules[1].getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(
                    Modules[1].getDriveMotor().getPosition().getValueAsDouble(), Rotations))
                .angularVelocity(m_velocity.mut_replace(
                    Modules[1].getDriveMotor().getVelocity().getValueAsDouble(),
                    RotationsPerSecond));
            log.motor("Back Left")
                .voltage(m_appliedVoltage.mut_replace(
                    Modules[2].getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(
                    Modules[2].getDriveMotor().getPosition().getValueAsDouble(), Rotations))
                .angularVelocity(m_velocity.mut_replace(
                    Modules[2].getDriveMotor().getVelocity().getValueAsDouble(),
                    RotationsPerSecond));
            log.motor("Back Right")
                .voltage(m_appliedVoltage.mut_replace(
                    Modules[3].getDriveMotor().get() * RobotController.getBatteryVoltage(), Volts))
                .angularPosition(m_angle.mut_replace(
                    Modules[3].getDriveMotor().getPosition().getValueAsDouble(), Rotations))
                .angularVelocity(m_velocity.mut_replace(
                    Modules[3].getDriveMotor().getVelocity().getValueAsDouble(),
                    RotationsPerSecond));
          }, this));

  /**
   * @brief Class constructor.
   */
  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);

    // Simulation.
    if (Utils.isSimulation()) {
      startSimThread();
    }

    // Autonomous builder.
    AutoBuilder.configureHolonomic(this::getPose, // Supply robot pose. See SwerveDrivetrain
                                                  // superclass.
        this::seedFieldRelative, // Reset odometry (only called if auto has a set starting
                                 // pose).
        this::getChassisSpeeds, // Supply robot relative chassis speed.
        this::driveRobotRelative, // Drive the robot given robot relative chassis speeds.
        new HolonomicPathFollowerConfig(Constants.PathPlanner.kTranslationalPIDConstants,
            Constants.PathPlanner.kRotationalPIDConstants, Constants.PathPlanner.kMaxModuleSpeed,
            Constants.PathPlanner.kDriveBaseRadius, Constants.PathPlanner.kReplanningConfig),
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

    PathConstraints pathConstraints = new PathConstraints(Constants.Drivetrain.kMaxLateralSpeed,
        Constants.Drivetrain.kMaxLateralAcceleration, Constants.Drivetrain.kMaxAngularSpeed,
        Constants.Drivetrain.kMaxAngularAcceleration);


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
