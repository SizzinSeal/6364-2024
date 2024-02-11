// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision.MeasurementInfo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.autonomous.Trajectories;
import frc.robot.autonomous.TurnToPose;
import frc.robot.autonomous.Trajectories.TrajectoryFollower;
import frc.robot.Constants.Field;

public class RobotContainer {
  private static final double kMaxSpeed = 0.75; // 6 meters per second desired top speed
  private static final double kMaxAngularRate = Math.PI; // Half a rotation per second max angular
                                                         // velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public final Vision limelight1 = new Vision("limelight");

  // subsystems
  private final Intake m_intake = new Intake();
  public final Indexer m_indexer = new Indexer();
  private final Flywheel m_shooter = new Flywheel();
  private final TurnToPose turnToPose = new TurnToPose(m_drivetrain);

  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController m_controller = new CommandXboxController(0); // My joystick
  public static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  public static final Trajectories.TrajectoryGenerator m_trajectory =
      new Trajectories.TrajectoryGenerator();
  // My
  // drivetrain

  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxSpeed * 0.2).withRotationalDeadband(kMaxAngularRate * 0.2) // Add a 10%
      // deadband
      .withDriveRequestType(DriveRequestType.Velocity); // I want field-centric
                                                        // driving in open loop
                                                        // TODO: change this to closed
                                                        // loop velocity
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
  private final Telemetry m_logger = new Telemetry(kMaxSpeed);

  /**
   * @brief Configure the controller bindings for teleop
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-m_controller.getLeftY() * kMaxSpeed) // Drive
            // forward
            // with
            // negative
            // Y
            // (forward)
            .withVelocityY(-m_controller.getLeftX() * kMaxSpeed) // Drive left with negative
            // X (left)
            .withRotationalRate(-m_controller.getRightX()) // Drive
        // counterclockwise
        // with
        // negative X
        // (left)
        ));

    // m_controller.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

    m_controller.a().onTrue(Commands.runOnce(
        () -> m_trajectory.generate(new Pose2d(new Translation2d(3, 5), new Rotation2d(0)))));
    m_controller.b().whileTrue(
        new TrajectoryFollower(m_trajectory.getTrajectory(), new Rotation2d(0), m_drivetrain));

    // m_controller.b().whileTrue(m_drivetrain.applyRequest(() -> m_point
    // .withModuleDirection(new Rotation2d(-m_controller.getLeftY(), -m_controller.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_controller.leftBumper().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

    // intake a note
    m_controller.rightBumper()
        .onTrue(Commands.sequence(m_intake.intake(), m_indexer.load(), m_intake.stop()));
    // indexer unstuck
    m_controller.x()
        .onTrue(Commands.sequence(m_indexer.eject(), Commands.waitSeconds(0.5), m_indexer.load()));
    // load a note into the indexer
    m_controller.y().onTrue(m_indexer.load());
    // shoot a note
    m_controller.b().onTrue(Commands.sequence(m_shooter.forwards(), Commands.waitSeconds(3.0),
        m_indexer.eject(), Commands.waitSeconds(0.8), m_shooter.stop(), m_indexer.stop()));
    // move to the Amp
    // m_controller.a().whileTrue(new MoveToPose(Field.getAmpLineupPose(), m_drivetrain));

    // reset position if in simulation
    if (Utils.isSimulation()) {
      m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // register telemetry
    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  /**
   * @brief Update the pose estimator with vision measurements
   */
  public void updatePoseEstimator() {
    double xystd; // standard deviation of the x and y measurements
    double degstd; // standard deviation of the angle measurement
    final MeasurementInfo internalTag = limelight1.tagDetector();
    final double posdiff = m_drivetrain.getPoseDifference(limelight1.getPos2D());
    // return if no tag detected
    if (internalTag.tagId == -1)
      return;
    // more than 1 tag in view
    if (internalTag.tagCount > 1) {
      xystd = 0.5;
      degstd = 6;
    }
    // 1 target with large area and close to estimated pose
    else if (internalTag.tagArea > 0.8 && posdiff < 0.5) {
      xystd = 1.0;
      degstd = 12;
    }
    // 1 target farther away and estimated pose is close
    else if (internalTag.tagArea > 0.1 && posdiff < 0.3) {
      xystd = 2.0;
      degstd = 30;
    }
    // conditions don't match to add a vision measurement
    else
      return;
    // update the pose estimator
    m_drivetrain.addVisionMeasurement(limelight1.getPos2D(),
        limelight1.getLatestLatencyAdjustedTimeStamp(),
        VecBuilder.fill(xystd, xystd, Units.degreesToRadians(degstd)));
  }

  /**
   * @brief Construct the container for the robot. This will be called upon startup
   */
  public RobotContainer() {
    configureBindings();
    limelight1.init();
    SmartDashboard.putData("Intake", m_intake);
    SmartDashboard.putData("Indexer", m_indexer);
    SmartDashboard.putData("Flywheel", m_shooter);
  }

  /**
   * @brief Get the autonomous command to run
   * 
   * @return Command
   */
  // public Command getAutonomousCommand() {
  // return new MoveToPose(new Pose2d(5, 5, new Rotation2d(0)), m_drivetrain);
  // }
  public Command getAutonomousCommand() {

    return new TrajectoryFollower(m_trajectory.getTrajectory(), new Rotation2d(90), m_drivetrain);
  }

}
