// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision.MeasurementInfo;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Deployer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {

  // 6 meters per second desired top speed.
  private static final double kMaxSpeed = 2.0;

  // Half a rotation per second max angular velocity.
  private static final double kMaxAngularRate = Math.PI;

  // Vision - Limelight - initialization.
  public final Vision limelight1 = new Vision("limelight");

  // Subsystems initialization.
  private final Angler m_angler = new Angler();
  private final Deployer m_deployer = new Deployer();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final Flywheel m_flywheel = new Flywheel();

  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController m_controller = new CommandXboxController(0);
  public static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  private final SendableChooser<Command> autoChooser;
  private final BooleanSupplier shooterStateSupplier = () -> m_flywheel.isAtSpeed();
  private final BooleanSupplier noteStateSupplier = () -> m_indexer.noteDetected();

  // Swerve drive request initialization. Using FieldCentric request type.
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxSpeed * 0.2).withRotationalDeadband(kMaxAngularRate * 0.2) // 20% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control

  private final Telemetry m_logger = new Telemetry(kMaxSpeed);

  private void ConfigureCommands() {
    NamedCommands.registerCommand("Intake",
        m_deployer.deploy().until(() -> m_deployer.isDeployed()).withTimeout(2)
            .andThen(m_intake.intake()).alongWith(m_indexer.load()).until(noteStateSupplier)
            .andThen(m_angler.goToShoot()));
    NamedCommands.registerCommand("RetractDeployer", m_deployer.retract());
    NamedCommands.registerCommand("Shoot",
        Commands.sequence(Commands.waitUntil(shooterStateSupplier).andThen(m_indexer.eject()),
            Commands.waitSeconds(1), m_angler.goToLoad()));
  }

  /**
   * @brief Configure the controller bindings for teleop
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-m_controller.getLeftY() * kMaxSpeed)
            .withVelocityY(-m_controller.getLeftX() * kMaxSpeed)
            .withRotationalRate(-m_controller.getRightX())));

    // m_controller.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

    // m_controller.a().onTrue(Commands.runOnce(() -> m_trajectory
    // .generate(new Pose2d(new Translation2d(3, 5), Rotation2d.fromDegrees(90)))));

    // m_controller.a().whileTrue(m_drivetrain
    // .findAndFollowPath((new Pose2d(new Translation2d(3, 5.5), Rotation2d.fromDegrees(180)))));

    // m_controller.b().whileTrue(m_drivetrain.applyRequest(() -> m_point
    // .withModuleDirection(new Rotation2d(-m_controller.getLeftY(),
    // -m_controller.getLeftX()))));

    // deploy and intake
    m_controller.rightTrigger().whileTrue(NamedCommands.getCommand("Intake"));
    // retract deployer
    m_controller.rightBumper().onTrue(NamedCommands.getCommand("RetractDeployer"));
    // shoot
    m_controller.leftBumper().onTrue(NamedCommands.getCommand("Shoot"));

    // move to the Amp
    // m_controller.a().whileTrue(new MoveToPose(Field.getAmpLineupPose(),
    // m_drivetrain));

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
    ConfigureCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    configureBindings();
    limelight1.init();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Intake", m_intake);
    SmartDashboard.putData("Indexer", m_indexer);
    SmartDashboard.putData("Flywheel", m_flywheel);
  }

  /**
   * @brief Get the autonomous command to run
   * 
   * @return Command
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
