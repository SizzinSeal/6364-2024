// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Vision.MeasurementInfo;
import frc.robot.subsystems.Climber;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Deployer;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {

  // 6 meters per second desired top speed.
  private static final double kMaxSpeed = 4.0;

  // Half a rotation per second max angular velocity.
  private static final double kMaxAngularRate = 1.5 * Math.PI;

  // Vision - Limelight - initialization.
  public final Vision limelight1 = new Vision("limelight");

  // Subsystems initialization.
  private final Angler m_angler = new Angler();
  private final Deployer m_deployer = new Deployer();
  private final Intake m_intake = new Intake();
  public final Indexer m_indexer = new Indexer();
  private final Flywheel m_flywheel = new Flywheel();
  private final Climber m_climber = new Climber();

  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandXboxController m_secondary = new CommandXboxController(1);
  public static final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
  private final SendableChooser<Command> autoChooser;

  // Swerve drive request initialization. Using FieldCentric request type.
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05) // 20% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control

  private final Telemetry m_logger = new Telemetry(kMaxSpeed);

  // command to move the deployer down
  private final SequentialCommandGroup m_deployerDownCommand = new SequentialCommandGroup(m_deployer.down()
      .andThen(Commands.waitUntil(() -> m_deployer.isDeployed())).andThen(m_deployer.stop()));

  // command to move the deployer up
  private final SequentialCommandGroup m_deployerUpCommand = new SequentialCommandGroup(m_deployer.up()
      .andThen(Commands.waitUntil(() -> m_deployer.isRetracted())).andThen(m_deployer.stop()));

  // command to intake
  private final SequentialCommandGroup m_intakeCommand = new SequentialCommandGroup(
      m_intake.intake().alongWith(m_indexer.load()).alongWith(m_angler.goToLoad())
          .until(() -> m_indexer.noteDetected()).andThen(m_indexer.stop()).andThen(m_angler.goToShoot())
          .andThen(Commands.waitSeconds(0.2))
          .andThen(m_indexer.slowLoad().onlyIf(() -> !m_indexer.noteDetected()))
          .andThen(Commands.waitUntil(() -> m_indexer.noteDetected())).andThen(m_indexer.stop())
          .andThen(m_intake.stop()));

  // command to shoot
  private final SequentialCommandGroup m_shootCommand = new SequentialCommandGroup(
      m_flywheel.forwards().andThen(m_angler.goToShoot()).andThen(Commands.waitSeconds(1)).andThen(m_indexer.eject())
          .andThen(Commands.waitSeconds(1)).andThen(m_flywheel.stop()).andThen(m_indexer.stop())
          .andThen(m_angler.goToLoad()));

  // command to calibrate angler
  private final SequentialCommandGroup m_calibrateCommand = new SequentialCommandGroup(m_angler.setSpeed(-3))
      .andThen(Commands.waitUntil(() -> m_angler.getLimit()))
      .andThen(m_angler.setSpeed(5)).andThen(Commands.waitSeconds(0.2))
      .andThen(m_angler.setSpeed(-1))
      .andThen(Commands.waitUntil(() -> m_angler.getLimit()))
      .andThen(m_angler.setSpeed(0)).andThen(Commands.waitSeconds(1))
      .andThen(m_angler.zero());

  private final SequentialCommandGroup m_climberUp = new SequentialCommandGroup(m_climber.up()
      .andThen(Commands.waitUntil(() -> m_climber.isRetracted())).andThen(m_climber.stop()));

  // command to climb
  private final SequentialCommandGroup m_climberDown = new SequentialCommandGroup(m_climber.down()
      .andThen(Commands.waitUntil(() -> m_climber.isDeployed())).andThen(m_deployer.stop()));

  private final SequentialCommandGroup m_manualLoad = new SequentialCommandGroup(
      m_indexer.slowLoad().onlyIf(() -> !m_indexer.noteDetected())
          .andThen(Commands.waitUntil(() -> m_indexer.noteDetected())).andThen(m_indexer.stop())
          .andThen(m_intake.stop()));

  // auto routine
  private final SequentialCommandGroup m_autoRoutine = new SequentialCommandGroup(
      m_angler.goToShoot().alongWith(m_deployerDownCommand).andThen(m_flywheel.forwards())
          .andThen(Commands.waitSeconds(2)).andThen(m_indexer.eject())
          .andThen(Commands.waitSeconds(1)).andThen(m_indexer.stop()).andThen(m_angler.goToLoad())
          .andThen(m_flywheel.stop().alongWith(m_intake.intake()).alongWith(m_indexer.load())
              .alongWith(m_drivetrain.applyRequest(() -> m_drive.withVelocityX(1.5)
                  .withVelocityY(0)
                  .withRotationalRate(0)))
              .until(() -> m_indexer.noteDetected()))
          .andThen(m_indexer.stop()).andThen(m_intake.stop()).andThen(m_manualLoad).andThen(m_angler.goToShoot())
          .andThen(m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-1)
              .withVelocityY(0)
              .withRotationalRate(0)).withTimeout(3.0).andThen(m_shootCommand)));

  private void ConfigureCommands() {
    NamedCommands.registerCommand("DeployerDown", m_deployerDownCommand);
    NamedCommands.registerCommand("DeployerUp", m_deployerUpCommand);
    NamedCommands.registerCommand("IntakeCommand", m_intakeCommand);
    NamedCommands.registerCommand("ShootCommand", m_shootCommand);
    NamedCommands.registerCommand("CalibrateAngler", m_calibrateCommand);
    NamedCommands.registerCommand("ManualLoad", m_manualLoad);
    NamedCommands.registerCommand("ClimerDown", m_climberDown);
    NamedCommands.registerCommand("ClimberUp", m_climberUp);

  }

  /**
   * @brief Configure the controller bindings for teleop
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-m_controller.getLeftY() * kMaxSpeed)
            .withVelocityY(-m_controller.getLeftX() * kMaxSpeed)
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate)));

    // m_controller.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

    // m_controller.a().onTrue(Commands.runOnce(() -> m_trajectory
    // .generate(new Pose2d(new Translation2d(3, 5), Rotation2d.fromDegrees(90)))));

    // m_controller.a().whileTrue(m_drivetrain
    // .findAndFollowPath((new Pose2d(new Translation2d(3, 5.5),
    // Rotation2d.fromDegrees(180)))));

    // m_controller.b().whileTrue(m_drivetrain.applyRequest(() -> m_point
    // .withModuleDirection(new Rotation2d(-m_controller.getLeftY(),
    // -m_controller.getLeftX()))));

    m_controller.rightTrigger()
        .whileTrue(NamedCommands.getCommand("IntakeCommand"));
    m_controller.rightTrigger().onTrue(NamedCommands.getCommand("DeployerDown"));
    m_controller.rightTrigger()
        .onFalse(m_indexer.stop().alongWith(m_intake.stop()).andThen(NamedCommands.getCommand("DeployerUp")));
    // m_controller.rightBumper().whileTrue(NamedCommands.getCommand("DeployerUp"));

    m_controller.leftBumper()
        .whileTrue(NamedCommands.getCommand("ShootCommand"));
    m_controller.leftBumper().onFalse(m_flywheel.stop().andThen(m_indexer.stop()).andThen(m_angler.goToLoad()));

    // manual indexer load
    m_controller.b().onTrue(NamedCommands.getCommand("ManualLoad"));
    // reset robot position
    m_controller.a().onTrue(Commands.runOnce(() -> m_drivetrain.seedFieldRelative()));
    // outtake
    m_controller.x().whileTrue(Commands.runOnce(() -> m_indexer.setSpeed(-5)).andThen(() -> m_intake.setSpeed(-8))
        .andThen(NamedCommands.getCommand("DeployerDown")));
    m_controller.x().onFalse(m_intake.stop().andThen(m_indexer.stop()).andThen(NamedCommands.getCommand("DeployerUp")));

    // primary controller climber controls
    m_controller.povUp().whileTrue(m_climber.up());
    m_controller.povUp().onFalse(m_climber.stop());
    m_controller.povDown().whileTrue(m_climber.down());
    m_controller.povDown().onFalse(m_climber.stop());

    // secondary controller climber controls
    m_secondary.povUp().whileTrue(m_climber.up());
    m_secondary.povUp().onFalse(m_climber.stop());
    m_secondary.povDown().whileTrue(m_climber.down());
    m_secondary.povDown().onFalse(m_climber.stop());

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
    double lateralDeviation; // standard deviation of the x and y measurements
    double angularDeviation; // standard deviation of the angle measurement
    final MeasurementInfo internalTag = limelight1.tagDetector();
    final double posDiff = m_drivetrain.getPoseDifference(limelight1.getPos2D());
    // return if no tag detected
    if (internalTag.tagId == -1)
      return;
    // more than 1 tag in view
    if (internalTag.tagCount > 1) {
      lateralDeviation = 0.5;
      angularDeviation = 6;
    }
    // 1 target with large area and close to estimated pose
    else if (internalTag.tagArea > 0.8 && posDiff < 0.5) {
      lateralDeviation = 1.0;
      angularDeviation = 12;
    }
    // 1 target farther away and estimated pose is close
    else if (internalTag.tagArea > 0.1 && posDiff < 0.3) {
      lateralDeviation = 2.0;
      angularDeviation = 30;
    }
    // conditions don't match to add a vision measurement
    else
      return;
    // update the pose estimator
    m_drivetrain.addVisionMeasurement(limelight1.getPos2D(),
        limelight1.getLatestLatencyAdjustedTimeStamp(), VecBuilder.fill(lateralDeviation,
            lateralDeviation, Units.degreesToRadians(angularDeviation)));
  }

  /**
   * @brief Construct the container for the robot. This will be called upon
   *        startup
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
    SmartDashboard.putData("Deployer", m_deployer);
    SmartDashboard.putData("Angler", m_angler);
    SmartDashboard.putData("Deployer Down", m_deployer.down()
        .andThen(Commands.waitUntil(() -> m_deployer.isDeployed())).andThen(m_deployer.stop()));
    SmartDashboard.putData("Deployer Up", m_deployer.up()
        .andThen(Commands.waitUntil(() -> m_deployer.isRetracted())).andThen(m_deployer.stop()));
    SmartDashboard.putData("CalibrateAngler", m_calibrateCommand);
    SmartDashboard.putData("Climber Up", m_climberUp);
    SmartDashboard.putData("Climber Down", m_climberDown);
  }

  /**
   * @brief Get the autonomous command to run
   * 
   * @return Command
   */
  public Command getAutonomousCommand() {
    return m_autoRoutine;
  }
}
