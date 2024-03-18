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
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
  private static final double kMaxSpeed = 1.5;

  // Half a rotation per second max angular velocity.
  private static final double kMaxAngularRate = 1.0 * Math.PI;

  // Vision - Limelight - initialization.
  public final Vision limelight1 = new Vision("limelight");

  // Subsystems initialization
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

  // test request
  private final SwerveRequest.FieldCentricFacingAngle m_angleRequest = new SwerveRequest.FieldCentricFacingAngle()
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05)
      .withDriveRequestType(DriveRequestType.Velocity);

  // Swerve drive request initialization. Using FieldCentric request type.
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxSpeed * 0.05).withRotationalDeadband(kMaxAngularRate * 0.05) // 20% deadband
      .withDriveRequestType(DriveRequestType.Velocity); // closed loop velocity control

  private final Telemetry m_logger = new Telemetry(kMaxSpeed);

  // command to intake
  private final Command m_intakeCommand = Commands
      .sequence(m_angler.goToLoad(), m_deployer.deploy(), m_flywheel.forwards(),
          Commands.waitUntil(() -> m_deployer.isDeployed()), m_intake.intake(), m_indexer.load())
      .onlyIf(() -> !m_indexer.isNoteDetected());

  // command to intake a note which will be shot in the amp
  private final Command m_ampIntakeCommand = Commands.sequence(m_deployer.deploy(), m_angler.goToLoad(),
      Commands.waitUntil(() -> m_deployer.isDeployed()), m_intake.ampIntake(),
      Commands.waitUntil(() -> m_intake.isNoteDetected()), m_intake.ampLoad(),
      Commands.waitUntil(() -> !m_intake.isNoteDetected()),
      m_intake.stop(), m_deployer.toAmp(), Commands.waitSeconds(2.0))
      .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

  // command to shoot
  private final Command m_shootCommand = Commands.sequence(m_indexer.eject(), Commands.waitSeconds(1.0),
      m_indexer.stop());

  /**
   * @brief Poll the beam break sensors
   */
  public void pollBeamBreaks() {
    m_indexer.pollNoteDetector();
    m_intake.pollNoteDetector();
  }

  /**
   * @brief Configure Named Commands
   */
  private void ConfigureCommands() {
    NamedCommands.registerCommand("Intake", Commands.runOnce(() -> m_intakeCommand.schedule()));
    NamedCommands.registerCommand("Shoot", Commands.runOnce(() -> m_shootCommand.schedule()));
    NamedCommands.registerCommand("Deploy", Commands.runOnce(() -> m_deployer.deploy().schedule()));
    NamedCommands.registerCommand("Retract", Commands.runOnce(() -> m_deployer.retract().schedule()));
    NamedCommands.registerCommand("SpinUp", Commands.runOnce(() -> m_flywheel.forwards().schedule()));
    NamedCommands.registerCommand("ToShoot", Commands.runOnce(() -> m_angler.goToShoot().schedule()));
  }

  /**
   * @brief Configure the controller bindings for teleop
   */
  private void configureBindings() {
    // drivetrain control
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() -> m_drive.withVelocityX(-m_controller.getLeftY() * kMaxSpeed)
            .withVelocityY(-m_controller.getLeftX() * kMaxSpeed)
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate)));

    // note detected in the intake
    m_intake.noteDetected.onTrue(Commands.sequence(m_intake.slowIntake(),
        m_indexer.slowLoad()).onlyIf(() -> !m_indexer.isNoteDetected()));

    // note detected in the indexer
    m_indexer.noteDetected.onTrue(Commands.sequence(m_indexer.stop(),
        m_intake.stop(),
        m_deployer.retract(), m_angler.goToShoot()));

    // note left the indexer
    m_indexer.noteDetected.onFalse(Commands.sequence(m_indexer.stop(), m_angler.goToLoad()));

    // intake button pressed
    m_controller.rightTrigger().whileTrue(m_intakeCommand);
    // intake button released;
    m_controller.rightTrigger()
        .onFalse(Commands.sequence(m_intake.stop(), m_indexer.stop(),
            m_deployer.retract()));
    // shoot button pressed
    m_controller.leftBumper().onTrue(m_shootCommand);
    // amp intake button pressed
    m_controller.a().onTrue(m_ampIntakeCommand);
    // outtake
    m_controller.b().onTrue(
        Commands.sequence(m_intake.ampShoot(), Commands.waitSeconds(1.0), m_intake.stop(), Commands.waitSeconds(1.0),
            m_deployer.retract()));
    // reset angle
    m_controller.povUp().onTrue(Commands.runOnce(() -> m_drivetrain.seedFieldRelative()));
    // m_controller.povUp().onTrue(Commands.runOnce(() -> {
    // m_angleRequest.withTargetDirection(new Rotation2d(Degrees.of(0)));
    // m_drivetrain.setControl(m_angleRequest);
    // }));
    // m_controller.povRight().onTrue(Commands.runOnce(() -> {
    // m_angleRequest.withTargetDirection(new Rotation2d(Degrees.of(90)));
    // m_drivetrain.setControl(m_angleRequest);
    // }));
    // m_controller.povDown().onTrue(Commands.runOnce(() -> {
    // m_angleRequest.withTargetDirection(new Rotation2d(Degrees.of(180)));
    // m_drivetrain.setControl(m_angleRequest);
    // }));
    // m_controller.povLeft().onTrue(Commands.runOnce(() -> {
    // m_angleRequest.withTargetDirection(new Rotation2d(Degrees.of(270)));
    // m_drivetrain.setControl(m_angleRequest);
    // }));

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
    m_angleRequest.HeadingController.setP(3);
    m_angleRequest.HeadingController.setI(0);
    m_angleRequest.HeadingController.setD(0);
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
