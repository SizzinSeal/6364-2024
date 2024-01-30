// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class RobotContainer {
  private static final double kMaxSpeed = 0.75; // 6 meters per second desired top speed
  private static final double kMaxAngularRate = Math.PI; // Half a rotation per second max angular
                                                         // velocity

  // subsystems
  private final Intake m_intake = new Intake();
  public final Indexer m_indexer = new Indexer();
  private final Flywheel m_shooter = new Flywheel();


  // Setting up bindings for necessary control of the swerve drive platform
  private final CommandXboxController m_controller = new CommandXboxController(0); // My joystick
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      .withDeadband(kMaxSpeed * 0.2).withRotationalDeadband(kMaxAngularRate * 0.2) // Add a 10%
      // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
                                                               // TODO: change this to closed
                                                               // loop velocity
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();
  private final Telemetry m_logger = new Telemetry(kMaxSpeed);

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
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate) // Drive
        // counterclockwise
        // with
        // negative X
        // (left)
        ));

    m_controller.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));
    m_controller.b().whileTrue(m_drivetrain.applyRequest(() -> m_point
        .withModuleDirection(new Rotation2d(-m_controller.getLeftY(), -m_controller.getLeftX()))));

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

    if (Utils.isSimulation()) {
      m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    m_drivetrain.registerTelemetry(m_logger::telemeterize);
  }

  public RobotContainer() {
    configureBindings();
    SmartDashboard.putData("Intake", m_intake);
    SmartDashboard.putData("Indexer", m_indexer);
    SmartDashboard.putData("Flywheel", m_shooter);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
