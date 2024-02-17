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
  private final Intake m_intake = new Intake();
  public final Indexer m_indexer = new Indexer();
  private final Flywheel m_shooter = new Flywheel();

  // Controller initialization.
  private final CommandXboxController m_controller = new CommandXboxController(0); // My joystick

  // Drivetrain initialization.
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain; // My drivetrain

  // Swerve drive request initialization. Using FieldCentric request type.
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
      // 10% drive deadband.
      .withDeadband(kMaxSpeed * 0.2)
      // 10% rotational deadband.
      .withRotationalDeadband(kMaxAngularRate * 0.2)
      // Using a velocity closed-loop request.
      // Closed loop output type is set in the module constants in TunerConstants.
      .withDriveRequestType(DriveRequestType.Velocity);

  // Swerve brake request initialization.
  private final SwerveRequest.SwerveDriveBrake m_brake = new SwerveRequest.SwerveDriveBrake();

  // Swerve point wheels at request initialization.
  private final SwerveRequest.PointWheelsAt m_point = new SwerveRequest.PointWheelsAt();

  // Telemetry - logger - initialization.
  private final Telemetry m_logger = new Telemetry(kMaxSpeed);

  /**
   * @brief Configure the controller bindings for teleop
   */
  private void configureBindings() {

    // Set the default command for the drivetrain.
    // Executes command periodically.
    m_drivetrain.setDefaultCommand(
        // Apply the swerve drive request.
        m_drivetrain.applyRequest(() -> m_drive
            // Set the desired velocity based on the y-axis left joystick.
            .withVelocityX(-m_controller.getLeftY() * kMaxSpeed)
            // Set the desired velocity based on the x-axis left joystick.
            .withVelocityY(-m_controller.getLeftX() * kMaxSpeed)
            // Input for field centric rotation. Refer to Swerve Request.
            .withRotationalRate(-m_controller.getRightX() * kMaxAngularRate)));

    // Brake.
    m_controller.a().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

    // Point wheels.
    m_controller.b().whileTrue(m_drivetrain.applyRequest(() -> m_point
        .withModuleDirection(new Rotation2d(-m_controller.getLeftY(), -m_controller.getLeftX()))));

    // intake a note
    m_controller.rightBumper().whileTrue(m_intake.intake());
    m_controller.rightBumper().whileFalse(m_intake.stop());

    // indexer unstuck
    m_controller.x().whileTrue(m_indexer.load());
    m_controller.x().whileFalse(m_indexer.stop());

    // shoot a note
    m_controller.b().whileTrue(m_indexer.eject());
    m_controller.b().whileFalse(m_indexer.eject());

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
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
