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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
  private static final double MaxSpeed = 0.75; // 6 meters per second desired top speed
  private static final double MaxAngularRate = Math.PI; // Half a rotation per second max angular
                                                        // velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final Vision limelight1 = new Vision("limelight");

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2) // Add a 10%
                                                                                 // deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
                                                               // TODO: change this to closed
                                                               // loop velocity
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                           // forward
                                                                                           // with
                                                                                           // negative
                                                                                           // Y
                                                                                           // (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative
                                                            // X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                        // counterclockwise
                                                                        // with
                                                                        // negative X
                                                                        // (left)
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain.applyRequest(() -> point
        .withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  // public void updatePosEstimatorv1() {
  // double xystd;
  // double degstd;
  // double[] internaltag = limelight1.tagDetector();
  // double posdiff = drivetrain.getPoseDifference(limelight1.getPos2D());

  // if (internaltag[0] != -1) {
  // if (internaltag[1] > 1) {
  // xystd = 0.5;
  // degstd = 6;
  // }
  // // 1 target with large area and close to estimated pose
  // else if (internaltag[2] > 0.8 && posdiff < 0.5) {
  // xystd = 1.0;
  // degstd = 12;
  // }
  // // 1 target farther away and estimated pose is close
  // else if (internaltag[2] > 0.1 && posdiff < 0.3) {
  // xystd = 2.0;
  // degstd = 30;
  // }
  // // conditions don't match to add a vision measurement
  // else {
  // return;
  // }
  // } else {
  // return;
  // }

  // drivetrain.UpdateVision(limelight1.getPos2D(), xystd, degstd,
  // limelight1.getLatestLatencyAdjustedTimeStamp());

  // }


  public RobotContainer() {
    drivetrain.StartOdomThread();
    configureBindings();
    limelight1.init();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
