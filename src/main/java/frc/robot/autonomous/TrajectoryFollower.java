package frc.robot.autonomous;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TrajectoryFollower extends Command {

  private final Timer m_timer = new Timer();
  private final Trajectory m_trajectory;
  private final HolonomicDriveController m_controller;
  private final Rotation2d m_desiredRotation;
  private final CommandSwerveDrivetrain m_drivetrain;

  private final SwerveRequest.FieldCentric m_drive =
      new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); //
  // field-centric

  // private final SwerveRequest.RobotCentric m_drive =
  // new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  /*
   * @param trajectory The trajectory to follow.
   * 
   * @param controller The HolonomicDriveController for the drivetrain.
   * 
   * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at each
   * time step.
   * 
   * @param driveSubsystem The subsystem to use to drive the robot.
   * 
   * @param requirements The subsystems to require.
   */

  public TrajectoryFollower(Trajectory trajectory, Rotation2d endTheta,
      CommandSwerveDrivetrain driveSubsystem) {
    m_trajectory = trajectory;
    m_controller = new HolonomicDriveController(
        new PIDController(Constants.Drivetrain.kLateralPositionP, 0,
            Constants.Drivetrain.kLateralPositionD),
        new PIDController(Constants.Drivetrain.kLateralPositionP, 0,
            Constants.Drivetrain.kLateralPositionD),
        new ProfiledPIDController(Drivetrain.kAngularPositionP, 0.0, Drivetrain.kAngularPositionD,
            new TrapezoidProfile.Constraints(Drivetrain.kMaxAngularSpeed,
                Drivetrain.kMaxAngularAcceleration)));
    m_drivetrain = driveSubsystem;
    m_desiredRotation = endTheta;
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    if (this.m_trajectory == null) {
      return;
    }
    double curTime = m_timer.get();
    var desiredState = m_trajectory.sample(curTime);
    Rotation2d desiredRotation = m_desiredRotation;
    System.out.println(curTime);
    ChassisSpeeds targetChassisSpeeds =
        m_controller.calculate(m_drivetrain.getPose2d(), desiredState, desiredRotation);
    // This is done because the rotation is inverted.
    // It may not be the same on your robot, so if the rotation does not function as
    // expected,
    // remove this.
    // targetChassisSpeeds.omegaRadiansPerSecond *= -1;
    m_drivetrain
        .applyRequest(() -> new SwerveRequest.ApplyChassisSpeeds().withSpeeds(targetChassisSpeeds));
    m_drivetrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(targetChassisSpeeds));
    // m_drivetrain.applyRequest(() ->
    // m_drive.withVelocityX(targetChassisSpeeds.vxMetersPerSecond)
    // .withVelocityY(targetChassisSpeeds.vyMetersPerSecond)
    // .withRotationalRate(targetChassisSpeeds.omegaRadiansPerSecond));
    // m_drivetrain.setControl(m_drive.withVelocityX(-targetChassisSpeeds.vxMetersPerSecond)
    // .withVelocityY(targetChassisSpeeds.vyMetersPerSecond)
    // .withRotationalRate(targetChassisSpeeds.omegaRadiansPerSecond));
  }

  @Override
  public void end(boolean interrupted) {
    if (!interrupted)
      m_drivetrain
          .applyRequest(() -> m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    m_timer.stop();

  }

  @Override
  public boolean isFinished() {
    if (this.m_trajectory != null) {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
    return true;
  }
}
