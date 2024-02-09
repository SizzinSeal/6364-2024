package frc.robot.autonomous;

import java.util.function.Supplier;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Constants.Drivetrain;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.algorithms.SearchAlgorithm;
import me.nabdev.pathfinding.algorithms.SearchAlgorithm.SearchAlgorithmType;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public interface Trajectories {

  public record outerror() {
    static boolean err = false;
  }

  public class GenerateTrajectory {
    PathfinderBuilder m_pathbuilder =
        new PathfinderBuilder(Field.CRESCENDO_2024).setRobotLength(Constants.Drivetrain.kBotLength)
            .setRobotWidth(Constants.Drivetrain.kBotWidth).setNormalizeCorners(false);
    Pathfinder m_pathfinder = m_pathbuilder.build();


    public Trajectory newTrajectory(Pose2d targPose2d, CommandSwerveDrivetrain m_drivetrain) {
      try {
        outerror.err = false;
        return m_pathfinder.generateTrajectory(m_drivetrain.getPose2d(), targPose2d,
            Constants.Drivetrain.K_TRAJECTORY_CONFIG);
      } catch (ImpossiblePathException e) {
        e.printStackTrace();
        outerror.err = true;
        return new Trajectory();
      }
    }

  }

  public class TrajectoryFollower extends Command {

    private final Timer m_timer = new Timer();
    private final Trajectory m_trajectory;
    private final HolonomicDriveController m_controller;
    private final Rotation2d m_desiredRotation;
    private final CommandSwerveDrivetrain m_drivetrain;


    private final SwerveRequest.FieldCentric m_drive =
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage); // field-centric

    /**
     * Constructs a new FollowTrajectory command that when executed will follow the provided
     * trajectory. This command will not return output voltages but rather raw module states from
     * the position controllers which need to be put into a velocity PID.
     *
     * <p>
     * Note: The controllers will *not* set the outputVolts to zero upon completion of the path-
     * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory The trajectory to follow.
     * @param controller The HolonomicDriveController for the drivetrain.
     * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at
     *        each time step.
     * @param driveSubsystem The subsystem to use to drive the robot.
     * @param requirements The subsystems to require.
     */
    public TrajectoryFollower(Trajectory trajectory, Rotation2d desiredRotation,
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
      m_desiredRotation = desiredRotation;
    }

    @Override
    public void initialize() {
      m_timer.restart();
    }

    @Override
    public void execute() {
      if (outerror.err == true) {
        return;
      }
      double curTime = m_timer.get();
      var desiredState = m_trajectory.sample(curTime);
      Rotation2d desiredRotation = m_desiredRotation;
      ChassisSpeeds targetChassisSpeeds =
          m_controller.calculate(m_drivetrain.getPose2d(), desiredState, desiredRotation);
      // This is done because the rotation is inverted.
      // It may not be the same on your robot, so if the rotation does not function as expected,
      // remove this.
      targetChassisSpeeds.omegaRadiansPerSecond *= -1;
      m_drivetrain.applyRequest(() -> m_drive.withVelocityX(targetChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(targetChassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(targetChassisSpeeds.omegaRadiansPerSecond));
      m_drivetrain.setControl(m_drive.withVelocityX(targetChassisSpeeds.vxMetersPerSecond)
          .withVelocityY(targetChassisSpeeds.vyMetersPerSecond)
          .withRotationalRate(targetChassisSpeeds.omegaRadiansPerSecond));
    }

    @Override
    public void end(boolean interrupted) {
      m_timer.stop();
    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
  }

}


