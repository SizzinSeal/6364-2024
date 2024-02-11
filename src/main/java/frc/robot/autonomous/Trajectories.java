package frc.robot.autonomous;

import java.util.ArrayList;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import java.util.Optional;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.PathfindingDebugUtils;
import me.nabdev.pathfinding.Pathfinder;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.Pathfinder.PathfindSnapMode;
import me.nabdev.pathfinding.structures.Edge;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public interface Trajectories {

  public record outerror() {
    static boolean err = false;
  }

  public class TrajectoryGenerator {
    private final Pathfinder m_pathfinder;
    private Optional<Path> m_path = Optional.empty();

    /**
     * @brief TrajectoryGenerator constructor
     * 
     *        This class is used to generate trajectories
     */
    public TrajectoryGenerator() {

      Field2d field = new Field2d();

      m_pathfinder =
          new PathfinderBuilder(Field.CRESCENDO_2024).setInjectPoints(true).setPointSpacing(0.5)
              .setCornerPointSpacing(0.05).setRobotLength(Constants.Drivetrain.kBotLength)
              .setRobotWidth(Constants.Drivetrain.kBotWidth).setCornerDist(0.3).build();

      ArrayList<Edge> edges = m_pathfinder.visualizeEdges();
      PathfindingDebugUtils.drawLines("Field Map", edges, m_pathfinder.visualizeVertices());
      PathfindingDebugUtils.drawLines("Field Map Inflated", edges,
          m_pathfinder.visualizeInflatedVertices());



    }

    /**
     * @brief generate the trajectory
     * 
     *        This method needs to be called before calling the getTrajectory method
     * 
     * @param targetPose
     */
    public void generate(Pose2d targetPose) {
      final Pose2d pose = RobotContainer.m_drivetrain.getPose2d();
      try {
        m_path = Optional.of(m_pathfinder.generatePath(pose, targetPose));
        System.out.println("trajnew" + m_path);

      } catch (ImpossiblePathException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
        System.out.println("trajold" + m_path);
      }
      telemetry();
    }

    private void telemetry() {}

    /**
     * @brief get the generated trajectory
     *
     *        This method should not be called before the generate method has been called, otherwise
     *        a runtime exception will be thrown
     * 
     * @return Trajectory
     */
    public Trajectory getTrajectory() {
      if (m_path.isEmpty())
        throw new RuntimeException("Tried to follow a path that has not been generated!");
      return m_path.get().asTrajectory(Constants.Drivetrain.K_TRAJECTORY_CONFIG);
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

    /*
     * @param trajectory The trajectory to follow.
     * 
     * @param controller The HolonomicDriveController for the drivetrain.
     * 
     * @param desiredRotation The angle that the drivetrain should be facing. This is sampled at
     * each time step.
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
      double curTime = m_timer.get();
      var desiredState = m_trajectory.sample(curTime);
      Rotation2d desiredRotation = m_desiredRotation;
      ChassisSpeeds targetChassisSpeeds =
          m_controller.calculate(m_drivetrain.getPose2d(), desiredState, desiredRotation);
      // This is done because the rotation is inverted.
      // It may not be the same on your robot, so if the rotation does not function as
      // expected,
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
      if (!interrupted)
        m_drivetrain
            .applyRequest(() -> m_drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
      m_timer.stop();

    }

    @Override
    public boolean isFinished() {
      return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
    }
  }

}
