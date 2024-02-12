package frc.robot.autonomous;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.Pathfinder;
import java.util.ArrayList;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.PathfindingDebugUtils;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.Edge;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;


public class TrajectoryGen {
  private final Pathfinder m_pathfinder;
  private Optional<Path> m_path = Optional.empty();
  private Pose2d m_targetPose;

  /**
   * @brief TrajectoryGenerator constructor
   * 
   *        This class is used to generate trajectories
   */
  public TrajectoryGen() {
    m_pathfinder =
        new PathfinderBuilder(Field.CRESCENDO_2024).setInjectPoints(true).setPointSpacing(0.5)
            .setCornerPointSpacing(0.05).setRobotLength(Constants.Drivetrain.kBotLength)
            .setRobotWidth(Constants.Drivetrain.kBotWidth).setCornerDist(0.3).build();

    ArrayList<Edge> edges = m_pathfinder.visualizeEdges();
    PathfindingDebugUtils.drawLines("Field Map", edges, m_pathfinder.visualizeVertices());
    PathfindingDebugUtils.drawLines("Field Map Inflated", edges,
        m_pathfinder.visualizeInflatedVertices());
    generate(new Pose2d(new Translation2d(1, 0), new Rotation2d(1, 0)));
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
    if (m_targetPose != targetPose) {
      m_targetPose = targetPose;
      try {
        m_path = Optional.of(m_pathfinder.generatePath(pose, targetPose));
        System.out.println("trajnew");

      } catch (ImpossiblePathException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
        // System.out.println("trajold" + m_path);
      }
    }
  }

  /**
   * @brief get the generated trajectory
   *
   *        This method should not be called before the generate method has been called, otherwise a
   *        runtime exception will be thrown
   * 
   * @return Trajectory
   */
  public Trajectory getTrajectory() {
    // if (m_path.isEmpty())
    // throw new RuntimeException("Tried to follow a path that has not been generated!");
    try {
      return m_path.get().asTrajectory(Constants.Drivetrain.K_TRAJECTORY_CONFIG);
    } catch (ImpossiblePathException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      return new Trajectory();
    }
  }

  public Pose2d gettargetPose2d() {
    return m_path.get().getTarget().asPose2d();
  }

}
