package frc.robot.autonomous;

import java.util.Optional;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.structures.Vertex;
import me.nabdev.pathfinding.Pathfinder;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.PathfindingDebugUtils;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.Pathfinder.PathfindSnapMode;
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
            .setRobotWidth(Constants.Drivetrain.kBotWidth).setCornerDist(0.3).setCornerCutDist(0.1)
            .build();

    ArrayList<Edge> edges = m_pathfinder.visualizeEdges();
    PathfindingDebugUtils.drawLines("Field Map", edges, m_pathfinder.visualizeVertices());
    PathfindingDebugUtils.drawLines("Field Map Inflated", edges,
        m_pathfinder.visualizeInflatedVertices());
    generate(new Pose2d(new Translation2d(1, 0), new Rotation2d(0, 0)));
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

  public Path generateandget(Pose2d targetPose) {
    final Pose2d pose = RobotContainer.m_drivetrain.getPose2d();
    try {
      return m_pathfinder.generatePath(pose, targetPose);
    } catch (ImpossiblePathException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      return new Path(new Vertex(pose), new Vertex(new Pose2d(1, 0, Rotation2d.fromDegrees(0))),
          m_pathfinder);
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
    generate(m_targetPose);
    return m_path.get().getTarget().asPose2d();
  }

  public PathPlannerPath asPathPlannerPath(Pose2d targetPose2d) {
    List<Translation2d> bezierPoints =
        PathPlannerPath.bezierFromPoses(generateandget(targetPose2d).asPose2dList());

    PathPlannerPath path = new PathPlannerPath(bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The
                                                                 // constraints
                                                                 // for
                                                                 // this
                                                                 // path.
                                                                 // If
                                                                 // using
                                                                 // a
                                                                 // differential
                                                                 // drivetrain,
                                                                 // the
                                                                 // angular
                                                                 // constraints
                                                                 // have
                                                                 // no
                                                                 // effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(gettargetPose2d().getRotation().getDegrees())) // Goal
                                                                                                    // end
                                                                                                    // state.
                                                                                                    // You
                                                                                                    // can
                                                                                                    // set
                                                                                                    // a
    // holonomic rotation here. If using
    // a differential drivetrain, the
    // rotation will have no effect.
    );
    bezierPoints.clear();
    path.preventFlipping = true;
    return path;
  }
}
