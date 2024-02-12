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
import frc.robot.Robot;
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
  private PathPlannerPath path;

  /**
   * @brief TrajectoryGenerator constructor
   * 
   *        This class is used to generate trajectories
   */
  public TrajectoryGen() {
    m_pathfinder = new PathfinderBuilder(Field.CRESCENDO_2024).setInjectPoints(true).setPointSpacing(0.5)
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
    final Pose2d pose = RobotContainer.m_drivetrain.getPose();
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

  public Path getPath(Pose2d targetPose) {
    generate(targetPose);
    return m_path.get();
  }

  public Path generateandget(Pose2d targetPose) {
    final Pose2d pose = RobotContainer.m_drivetrain.getPose();
    try {
      m_path = Optional.of(m_pathfinder.generatePath(pose, targetPose));
      return m_path.get();
    } catch (ImpossiblePathException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
      return m_path.get();
    }
  }

  /**
   * @brief get the generated trajectory
   *
   *        This method should not be called before the generate method has been
   *        called, otherwise a
   *        runtime exception will be thrown
   * 
   * @return Trajectory
   */
  public Trajectory getTrajectory() {
    // if (m_path.isEmpty())
    // throw new RuntimeException("Tried to follow a path that has not been
    // generated!");
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

  public List<Translation2d> generatebezierPoints(Path path) {
    return PathPlannerPath.bezierFromPoses(path.asPose2dList());
  }

  public List<Pose2d> generatePosesFromBezierPoints(List<Translation2d> bezierPoints) {
    List<Pose2d> poses = new ArrayList<>();
    // Assuming you want to set a default orientation (facing in the x-direction)
    Rotation2d defaultOrientation = new Rotation2d();

    // Convert each Translation2d point to a Pose2d with default orientation
    for (Translation2d point : bezierPoints) {
      poses.add(new Pose2d(point, defaultOrientation));
    }

    return poses;
  }

  public PathPlannerPath GetPath(List<Translation2d> points) {
    PathPlannerPath Path = new PathPlannerPath(points,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path.
                                                                 // If using a differential
                                                                 // drivetrain, the angular
                                                                 // constraints have no effect.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a
                                                           // holonomic rotation here. If using a
                                                           // differential drivetrain, the rotation
                                                           // will have no effect.
    );

    Path.preventFlipping = true;
    return Path;
  }

  public void generatePathPlannerPath(Pose2d targetPose2d) {
    generate(targetPose2d);
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(m_path.get().asPose2dList());
    path = new PathPlannerPath(bezierPoints,
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
    System.out.println(path.numPoints());
    path.preventFlipping = true;
    path.replan(RobotContainer.m_drivetrain.getPose(),
        RobotContainer.m_drivetrain.getChassisSpeeds());
  }

  boolean firstrun = true;

  public PathPlannerPath getPathPlannerPath(Pose2d targPose2d) {
    generatePathPlannerPath(targPose2d);
    if (firstrun) {
      generatePathPlannerPath(new Pose2d(new Translation2d(0, 1), Rotation2d.fromDegrees(0)));
      firstrun = false;
    }
    return path;
  }
}
