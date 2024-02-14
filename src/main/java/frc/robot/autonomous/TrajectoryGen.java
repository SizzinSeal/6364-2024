package frc.robot.autonomous;

import java.util.Optional;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import me.nabdev.pathfinding.structures.Path;
import me.nabdev.pathfinding.Pathfinder;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.util.PathfindingDebugUtils;
import me.nabdev.pathfinding.PathfinderBuilder;
import me.nabdev.pathfinding.structures.Edge;
import me.nabdev.pathfinding.structures.ImpossiblePathException;
import me.nabdev.pathfinding.utilities.FieldLoader.Field;

public class TrajectoryGen {
  // m_pathfinder is used to generate a path that avoids obstacles
  private final Pathfinder m_pathfinder = new PathfinderBuilder(Field.CRESCENDO_2024)
      .setInjectPoints(true).setPointSpacing(0.5).setCornerPointSpacing(0.05)
      .setRobotLength(Constants.Drivetrain.kBotLength).setRobotWidth(Constants.Drivetrain.kBotWidth)
      .setCornerDist(0.3).setCornerCutDist(0.1).build();
  // m_pathfinderPath is the path generated by the pathfinder. It's converted to a PathPlannerPath
  // later
  private Optional<Path> m_pathfinderPath = Optional.empty();
  // m_pathPlannerPath is what's fed into the path follower
  private Optional<PathPlannerPath> m_pathPlannerPath = Optional.empty();
  // m_targetPose is the target pose
  private Pose2d m_targetPose;

  /**
   * @brief TrajectoryGenerator constructor
   * 
   *        This class is used to generate trajectories
   */
  public TrajectoryGen() {
    // set up visualizations
    final ArrayList<Edge> edges = m_pathfinder.visualizeEdges();
    PathfindingDebugUtils.drawLines("Field Map", edges, m_pathfinder.visualizeVertices());
    PathfindingDebugUtils.drawLines("Field Map Inflated", edges,
        m_pathfinder.visualizeInflatedVertices());
    generate(new Pose2d(new Translation2d(1, 0), new Rotation2d(0, 0)));
  }

  /**
   * @brief generate the pathfinder path
   * 
   * @param targetPose
   */
  public void generate(final Pose2d targetPose) {
    final Pose2d pose = RobotContainer.m_drivetrain.getPose();
    // don't regenerate the path if the target pose is the same as the last one
    if (m_targetPose == targetPose)
      return;
    // update the target pose
    m_targetPose = targetPose;
    try {
      m_pathfinderPath = Optional.of(m_pathfinder.generatePath(pose, targetPose));
    } catch (final ImpossiblePathException e) { // throw an error if the path is impossible
      e.printStackTrace();
    }
  }

  public Pose2d getTargetPose() {
    generate(m_targetPose);
    return m_pathfinderPath.get().getTarget().asPose2d();
  }

  public List<Pose2d> generatePosesFromBezierPoints(final List<Translation2d> bezierPoints) {
    final List<Pose2d> poses = new ArrayList<>();
    // Assuming you want to set a default orientation (facing in the x-direction)
    final Rotation2d defaultOrientation = new Rotation2d();

    // Convert each Translation2d point to a Pose2d with default orientation
    for (final Translation2d point : bezierPoints) {
      poses.add(new Pose2d(point, defaultOrientation));
    }

    return poses;
  }

  public void generatePathPlannerPath(final Pose2d targetPose2d) {
    generate(targetPose2d);
    final List<Translation2d> bezierPoints =
        PathPlannerPath.bezierFromPoses(m_pathfinderPath.get().asPose2dList());
    m_pathPlannerPath = Optional.of(new PathPlannerPath(bezierPoints,
        new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // constraints
        new GoalEndState(0.0, Rotation2d.fromDegrees(getTargetPose().getRotation().getDegrees())) // goal
    ));
    m_pathPlannerPath.get().preventFlipping = true;
    m_pathPlannerPath.get().replan(RobotContainer.m_drivetrain.getPose(),
        RobotContainer.m_drivetrain.getChassisSpeeds());
  }

  public PathPlannerPath getPath(final Pose2d targetPose) {
    if (m_pathPlannerPath.isEmpty())
      generatePathPlannerPath(new Pose2d(new Translation2d(0, 1), Rotation2d.fromDegrees(0)));
    else
      generatePathPlannerPath(targetPose);
    return m_pathPlannerPath.get();
  }
}
