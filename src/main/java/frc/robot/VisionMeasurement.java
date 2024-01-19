package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * @brief VisionMeasurement class
 * 
 *        April Tags have 2 properties: An id, and a position The position is actually a little more
 *        complex, since it can be offset to get the position of the robot, the camera, or the
 *        target and it can be in different coordinate spaces (robot, camera, target, or field)
 */
public class VisionMeasurement {
  private final Pose3d m_botPose;
  private final Pose3d m_botPoseBlue;
  private final Pose3d m_botPoseRed;
  private final Pose3d m_cameraPoseTargetSpace;
  private final Pose3d m_targetPoseCameraSpace;
  private final Pose3d m_targetPoseRobotSpace;
  private final Pose3d m_botPoseTargetSpace;
  private final Pose3d m_cameraPoseRobotSpace;
  private final Integer m_id;

  /**
   * @brief VisionMeasuremet constructor
   * 
   * @param botPose The position of the robot in the field coordinate space
   * @param botPoseBlue The position of the robot in the field coordinate space
   * @param botPoseRed The position of the robot in the field coordinate space
   * @param cameraPoseTargetSpace The position of the camera in the target coordinate space
   * @param targetPoseCameraSpace The position of the target in the camera coordinate space
   * @param targetPoseRobotSpace The position of the target in the robot coordinate space
   * @param botPoseTargetSpace The position of the robot in the target coordinate space
   * @param cameraPoseRobotSpace The position of the camera in the robot coordinate space
   * @param id The id of the AprilTag
   */
  public VisionMeasurement(Pose3d botPose, Pose3d botPoseBlue, Pose3d botPoseRed,
      Pose3d cameraPoseTargetSpace, Pose3d targetPoseCameraSpace, Pose3d targetPoseRobotSpace,
      Pose3d botPoseTargetSpace, Pose3d cameraPoseRobotSpace, Integer id) {
    m_botPose = botPose;
    m_botPoseBlue = botPoseBlue;
    m_botPoseRed = botPoseRed;
    m_cameraPoseTargetSpace = cameraPoseTargetSpace;
    m_targetPoseCameraSpace = targetPoseCameraSpace;
    m_targetPoseRobotSpace = targetPoseRobotSpace;
    m_botPoseTargetSpace = botPoseTargetSpace;
    m_cameraPoseRobotSpace = cameraPoseRobotSpace;
    m_id = id;
  }

  /**
   * @brief Get the Pose of the robot in the field coordinate space
   * 
   * @return Pose3d The position of the robot in the field coordinate space
   */
  public Pose3d getBotPose() {
    return m_botPose;
  }

  /**
   * @brief Get the Pose of the robot in the field coordinate space on the Blue alliance
   * 
   * @return Pose3d The position of the robot in the field coordinate space on the Blue alliance
   */
  public Pose3d getBotPoseBlue() {
    return m_botPoseBlue;
  }

  /**
   * @brief Get the Pose of the robot in the field coordinate space on the Red alliance
   * 
   * @return Pose3d The position of the robot in the field coordinate space on the Red alliance
   */
  public Pose3d getBotPoseRed() {
    return m_botPoseRed;
  }

  /**
   * @brief Get the Pose of the camera in the target coordinate space
   * 
   * @return Pose3d The position of the camera in the target coordinate space
   */
  public Pose3d getCameraPoseTargetSpace() {
    return m_cameraPoseTargetSpace;
  }

  /**
   * @brief Get the Pose of the target in the camera coordinate space
   * 
   * @return Pose3d The position of the target in the camera coordinate space
   */
  public Pose3d getTargetPoseCameraSpace() {
    return m_targetPoseCameraSpace;
  }

  /**
   * @brief Get the Pose of the target in the robot coordinate space
   * 
   * @return Pose3d The position of the target in the robot coordinate space
   */
  public Pose3d getTargetPoseRobotSpace() {
    return m_targetPoseRobotSpace;
  }

  /**
   * @brief Get the Pose of the robot in the target coordinate space
   * 
   * @return Pose3d The position of the robot in the target coordinate space
   */
  public Pose3d getBotPoseTargetSpace() {
    return m_botPoseTargetSpace;
  }

  /**
   * @brief Get the Pose of the camera in the robot coordinate space
   * 
   * @return Pose3d The position of the camera in the robot coordinate space
   */
  public Pose3d getCameraPoseRobotSpace() {
    return m_cameraPoseRobotSpace;
  }

  /**
   * @brief Get the id of the AprilTag
   * 
   * @return Integer The id of the AprilTag
   */
  public Integer getId() {
    return m_id;
  }
}
