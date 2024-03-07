package frc.robot.util;

/**
 * @brief Custom data type with AprilTag information.
 */
public class AprilTagInfo {
  public Integer primaryTagId;
  public Integer numTagInView;
  public Double tagArea;

  public AprilTagInfo(Integer id, Integer count, Double area) {
    primaryTagId = id;
    numTagInView = count;
    tagArea = area;
  };
}
