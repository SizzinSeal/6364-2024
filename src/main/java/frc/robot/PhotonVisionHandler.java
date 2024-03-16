package frc.robot;

import java.util.HashMap;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonVisionHandler {

  private static double inchToMeter(double inchValue) {
    double meter = inchValue / 39.37;
    return meter;
  }

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private static PhotonCamera vision = new PhotonCamera("photonvision");
  private static PhotonTrackedTarget target = vision.getLatestResult().getBestTarget();
  private final static double latencyMilis = vision.getLatestResult().getLatencyMillis() / 1000.0;
  private final static double kCameraHeight = inchToMeter(13.8);
  private final static Pose3d camToTarget = aprilTagFieldLayout.getTagPose(target.getFiducialId());

  // camera dimensions from center of robot (x, y, z) (10.4x, -6.5y, 13.8z) inches
  // front is intake (y), pos x to the right of that, z right
  private final static Transform3d robotToCam = new Transform3d(
      new Translation3d(inchToMeter(10.4), inchToMeter(-6.5), inchToMeter(13.8)),
      new Rotation3d(0, 0, 0));

  private final static HashMap<Integer, Double> kAprilTagHeights;
  static {
    kAprilTagHeights = new HashMap<>();
    kAprilTagHeights.put(1, inchToMeter(53.38));
    kAprilTagHeights.put(2, inchToMeter(53.38));
    kAprilTagHeights.put(3, inchToMeter(57.13));
    kAprilTagHeights.put(4, inchToMeter(57.13));
    kAprilTagHeights.put(5, 0.0);
    kAprilTagHeights.put(6, 0.0);
    kAprilTagHeights.put(7, 0.0);
    kAprilTagHeights.put(8, 0.0);
    kAprilTagHeights.put(9, 0.0);
    kAprilTagHeights.put(10, 0.0);
    kAprilTagHeights.put(11, 0.0);
    kAprilTagHeights.put(12, 0.0);
    kAprilTagHeights.put(13, 0.0);
    kAprilTagHeights.put(14, 0.0);
    kAprilTagHeights.put(15, 0.0);
    kAprilTagHeights.put(16, 0.0);

  }

  /**
   * Get the number of April Tags in total
   */
  public static int getNumberofTags() {
    return vision.getLatestResult().getTargets().size();
  }

  public static int getAprilTagID() {
    return target.getFiducialId(); // gets apriltag being identified
  }

  public static double areaOfAprilTag() {
    return target.getArea(); // gets area of apriltag
  }

  public static double latency() {
    return latencyMilis; // gets area of apriltag
  }

  public final static Transform3d getPos3D() {
    public Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);

  }

}
