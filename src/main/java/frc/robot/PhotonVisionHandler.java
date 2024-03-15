package frc.robot;

import java.util.HashMap;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PhotonVisionHandler {

  private static double inchToMeter(double inchValue) {
    double meter = inchValue / 39.37;
    return meter;
  }

  private static PhotonCamera vision = new PhotonCamera("photonvision");
  private static PhotonTrackedTarget target = vision.getLatestResult().getBestTarget();
  private final static double latencyMilis = vision.getLatestResult().getLatencyMillis() / 1000.0;
  private final static double kCameraHeight = inchToMeter(13.8);
  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

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

  // camera dimensions from center of robot (x, y, z) (10.4x, -6.5y, 13.8z) inches
  // front is intake (y), pos x to the right of that, z right
  public static Pose2d getPos2D() {
    return PhotonUtils.estimateFieldToRobot(kCameraHeight, kAprilTagHeights, kCameraPitch,
        kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose,
        cameraToRobot);
  }

}
