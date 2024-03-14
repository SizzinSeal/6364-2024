package frc.robot;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PhotonVisionHandler {
  private static PhotonCamera vision = new PhotonCamera("photonvision");
  private static PhotonTrackedTarget target = vision.getLatestResult().getBestTarget();
  private final static double kCameraHeight = 0.35052; // meters

  // in order of ID 1 to ID 16
  private static double[] AprilTagHeights = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

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

  // camera dimensions from center of robot (x, y, z) (10.4x, -6.5y, 13.8z) inches
  // front is intake (y), pos x to the right of that, z right
  public static Pose2d getPos2D() {
    return PhotonUtils.estimateFieldToRobot(kCameraHeight, kTargetHeight, kCameraPitch,
        kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose,
        cameraToRobot);
  }

}
