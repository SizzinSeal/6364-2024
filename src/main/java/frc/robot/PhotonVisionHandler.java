package frc.robot;

import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Rotation2d;

public class PhotonVisionHandler {
  private static PhotonCamera vision = new PhotonCamera("photonvision");
  private static PhotonTrackedTarget target = vision.getLatestResult().getBestTarget();

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

  public static Pose2d getPos2D() {
    Pose2D robotPose = PhotonUtils.estimateFieldToRobot(kCameraHeight, kTargetHeight, kCameraPitch,
        kTargetPitch, Rotation2d.fromDegrees(-target.getYaw()), gyro.getRotation2d(), targetPose,
        cameraToRobot);
  }

}
