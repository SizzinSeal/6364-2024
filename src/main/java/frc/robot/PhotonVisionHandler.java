package frc.robot;

import java.util.HashMap;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonVisionHandler {

  public PhotonVisionHandler() {

  }

  private double inchToMeter(double inchValue) {
    double meter = inchValue / 39.37;
    return meter;
  }

  AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

  private PhotonCamera vision = new PhotonCamera("photonvision");
  private PhotonTrackedTarget target = vision.getLatestResult().getBestTarget();
  private final double latencyMilis = vision.getLatestResult().getLatencyMillis() / 1000.0;
  private final double kCameraHeight = inchToMeter(13.8);
  private DoubleArraySubscriber m_poseSubscriber;

  // camera dimensions from center of robot (x, y, z) (10.4x, -6.5y, 13.8z) inches
  // front is intake (y), pos x to the right of that, z right
  private final Transform3d robotToCam = new Transform3d(
      new Translation3d(inchToMeter(10.4), inchToMeter(-6.5), inchToMeter(13.8)),
      new Rotation3d(0, 0, 0));// ***REMINDER GET ROTATION VALUES OF CAMERA MOUNT***

  /**
   * @breif Get the number of April Tags in total
   */
  public int getNumberofTags() {
    return vision.getLatestResult().getTargets().size();
  }

  public int getAprilTagID() {
    return target.getFiducialId(); // gets apriltag being identified
  }

  public double areaOfAprilTag() {
    return target.getArea(); // gets area of apriltag
  }

  public double latency() {
    return latencyMilis;
  }

  /**
   * @breif gets the position in 3d and then converts it to 2d field relative pose
   * 
   * @return pos2d
   */
  public Pose2d k3Dto2D() { // gets the position in 3d and then converts it to 2d field relative pose
    Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), robotToCam);

    double Xcoord = robotPose.getTranslation().getX();
    double Ycoord = robotPose.getTranslation().getY();

    Pose2d pos2d = new Pose2d(Xcoord, Ycoord, robotPose.getRotation().toRotation2d());
    return pos2d;
  }

  public double getLatestLatencyAdjustedTimeStamp() {
    final TimestampedDoubleArray internal2 = m_poseSubscriber.getAtomic();
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }
}
