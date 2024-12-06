package frc.robot;

import java.util.Optional;
import java.util.OptionalDouble;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

public class PhotonVisionHandler {

  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;
  private PhotonCamera vision;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private boolean simulated;
  private PhotonPoseEstimator photonPoseEstimator;

  private final Transform3d robotToCam =
      new Transform3d(new Translation3d(Units.inchesToMeters(10.4), Units.inchesToMeters(-6.5),
          Units.inchesToMeters(13.8)), new Rotation3d(0, Math.toRadians(-15), 0)); // Adjusted
                                                                                   // camera angle

  private DoubleArraySubscriber m_poseSubscriber;

  public PhotonVisionHandler() {
    vision = new PhotonCamera("photonvision");
    simulated = Utils.isSimulation();


    // Load AprilTag field layout
    try {
      aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    } catch (Exception e) {
      System.err.println("Error loading AprilTag field layout: " + e.getMessage());
      aprilTagFieldLayout = null;
    }

    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_RIO, vision, robotToCam);


    if (simulated) {
      initializeSimulation();
    }
  }

  private void initializeSimulation() {
    if (aprilTagFieldLayout == null) {
      System.out.println("AprilTag field layout is null. Simulation will not include tags.");
      return;
    }

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(aprilTagFieldLayout);

    var cameraProps = new SimCameraProperties();
    cameraProps.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
    cameraProps.setCalibError(0.35, 0.10);
    cameraProps.setFPS(120);
    cameraProps.setAvgLatencyMs(50);
    cameraProps.setLatencyStdDevMs(15);
    // cameraProps.setFOV(75); // Set appropriate FOV for your camera
    cameraProps.setAvgLatencyMs(11.0); // Based on actual PhotonVision latency
    cameraProps.setLatencyStdDevMs(3.0);

    cameraSim = new PhotonCameraSim(vision, cameraProps);
    visionSim.addCamera(cameraSim, robotToCam);

    cameraSim.enableDrawWireframe(true);
    System.out.println("PhotonVision simulation initialized");
  }

  public void updateSimulation(Pose2d robotPose) {
    if (simulated && visionSim != null) {
      Pose3d pose3d = new Pose3d(robotPose.getX(), robotPose.getY(), 0.0,
          new Rotation3d(0.0, 0.0, robotPose.getRotation().getRadians()));
      visionSim.update(robotPose);
    }
  }

  // public double getEstimatedUncertainty() {
  // var result = vision.getLatestResult();

  // var estStdDevs = result.getEstimationStdDevs();

  // return
  // }

  public double getLatencySeconds() {
    var result = vision.getLatestResult();
    return result.getLatencyMillis() / 1000.0;
  }

  public int getNumberOfTags() {
    var result = vision.getLatestResult();
    return result.hasTargets() ? result.getTargets().size() : 0;
  }

  public int getAprilTagID() {
    var result = vision.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return (target != null) ? target.getFiducialId() : -1;
  }

  public double areaOfAprilTag() {
    var result = vision.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    return (target != null) ? target.getArea() : 0.0;
  }


  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    boolean run = false;

    if (run == false) {
      run = true;
      return photonPoseEstimator.update();
    }


    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return photonPoseEstimator.update();
  }
  // pose3d.toPose2d();
  // public Pose2d P3toP2(Pose3d pose3d) {
  //
  // double x = pose3d.getX();
  // double y = pose3d.getY();
  // Rotation2d thetarot = pose3d.getRotation().toRotation2d();
  // return new Pose2d(x, y, thetarot);
  // }



  // public Pose2d estimateRobotPose() {
  // var result = vision.getLatestResult();
  // if (!result.hasTargets() || aprilTagFieldLayout == null) {
  // return null;
  // }

  // if (result.getMultiTagResult().estimatedPose.isPresent) {
  // Transform3d fieldToCamera = result.getMultiTagResult().estimatedPose.best;
  // }


  // PhotonTrackedTarget target = result.getBestTarget();
  // var tagPoseOptional = aprilTagFieldLayout.getTagPose(target.getFiducialId());

  // if (tagPoseOptional.isEmpty()) {
  // return null;
  // }

  // Pose3d cameraPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
  // tagPoseOptional.get(), robotToCam);



  // return cameraPose.toPose2d();
  // }

  public OptionalDouble getLatestLatencyAdjustedTimeStamp() {
    if (this.m_poseSubscriber == null) {
      return OptionalDouble.empty();
    } else {
      final TimestampedDoubleArray internal2 = m_poseSubscriber.getAtomic();
      return OptionalDouble.of((internal2.timestamp - internal2.value[6]) * 0.001);

    }
  }
}
