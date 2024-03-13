package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.units.*;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.AprilTagInfo;

/**
 * @brief "Handles" incoming limelight data from Networktables.
 */
public class LimelightHandler {

  private static final NetworkTable kTable = NetworkTableInstance.getDefault().getTable("limelight");;
  private static DoubleArraySubscriber m_poseSubscriber;
  private static final double limelightMountAngleDegrees = -0.0; // SHOULD BE NEGATIVE
  private static final double limelightLensHeightInches = 20.0;

  /**
   * @brief Whether the limelight has any valid targets.
   */
  public static Boolean hasValidTarget() {
    if ((int) kTable.getEntry("tv").getDouble(0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * @return Active pipeline index of the camera (0 .. 9).
   */
  public static Integer getActivePipeline() {
    return (int) kTable.getEntry("getpipe").getDouble(0);
  }

  /**
   * @brief Class ID of primary neural detector result. Only applies to "Detector"
   *        pipeline.
   * 
   * @return Class ID.
   */
  public static Integer getPrimaryDetectorClassID() {
    return (int) kTable.getEntry("tclass").getDouble(0);
  }

  /**
   * @brief Horizontal offset of target from origin.
   * 
   * @return Offset in degrees (WPI Units Library).
   */
  public static Measure<Angle> getHorizontalOffset() {
    double value = kTable.getEntry("tx").getDouble(0);
    return Degrees.of(value);
  }

  /**
   * @brief Vertical offset of target from origin.
   * 
   * @return Offset in degrees (WPI Units Library).
   */
  public static Measure<Angle> getVerticalOffset() {
    double value = kTable.getEntry("ty").getDouble(0);
    return Degrees.of(value);
  }

  /**
   * @brief Target Area (0% of image to 100% of image).
   * 
   * @return Double target area of image (0.0 .. 1.0).
   */
  public static Double getTargetArea() {
    return kTable.getEntry("ta").getDouble(0);
  }

  /**
   * @brief Subscribe! Check for connection then subscribe to NetworkTable topic.
   *        Prints err message
   *        to console otherwise.
   */
  public static void SubscribeToRobotPose() {
    if (isConnected()) {
      m_poseSubscriber = kTable.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[7]);
    } else {
      System.err.print("Limelight not connected!");
    }
  }

  /**
   * @brief Detect tag.
   * 
   * @return AprilTagInfo about the detected tag.
   */
  public static AprilTagInfo detectTag() {
    Integer id = (int) kTable.getEntry("tid").getInteger(-1);
    double[] tempCornerCoordArray = kTable.getEntry("tcornxy").getDoubleArray(new double[32]);
    Integer count = tempCornerCoordArray.length / 8;
    Double area = getTargetArea();

    return new AprilTagInfo(id, count, area);
  }

  /**
   * @brief Get the latest latency adjusted timestamp in seconds.
   * 
   * @return Double latest latency adjusted timestamp in seconds.
   */
  public static Double getLatestLatencyAdjustedTimestamp() {
    TimestampedDoubleArray internal2 = m_poseSubscriber.getAtomic();
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }

  /**
   * @brief Get the 2D position measured by the limelight.
   * 
   * @return Pose2d 2D position measured by the limelight
   */
  public static Pose2d getLimelightFieldPose2DEstimate() {
    try {
      double[] raw = m_poseSubscriber.get();
      return new Pose2d(new Translation2d(raw[0], raw[1]),
          new Rotation2d(Units.degreesToRadians(raw[5])));
    } catch (NullPointerException e) {
      System.err.println("Something went wrong when trying to getLimelightFieldPose2DEstimate.");
      System.out.println(e);
      return null;
    }
  }

  public static Pose2d getNoteFieldPose2DEstimate(CommandSwerveDrivetrain m_drivetrain) {
    Transform2d transformation = new Transform2d(
        new Translation2d(getDistanceToNote(), new Rotation2d(getHorizontalOffset())),
        new Rotation2d(getHorizontalOffset()));
    return m_drivetrain.getPose().transformBy(transformation);
  }

  private static double getDistanceToNote() {
    if (hasValidTarget()) {
      double limelightMountAngleRadians = Radians.convertFrom(limelightMountAngleDegrees, Degrees);
      double angleToNoteRadians = limelightMountAngleRadians + getVerticalOffset().in(Radians);
      double value = (-limelightLensHeightInches) / Math.tan(angleToNoteRadians);
      return value;
    }
    return 0.0;
  }

  /**
   * @brief Check connection.
   */
  private static Boolean isConnected() {
    return kTable.containsSubTable("botpose_wpiblue");
  }
}
