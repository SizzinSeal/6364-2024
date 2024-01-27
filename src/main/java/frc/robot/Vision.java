package frc.robot;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/**
 * @brief LimeLight wrapper
 *
 *        We interact with the limelight through networktables. It posts data, and we need to read
 *        that data from networktables.
 * 
 *        Using networktables all the time inflates code size, so we have this wrapper to simplify
 *        using limelights
 */
public class Vision {
  private DoubleArraySubscriber m_poseSubscriber; // limelight pose subscriber
  private final NetworkTable m_table; //
  private Optional<Alliance> m_allianceColour;

  /**
   * @brief Vision class constructor
   * 
   * @param limelightName name of the limelight
   */
  public Vision(String limelightName) {
    m_table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  /**
   * TODO: explain why this logic is not just in the constructor
   */
  public void init() {
    // robot position is different if its on the Blue alliance or the Red alliance
    m_allianceColour = DriverStation.getAlliance();
    if (m_allianceColour.get() == Alliance.Blue) {
      m_poseSubscriber = m_table.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[7]);
    } else {
      m_poseSubscriber = m_table.getDoubleArrayTopic("botpose_wpired").subscribe(new double[7]);
    }
  }

  /**
   * TODO: add documentation
   */
  public double detectMultitag() {
    double[] Cornercount = m_table.getEntry("tcornxy").getDoubleArray(new double[32]);

    double TagCount = ((Cornercount.length) / 8);

    if (TagCount < 1 && TagCount != 0) {
      TagCount = 1;
    }

    // System.out.println(TagCount);

    return TagCount;
  }

  /**
   * TODO: add documentation
   */
  public double tagSize() {
    return m_table.getEntry("ta").getDouble(0.0);
  }

  /**
   * TODO: add documentation
   */
  public double[] tagDetector() {
    double tagID = m_table.getEntry("tid").getInteger(-1);

    double[] internal = new double[3];

    internal[0] = tagID; // Tag ID

    internal[1] = detectMultitag(); // Number of Tags in view

    internal[2] = tagSize(); // Size of tags in view

    return internal;
  }

  /**
   * TODO: add documentation
   */
  public Pose2d getPos2D() {
    double[] DASubTpos = m_poseSubscriber.get();
    return new Pose2d(new Translation2d(DASubTpos[0], DASubTpos[1]),
        new Rotation2d(Units.degreesToRadians(DASubTpos[5])));
  }

  /**
   * @brief get the distance from the robot to the tag
   * 
   * @return distance in meters
   */

  public double getDist3D() {
    // get the measured pose in the target coordinate system
    double[] measuredPoseArray =
        m_table.getEntry("targetpose_robotspace").getDoubleArray(new double[6]);

    // create the vector
    Translation3d measuredPose =
        new Translation3d(measuredPoseArray[0], measuredPoseArray[1], measuredPoseArray[2]);
    // return the magnitude of the vector
    return measuredPose.getNorm();
  }

  /**
   * TODO: add documenation
   */
  public double getLatestTimestamp() {
    return m_poseSubscriber.getAtomic().timestamp;
  }

  /**
   * TODO: add documentation
   */
  public TimestampedDoubleArray getPoseRaw() {
    return m_poseSubscriber.getAtomic();
  }

  /**
   * TODO: add documentation
   */
  public void telemetry() {

    // TimestampedDoubleArray internal1 = DASub.getAtomic();

  }

  /**
   * TODO: add documentation
   */
  public double getLatestLatencyAdjustedTimeStamp() {
    TimestampedDoubleArray internal2 = m_poseSubscriber.getAtomic();
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }

  // TODO: move this comment somewhere that makes more sense
  // getLatestLatencyAdjustedTimeStamp() is in seconds
  // .timestamp is in millis and .value[6] is in millis

}
