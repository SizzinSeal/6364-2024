package frc.robot;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Vision {
  // private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  // private static NetworkTable vistable = inst.getTable("limelight"); // Declare
  // Vision Table at
  // the class level
  // private static NetworkTableEntry internalPosEntry =
  // vistable.getEntry("<botpose>")

  private DoubleArraySubscriber DASub;

  public Vision(String topicname) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DASub = inst.getDoubleArrayTopic(topicname).subscribe(new double[7]);
  }

  public Pose2d getPos2D() {
    double[] DASubTpos = DASub.get();

    return new Pose2d(new Translation2d(DASubTpos[0], DASubTpos[1]),
        new Rotation2d(Units.degreesToRadians(DASubTpos[5])));
  }

  public double getLatestTimestamp() {
    TimestampedDoubleArray DASubT = DASub.getAtomic();
    return DASubT.timestamp;
  }

  public TimestampedDoubleArray getPoseRaw() {
    TimestampedDoubleArray DASubT = DASub.getAtomic();
    return DASubT;
  }

  public void Telemetry() {

    TimestampedDoubleArray internal1 = DASub.getAtomic();
    Pose2d postest = getPos2D();

    // System.out.println("latency" + internal1.value[6]);
    // System.out.println("timestamp" + internal1.timestamp);
    System.out.println(postest.getX());

    getLatestLatencyAdjustedTimeStamp();
    // System.out.println("latency" + getLatestLatencyAdjustedTimeStamp());

  }

  public double getLatestLatencyAdjustedTimeStamp() {

    TimestampedDoubleArray internal2 = DASub.getAtomic();
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }

  // getLatestLatencyAdjustedTimeStamp() is in seconds
  // .timestamp is in millis and .value[6] is in millis

}
