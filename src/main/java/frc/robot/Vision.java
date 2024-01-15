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
  // private static NetworkTable vistable = inst.getTable("limelight"); // Declare Vision Table at
  // the class level
  // private static NetworkTableEntry internalPosEntry = vistable.getEntry("<botpose>")


  private DoubleArraySubscriber DASub;

  public Vision(String topicname) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DASub = inst.getDoubleArrayTopic(topicname).subscribe(new double[7]);
  }

  public Pose2d getPos2D() {
    double[] DASubTpos = DASub.get(null);

    return new Pose2d(new Translation2d(DASubTpos[0], DASubTpos[1]),
        new Rotation2d(Units.degreesToRadians(DASubTpos[5])));
  }

  // public double getLatestTimestamp() {
  // TimestampedDoubleArray DASubT = DASub.getAtomic(null);
  // return DASubT.timestamp;
  // }

  // public TimestampedDoubleArray getPoseRaw() {
  // TimestampedDoubleArray DASubT = DASub.getAtomic(null);
  // return DASubT;
  // }

  public void Telemetry() {

    TimestampedDoubleArray internal1 = DASub.getAtomic(null);


    System.out.println(
        "X:" + internal1.value[0] + " Timestamp: " + (getLatestLatencyAdjustedTimeStamp()));
    System.out.println(
        "Y:" + internal1.value[1] + " Timestamp: " + (getLatestLatencyAdjustedTimeStamp()));
    System.out.println(
        "Theta:" + internal1.value[5] + " Timestamp: " + (getLatestLatencyAdjustedTimeStamp()));

  }


  public double getLatestLatencyAdjustedTimeStamp() {

    TimestampedDoubleArray internal2 = DASub.getAtomic(null);
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }


  // getLatestLatencyAdjustedTimeStamp() is in seconds
  // .timestamp is in millis and .value[6] is in millis


}


