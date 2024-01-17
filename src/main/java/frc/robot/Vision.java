package frc.robot;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import java.util.EnumSet;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Vision {

  private DoubleArraySubscriber DASub;
  private final int taglistener;

  public boolean isTagDetected = false;

  private boolean TagDetector(Long id) {
    if (id > -1) {
      return true;
    } else {
      return false;
    }
  }

  public Vision(String topicname) {
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    DASub = inst.getDoubleArrayTopic(topicname).subscribe(new double[7]);

    taglistener = inst.addListener(new String[] {topicname},
        EnumSet.of(NetworkTableEvent.Kind.kTopic), event -> {
          if (event.is(NetworkTableEvent.Kind.kPublish)) {
            isTagDetected = TagDetector(inst.getEntry("<tid>").getInteger(-1));
          }
        });
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
    return DASub.getAtomic();

  }

  public void Telemetry() {

    // TimestampedDoubleArray internal1 = DASub.getAtomic();

  }

  public double getLatestLatencyAdjustedTimeStamp() {

    TimestampedDoubleArray internal2 = DASub.getAtomic();
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }

  // getLatestLatencyAdjustedTimeStamp() is in seconds
  // .timestamp is in millis and .value[6] is in millis

}
