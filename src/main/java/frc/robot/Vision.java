package frc.robot;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.EnumSet;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class Vision {

  private DoubleArraySubscriber botpos;
  NetworkTableInstance inst;
  private String limeLightName;
  private Optional<Alliance> alliancecolour;

  public Vision(String LimelightName) {
    limeLightName = LimelightName;
    inst = NetworkTableInstance.getDefault();
  }

  public void init() {
    this.alliancecolour = DriverStation.getAlliance();

    if (this.alliancecolour.get() == Alliance.Blue) {
      botpos =
          inst.getDoubleArrayTopic(limeLightName + "<botpose_wpiblue>").subscribe(new double[7]);

    } else {
      botpos =
          inst.getDoubleArrayTopic(limeLightName + "<botpose_wpired>").subscribe(new double[7]);

    }

  }

  public static double distanceFormula(double x1, double y1, double x2, double y2) {
    return Math.sqrt(Math.pow((x2 - x1), 2) - (Math.pow((y2 - y1), 2)));
  }

  private double detectMultitag() {
    double[] Cornercount =
        inst.getTable(limeLightName).getEntry("<tcornxy>").getDoubleArray(new double[16]);

    double TagCount = ((Cornercount.length) / 4);

    if (TagCount < 1 && TagCount != 0) {
      TagCount = 1;
    }

    return TagCount;
  }

  private double tagSize() {
    return inst.getTable(limeLightName).getEntry("<ta>").getDouble(0.0);
  }

  public double[] tagDetector() {
    long tagID = inst.getTable(limeLightName).getEntry("<tid>").getInteger(-1);

    double[] internal = new double[3];

    internal[0] = tagID; // Tag ID

    internal[1] = detectMultitag(); // Number of Tags in view

    internal[2] = tagSize(); // Size of tags in view

    return internal;
  }

  public Pose2d getPos2D() {
    double[] DASubTpos = botpos.get();
    return new Pose2d(new Translation2d(DASubTpos[0], DASubTpos[1]),
        new Rotation2d(Units.degreesToRadians(DASubTpos[5])));
  }

  public double getTargDist() { /* Get Target Distance from robot in Meters */
    Pose2d currpos = getPos2D();
    double[] currtargpos = inst.getTable(limeLightName).getEntry("<targetpose_robotspace>")
        .getDoubleArray(new double[6]);

    return distanceFormula(currtargpos[0], currtargpos[1], currpos.getX(), currpos.getY());

  }


  public double getLatestTimestamp() {
    return botpos.getAtomic().timestamp;
  }

  public TimestampedDoubleArray getPoseRaw() {
    return botpos.getAtomic();
  }

  public void telemetry() {

    // TimestampedDoubleArray internal1 = DASub.getAtomic();

  }

  public double getLatestLatencyAdjustedTimeStamp() {
    TimestampedDoubleArray internal2 = botpos.getAtomic();
    return ((internal2.timestamp - internal2.value[6]) / 1000.0);
  }

  // getLatestLatencyAdjustedTimeStamp() is in seconds
  // .timestamp is in millis and .value[6] is in millis

}
