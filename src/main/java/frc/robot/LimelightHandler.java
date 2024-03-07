package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.*;

public class LimelightHandler {
  public class AprilTagInfo {
    public Integer id;
    public Integer count;
    public Integer area;

    public AprilTagInfo(Integer id, Integer count, Integer area) {
      this.id = id;
      this.count = count;
      this.area = area;
    };
  }

  private NetworkTable m_table;
  private DoubleArraySubscriber m_poseSubscriber;

  public LimelightHandler(String limelightName) {
    m_table = NetworkTableInstance.getDefault().getTable(limelightName);
  }

  public Boolean hasValidTarget() {
    if ((int) m_table.getEntry("tv").getDouble(0) == 1) {
      return true;
    } else {
      return false;
    }
  }

  public Integer getActivePipeline() {
    return (int) m_table.getEntry("getpipe").getDouble(0);
  }

  public Integer getPrimaryDetectorClassID() {
    return (int) m_table.getEntry("tclass").getDouble(0);
  }

  public Measure<Angle> getHorizontalOffset() {
    double value = m_table.getEntry("tx").getDouble(0);
    return Degrees.of(value);
  }

  public Measure<Angle> getVerticalOffset() {
    double value = m_table.getEntry("ty").getDouble(0);
    return Degrees.of(value);
  }

  public Double getTargetArea() {
    return m_table.getEntry("ta").getDouble(0);
  }

  public void SubscribeToRobotPose() {}

  public AprilTagInfo detectTag() {
    Integer id = (int) m_table.getEntry("tid").getInteger(-1);
    Integer count = 0;
    out.area = 0;

    return out;
  }
}
