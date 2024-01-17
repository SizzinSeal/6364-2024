package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

public class Odometry {

  Vision Limelight1 = new Vision("/limelight/<botpose>");
  String CamName = "limelight";
  private SwerveDrivePoseEstimator VOdom = new SwerveDrivePoseEstimator(null,
      null, null, null, null, null);

  public void UpdateVisionOdom() {

    VOdom.addVisionMeasurement(Limelight1.getPos2D(),
        Limelight1.getLatestLatencyAdjustedTimeStamp());

  }

}
