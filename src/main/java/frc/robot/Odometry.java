package frc.robot;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;


public class Odometry extends LimelightHelpers {

    Vision Limelight1 = new Vision("/limelight/<botpose>");
    String CamName = "limelight";
    private SwerveDrivePoseEstimator VOdom = new SwerveDrivePoseEstimator(null, null, null, null, null, null);

    public void UpdateVision() {
        VOdom.addVisionMeasurement(Limelight1.getPos2D() , Limelight1.getLatestTimestamp());
    }


    
}
