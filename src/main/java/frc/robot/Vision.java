package frc.robot;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Vision {

    // Declare class-level variables
    private NetworkTable vistable = null; // Declare Vision Table at the class level

    public double[] getPose() {
        vistable = NetworkTableInstance.getDefault().getTable("limelight");
        return vistable.getEntry("<botpose>").getDoubleArray(new double[7]);
    }

}

