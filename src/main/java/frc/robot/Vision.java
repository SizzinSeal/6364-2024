package frc.robot;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.Subscriber;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.cscore.VideoListener;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;




public class Vision extends LimelightHelpers{
    // private static NetworkTableInstance inst = NetworkTableInstance.getDefault();
    // private static NetworkTable vistable = inst.getTable("limelight"); // Declare Vision Table at the class level
    // private static NetworkTableEntry internalPosEntry = vistable.getEntry("<botpose>");
    private DoubleArraySubscriber DASub;

    public Vision(String name){
       NetworkTableInstance inst = NetworkTableInstance.getDefault();
       DASub = inst.getDoubleArrayTopic(name).subscribe(new double[6], PubSubOption.keepDuplicates(true), PubSubOption.pollStorage(1));
    }

    public Pose2d getPos2D(){
        TimestampedDoubleArray DASubT = DASub.getAtomic(null);
        double[] DASubTpos = DASubT.value;

        Translation2d coords = new Translation2d(DASubTpos[0], DASubTpos[1]);
        Rotation2d rot = new Rotation2d(Units.degreesToRadians(DASubTpos[5]));

        Pose2d fpos2d = new Pose2d(coords, rot);
        return fpos2d;
    }

    public double getLatestTimestamp(){
        TimestampedDoubleArray DASubT = DASub.getAtomic(null);
        return DASubT.timestamp;
    }

    public TimestampedDoubleArray getPoseRaw(){
        TimestampedDoubleArray DASubT = DASub.getAtomic(null);
        return DASubT;
    }

}

