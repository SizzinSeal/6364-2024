package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;

public class PerspectiveProjection {

  // x y and z coordinates of the shooter relative to the center of the robot
  private double k_fwx;
  private double k_fwy;
  private Pose3d k_pointpos;
  // fw stands for flywheel

  PerspectiveProjection(double fwoffsetx, double fwoffsety, Translation3d pointpos) {
    k_fwx = fwoffsetx;
    k_fwy = fwoffsety;
    k_pointpos = new Pose3d(pointpos, new Rotation3d(0, 0, 0));
    // TODO: make fwz = calculate height of flyhweel
  }

  private double[] calculateVanishingPoint(double angle,
      double fwz/* make function to calculate height of the flywheel */) {
    // Assuming the angle is the angle of view and horizonHeight is the y-coordinate of the horizon
    double vanishingPointX = Math.tan(Math.toRadians(angle / 2.0));
    double vanishingPointY = fwz;

    return new double[] {vanishingPointX, vanishingPointY};
  }

  public Pose3d getpointpos() {
    return k_pointpos;
  }

  // x y z are goal relative not robot relative
  public double[] Calculate(double fwpitch, double fwheight) {
    double x = k_pointpos.getX();
    double y = k_pointpos.getY();
    double z = k_pointpos.getZ();

    // Get robot's heading (rotation) in radians
    Pose2d robotpos;
    robotpos = RobotContainer.m_drivetrain.getPose();


    double robotHeading = robotpos.getRotation().getRadians();

    // Transform the goal position into the robot's local coordinate system
    double relativeGoalX = (x - robotpos.getTranslation().getX()) * Math.cos(-robotHeading)
        - (y - robotpos.getTranslation().getY()) * Math.sin(-robotHeading);
    double relativeGoalY = (x - robotpos.getTranslation().getX()) * Math.sin(-robotHeading)
        + (y - robotpos.getTranslation().getY()) * Math.cos(-robotHeading);

    // Calculate distance to the plane (projection plane)
    double distanceToPlane = robotpos.getTranslation().getDistance(new Pose2d(k_pointpos.getX(),
        k_pointpos.getY(), new Rotation2d(k_pointpos.getRotation().getAngle())).getTranslation());

    // Calculate perspective projection
    double[] vanishingPoint = calculateVanishingPoint(fwpitch, fwheight);
    double perspectiveX = k_fwx + (relativeGoalX - k_fwx) * distanceToPlane / z;
    double perspectiveY = k_fwy + (relativeGoalY - k_fwy) * distanceToPlane / z;

    // Adjust based on vanishing point
    perspectiveX += vanishingPoint[0];
    perspectiveY += vanishingPoint[1];

    return new double[] {perspectiveX, perspectiveY};
    // plane is perpendicular to the 2d plane of the robot pos
  }

}

