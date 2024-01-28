package frc.robot;

import java.util.function.Supplier;
import javax.sound.sampled.DataLine;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  private Field2d m_field = new Field2d();
  public final Vision limelight1 = new Vision("limelight");
  private double k_pi = Math.PI;

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, modules);
    if (Utils.isSimulation()) {
      startSimThread();
    }
  }

  public double getPoseDifference(Pose2d pos) {
    return m_odometry.getEstimatedPosition().getTranslation().getDistance(pos.getTranslation());
  }

  private double fmod(double x, double y) {
    return (x - Math.floor((x / y)) * y);
  }

  public double calculateTurnToVelocity(double posx, double posy) {
    double k_ki = 0;
    double k_kd = 0.02;
    double k_kp = 30;
    double velocity = 0;


    Pose2d pos;
    pos = m_odometry.getEstimatedPosition();
    double theta = pos.getRotation().getDegrees();

    if (theta < 0) {
      theta += 360;
    }

    System.out.println("theta" + theta);

    double targTheta = fmod(Math.toDegrees((k_pi) / 2 - Math.atan2(posy, posx)), 360);
    double deltaTheta = Math.toRadians(Math.IEEEremainder(targTheta - theta, 360));

    if (deltaTheta > Math.PI) {
      deltaTheta -= 2 * Math.PI;
    } else if (deltaTheta < -Math.PI) {
      deltaTheta += 2 * Math.PI;
    }

    System.out.println("dtheta" + deltaTheta);

    PhoenixPIDController velocityTheta = new PhoenixPIDController(k_kp, k_ki, k_kd);
    if ((posx - pos.getX() < 1) && ((posy - pos.getY()) < 1)) {
      velocity = velocityTheta.calculate(Math.toRadians(deltaTheta), 0, Timer.getFPGATimestamp());
    }

    return (velocity);
  }

  public double[] calculateMoveToPointvelocity(double targX,
      double targY) /* [1] is vy, [0] is vx */ {
    double[] internal = new double[2];
    Pose2d pos = m_odometry.getEstimatedPosition();
    double k_ki = 0;
    double k_kd = 0;
    double k_kp = 0.5;

    PhoenixPIDController velocityX = new PhoenixPIDController(k_kp, k_ki, k_kd);

    PhoenixPIDController velocityY = new PhoenixPIDController(k_kp, k_ki, k_kd);

    // x velocity
    internal[0] = velocityX.calculate(pos.getX(), targX, Timer.getFPGATimestamp());
    // y velocity
    internal[1] = velocityY.calculate(pos.getY(), targY, Timer.getFPGATimestamp());

    // System.out.println("vx" + internal[0]);
    // System.out.println("vy:" + internal[1]);

    return internal;
  }

  public Pose2d calculategoalpos() {
    return new Pose2d(2.5, 5.5, new Rotation2d(0));
  }

  public void turnTo() {

  }

  public double getspeed() {

    double vx = m_kinematics.toChassisSpeeds().vxMetersPerSecond;
    double vy = m_kinematics.toChassisSpeeds().vyMetersPerSecond;

    double vt = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));

    return vt;
  }

  public void updateVision(Pose2d pos, double xyStds, double degStds, double timestamp) {

    m_odometry.addVisionMeasurement(pos, timestamp,
        VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));

  }

  public void updateFieldVisualiser() {
    m_field.setRobotPose(m_odometry.getEstimatedPosition());
  }

  public Pose2d getPos2D() {
    return m_odometry.getEstimatedPosition();
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }
}
