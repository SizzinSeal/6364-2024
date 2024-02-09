package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
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

  /**
   * @brief get the scalar distance between the robot and a position
   */
  public double getPoseDifference(Pose2d pos) {
    return m_odometry.getEstimatedPosition().getTranslation().getDistance(pos.getTranslation());
  }

  /**
   * @brief get the scalar speed of the robot in meters per second
   * 
   * @return double
   */
  public double getspeed() {
    double vx = m_kinematics.toChassisSpeeds().vxMetersPerSecond;
    double vy = m_kinematics.toChassisSpeeds().vyMetersPerSecond;
    double vt = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
    return vt;
  }

  public Pose2d getPose2d() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * @brief update the odometry with a vision measurement
   * 
   * @param pos the position of the robot
   * @param xyStds the standard deviation of the x and y measurements
   * @param degStds the standard deviation of the angle measurement
   * @param timestamp the timestamp of the measurement
   */
  public void updateVision(Pose2d pos, double xyStds, double degStds, double timestamp) {
    m_odometry.addVisionMeasurement(pos, timestamp,
        VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds)));
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
