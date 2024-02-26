package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.constants.Flywheel.*;

/**
 * @brief Flywheel Subsystem
 * 
 */
public class Flywheel extends SubsystemBase {
  // init motors
  private final TalonFX m_upperMotor = new TalonFX(kUpperMotorId, kUpperMotorBus);
  private final TalonFX m_lowerMotor = new TalonFX(kLowerMotorId, kLowerMotorBus);
  // control output objects
  private final VoltageOut m_upperOutput = new VoltageOut(0);
  private final VoltageOut m_lowerOutput = new VoltageOut(0);
  // simulation objects
  private final TalonFXSimState m_upperMotorSimState = m_upperMotor.getSimState();
  private final TalonFXSimState m_lowerMotorSimState = m_lowerMotor.getSimState();
  private final DCMotorSim m_upperMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);
  private final DCMotorSim m_lowerMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);

  /**
   * @brief FlywheelSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Flywheel() {
    super();
    // configure motors
    final TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    final TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
    // set controller gains
    upperConfig.Slot0 = kUpperControllerGains;
    lowerConfig.Slot0 = kLowerControllerGains;
    // invert motors
    upperConfig.MotorOutput.Inverted = kUpperMotorInverted;
    lowerConfig.MotorOutput.Inverted = kLowerMotorInverted;
    // set ratios
    upperConfig.Feedback.SensorToMechanismRatio = kUpperRatio;
    lowerConfig.Feedback.SensorToMechanismRatio = kLowerRatio;
    // set neutral modes
    m_upperMotor.setNeutralMode(kUpperNeutralMode);
    m_lowerMotor.setNeutralMode(kLowerNeutralMode);
    // apply configuration
    m_upperMotor.getConfigurator().apply((upperConfig));
    m_lowerMotor.getConfigurator().apply((lowerConfig));
  }

  /**
   * @brief Get the upper motor velocity
   * 
   * @return double
   */
  public double getUpperVelocity() {
    return m_upperMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @brief Get the lower motor velocity
   * 
   * @return double
   */
  public double getLowerVelocity() {
    return m_lowerMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @brief Check if the flywheel is at speed
   * 
   * @return boolean
   */
  public Boolean isAtSpeed() {
    return Math.abs(kUpperSpeed - m_upperMotor.getVelocity().getValueAsDouble()) < kUpperTolerance
        && Math.abs(kLowerSpeed - m_lowerMotor.getVelocity().getValueAsDouble()) < kLowerTolerance;
  }

  public Command setUpperSpeed(double speed) {
    return this.runOnce(() -> {
      m_upperOutput.Output = speed;
      m_upperMotor.setControl(m_upperOutput);
    });
  }

  public Command setLowerSpeed(double speed) {
    return this.runOnce(() -> {
      m_lowerOutput.Output = speed;
      m_lowerMotor.setControl(m_lowerOutput);
    });
  }

  public Command setSpeed(double speed) {
    return this.runOnce(() -> {
      this.setUpperSpeed(speed);
      this.setLowerSpeed(speed);
    });
  }

  /**
   * @brief Spin the intake motors to outake notes
   * 
   * @return Command
   */
  public Command reverse() {
    return this.runOnce(() -> {
      this.setUpperSpeed(-kUpperSpeed);
      this.setLowerSpeed(-kLowerSpeed);
    });
  }

  /**
   * @brief Spin up the flywheel motors
   * 
   * @return Command
   */
  public Command forwards() {
    return this.runOnce(() -> {
      this.setUpperSpeed(kUpperSpeed);
      this.setLowerSpeed(kLowerSpeed);
    });
  }

  /**
   * @brief Stop the flywheel motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.setSpeed(0);
  }

  /**
   * @brief periodic update method
   * 
   *        This method is called periodically by the scheduler. We use it to update the simulated
   *        motors.
   */
  @Override
  public void periodic() {
    if (Utils.isSimulation()) {
      // update simulated motors
      // set supply voltage (voltage of the simulated battery)
      m_upperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      m_lowerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      // set motor sim input voltage
      m_upperMotorSim.setInputVoltage(m_upperMotorSimState.getMotorVoltage());
      m_lowerMotorSim.setInputVoltage(m_lowerMotorSimState.getMotorVoltage());
      // update motor sim
      m_upperMotorSim.update(kSimLoopPeriod);
      m_lowerMotorSim.update(kSimLoopPeriod);
      // update motor sim state
      m_upperMotorSimState.setRawRotorPosition(m_upperMotorSim.getAngularPositionRotations());
      m_upperMotorSimState.setRotorVelocity(m_upperMotorSim.getAngularVelocityRPM() / 60.0);
      m_lowerMotorSimState.setRawRotorPosition(m_lowerMotorSim.getAngularPositionRotations());
      m_lowerMotorSimState.setRotorVelocity(m_lowerMotorSim.getAngularVelocityRPM() / 60.0);
    }
  }

  /**
   * @brief Send telemetry data to Shuffleboard
   * 
   *        The SendableBuilder object is used to send data to Shuffleboard. We use it to send the
   *        target velocity of the motors, as well as the measured velocity of the motors. This
   *        allows us to tune intake speed in real time, without having to re-deploy code.
   * 
   * @param builder the SendableBuilder object
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // call the superclass method
    // add upper motor target velocity property
    builder.addDoubleProperty("Upper Target Velocity", () -> m_upperOutput.Output,
        (double target) -> this.setUpperSpeed(target));
    // add upper motor measured velocity property
    builder.addDoubleProperty("Upper Measured Velocity",
        () -> m_upperMotor.getVelocity().getValueAsDouble(), null);
    // add lower motor target velocity property
    builder.addDoubleProperty("Lower Target Velocity", () -> m_lowerOutput.Output,
        (double target) -> this.setLowerSpeed(target));
    // add lower motor measured velocity property
    builder.addDoubleProperty("Lower Measured Velocity",
        () -> m_lowerMotor.getVelocity().getValueAsDouble(), null);
  }
}
