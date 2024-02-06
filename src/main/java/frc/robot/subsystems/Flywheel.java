package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.Constants.Flywheel.*;


/**
 * @brief Flywheel Subsystem
 * 
 */
public class Flywheel extends SubsystemBase {
  private final double kSimLoopPeriod = 0.005; // 5 ms
  // init motors
  private final TalonFX m_upperMotor = new TalonFX(kUpperMotorId, kUpperBusName);
  private final TalonFX m_lowerMotor = new TalonFX(kLowerMotorId, kLowerBusName);
  // control output objects
  private final VoltageOut m_upperMotorVelocity = new VoltageOut(0);
  private final VoltageOut m_lowerMotorVelocity = new VoltageOut(0);
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
    TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
    // set controller gains
    upperConfig.Slot0 = kUpperControllerConstants;
    lowerConfig.Slot0 = kLowerControllerConstants;
    // invert motors
    upperConfig.MotorOutput.Inverted = kUpperInverted;
    lowerConfig.MotorOutput.Inverted = kLowerInverted;
    // apply configuration
    m_upperMotor.getConfigurator().apply((upperConfig));
    m_lowerMotor.getConfigurator().apply((lowerConfig));
    m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    m_lowerMotor.setNeutralMode(NeutralModeValue.Brake);
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
    return Math.abs(m_upperMotor.getVelocity().getValueAsDouble() - kUpperSpeed) < kUpperTolerance
        && Math.abs(m_lowerMotor.getVelocity().getValueAsDouble() - kLowerSpeed) < kLowerTolerance;
  }

  /**
   * @brief Update motor speeds
   * 
   *        This is where we actually set the motor speeds. We do this in a seperate method to
   *        simplify the commands that change the target velocity.
   */
  private void updateMotorSpeeds() {
    m_upperMotor.setControl(m_upperMotorVelocity);
    m_lowerMotor.setControl(m_lowerMotorVelocity);
  }

  /**
   * @brief Spin the intake motors to outake notes
   * 
   * @return Command
   */
  public Command reverse() {
    return this.runOnce(() -> {
      m_upperMotorVelocity.Output = -kUpperSpeed;
      m_lowerMotorVelocity.Output = -kLowerSpeed;
      this.updateMotorSpeeds();
    });
  }

  /**
   * @brief Spin up the flywheel motors
   * 
   * @return Command
   */
  public Command forwards() {
    return this.runOnce(() -> {
      m_upperMotorVelocity.Output = kUpperSpeed;
      m_lowerMotorVelocity.Output = kLowerSpeed;
      this.updateMotorSpeeds();
    });
  }

  /**
   * @brief Stop the flywheel motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.runOnce(() -> {
      m_upperMotorVelocity.Output = 0;
      m_lowerMotorVelocity.Output = 0;
      this.updateMotorSpeeds();
    });
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
    builder.addDoubleProperty("Upper Target Velocity", () -> m_upperMotorVelocity.Output,
        (double target) -> {
          m_upperMotorVelocity.Output = target;
          this.updateMotorSpeeds();
        });
    // add upper motor measured velocity property
    builder.addDoubleProperty("Upper Measured Velocity",
        () -> m_upperMotor.getVelocity().getValueAsDouble(), null);
    // add lower motor target velocity property
    builder.addDoubleProperty("Lower Target Velocity", () -> m_lowerMotorVelocity.Output,
        (double target) -> {
          m_lowerMotorVelocity.Output = target;
          this.updateMotorSpeeds();
        });
    // add lower motor measured velocity property
    builder.addDoubleProperty("Lower Measured Velocity",
        () -> m_lowerMotor.getVelocity().getValueAsDouble(), null);
  }
}
