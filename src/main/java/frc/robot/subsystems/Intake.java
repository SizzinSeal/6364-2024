package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.Constants.Intake.*;

/**
 * @brief Intake Subsystem
 * 
 */
public class Intake extends SubsystemBase {
  // init motors
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // control outputs
  private final VoltageOut m_output = new VoltageOut(0);
  // simulation objects
  private final TalonFXSimState m_motorSimState = m_motor.getSimState();
  private final DCMotorSim m_motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);

  /**
   * @brief IntakeSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Intake() {
    super();
    // configure motors
    final TalonFXConfiguration config = new TalonFXConfiguration();
    // set motor inversions
    config.MotorOutput.Inverted = kMotorInverted;
    // set motor ratios
    config.Feedback.SensorToMechanismRatio = kRatio;
    // apply configuration
    m_motor.getConfigurator().apply(config);
    SmartDashboard.putData("Intake", this.intake());
    SmartDashboard.putData("Outtake", this.outtake());
    SmartDashboard.putData("Stop", this.stop());
  }

  /**
   * @brief set the speed of the intake
   * @param speed
   * @return
   */
  public void setSpeed(double speed) {
    m_output.Output = speed;
    m_motor.setControl(m_output);
  }

  /**
   * @brief Spin the intake motors to outtake notes
   * 
   * @return Command
   */
  public Command outtake() {
    return this.startEnd(() -> this.setSpeed(-kOuttakeSpeed), () -> this.setSpeed(0));
  }

  /**
   * @brief Spin the intake motors to intake notes
   * 
   * @return Command
   */
  public Command intake() {
    return this.runOnce(() -> this.setSpeed(kIntakeSpeed));
  }

  /**
   * @brief Stop the intake motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.runOnce(() -> setSpeed(0));
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
      m_motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      // set motor sim input voltage
      m_motorSim.setInputVoltage(m_motorSimState.getMotorVoltage());
      // update motor sim
      m_motorSim.update(kSimLoopPeriod);
      // update motor sim state
      m_motorSimState.setRawRotorPosition(m_motorSim.getAngularPositionRotations());
      m_motorSimState.setRotorVelocity(m_motorSim.getAngularVelocityRPM() / 60.0);
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
    // measured velocity
    builder.addDoubleProperty("Measured Velocity", () -> m_motor.getVelocity().getValueAsDouble(),
        null);
    // target velocity
    builder.addDoubleProperty("Target Velocity", () -> m_output.Output,
        (double target) -> this.setSpeed(target));
  }
}
