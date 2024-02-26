package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.constants.Indexer.*;

/**
 * @brief Intake Subsystem
 * 
 */
public class Indexer extends SubsystemBase {
  // init devices
  private final DigitalInput m_noteDetector = new DigitalInput(kNoteDetectorPort);
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // control output objects
  private final VoltageOut m_output = new VoltageOut(0);
  // simulation objects
  private final TalonFXSimState m_motorSimState = m_motor.getSimState();
  private final DCMotorSim m_motorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);

  /**
   * @brief IndexerSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Indexer() {
    super();
    // configure motors
    final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    // set controller gains
    motorConfig.Slot0 = kMotorControllerConstants;
    // invert motors
    motorConfig.MotorOutput.Inverted = kInverted;
    // set gear ratio
    motorConfig.Feedback.SensorToMechanismRatio = kRatio;
    // apply configuration
    m_motor.getConfigurator().apply((motorConfig));
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * @brief whether a note is detected in the indexer or not
   * 
   * @return true if not detected, false otherwise
   */
  public Boolean noteDetected() {
    return m_noteDetector.get();
  }

  /**
   * @brief set the speed of the indexer
   * 
   * @param speed the speed to move at
   * @return Command
   */
  private void setSpeed(double speed) {
    m_output.Output = speed;
    m_motor.setControl(m_output);
  }

  /**
   * @brief Spin the indexer motors to load a note
   * 
   * @return Command
   */
  public Command load() {
    return Commands.startEnd(() -> this.setSpeed(-kLoadSpeed), () -> this.stop());
  }

  /**
   * @brief Spin the intake motors to eject a note
   * 
   * @return Command
   */
  public Command eject() {
    return this.startEnd(() -> this.setSpeed(kEjectSpeed), () -> this.stop());
  }

  /**
   * @brief Stop the indexer motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.runOnce(() -> this.setSpeed(0));
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
    // add target velocity property
    builder.addDoubleProperty("Target Velocity", () -> m_output.Output,
        (double target) -> this.setSpeed(target));
    // add measured velocity property
    builder.addDoubleProperty("Measured Velocity", () -> m_motor.getVelocity().getValueAsDouble(),
        null);
    builder.addBooleanProperty("Note Detected", () -> this.noteDetected(), null);
  }
}
