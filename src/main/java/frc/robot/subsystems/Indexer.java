package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.Constants.Indexer.*;

/**
 * @brief Intake Subsystem
 * 
 */
public class Indexer extends SubsystemBase {
  // init devices
  private final AnalogInput m_beamBreak = new AnalogInput(kBeamBreakPort);
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // triggers and event loops
  public final Trigger noteDetected = new Trigger(() -> m_beamBreak.getVoltage() < 0.83);
  private final EventLoop m_noteDetectedLoop = new EventLoop();
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
    // invert motors
    motorConfig.MotorOutput.Inverted = kInverted;
    // set gear ratio
    motorConfig.Feedback.SensorToMechanismRatio = kRatio;
    // set brake
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    // apply configuration
    m_motor.getConfigurator().apply((motorConfig));
    // brake the motor
    m_motor.setControl(new StaticBrake());
    // commands
    SmartDashboard.putData("Indexer Load", this.load());
    SmartDashboard.putData("Indexer Slow Load", this.slowLoad());
    SmartDashboard.putData("Indexer Eject", this.eject());
    SmartDashboard.putData("Indexer Reverse", this.reverse());
    SmartDashboard.putData("Indexer Stop", this.stop());
  }

  /**
   * @brief Whether the note is detected or not
   * 
   * @return Boolean
   */
  public Boolean isNoteDetected() {
    return m_beamBreak.getVoltage() < 0.83;
  }

  /**
   * @brief Poll the note detector
   */
  public void pollNoteDetector() {
    m_noteDetectedLoop.poll();
  }

  /**
   * @brief set the speed of the indexer
   * 
   * @param speed the speed to move at
   * @return Command
   */
  public void setSpeed(double speed) {
    m_output.Output = speed;
    m_motor.setControl(m_output);
  }

  /**
   * @brief Spin the indexer motors to load a note
   * 
   * @return Command
   */
  public Command load() {
    return Commands.runOnce(() -> this.setSpeed(kLoadSpeed));
  }

  /**
   * @brief Spin the indexer motors to load a note slowly
   * 
   * @return Command
   */
  public Command slowLoad() {
    return Commands.runOnce(() -> this.setSpeed(kSlowLoadSpeed));
  }

  /**
   * @brief Spin the intake motors to eject a note
   * 
   * @return Command
   */
  public Command eject() {
    return this.runOnce(() -> this.setSpeed(kEjectSpeed));
  }

  /**
   * @brief Reverse the indexer. Only used in case of a jam
   * 
   * @return Command
   */
  public Command reverse() {
    return this.runOnce(() -> this.setSpeed(-kLoadSpeed));
  }

  /**
   * @brief Stop the indexer motors
   * 
   */
  public Command stop() {
    return this.runOnce(() -> {
      m_output.Output = 0;
      m_motor.setControl(m_output);
      m_motor.setControl(new StaticBrake());
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
    builder.addBooleanProperty("Note Detected", () -> m_beamBreak.getVoltage() < 0.83, null);
  }
}
