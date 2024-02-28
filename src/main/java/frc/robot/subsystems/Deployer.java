package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.constants.Deployer.*;

/**
 * @brief Intake Subsystem
 * 
 */
public class Deployer extends SubsystemBase {
  // init motors
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // init output
  private final VoltageOut m_output = new VoltageOut(0);

  /**
   * @brief IntakeSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Deployer() {
    super();
    // configure motors
    final TalonFXConfiguration config = new TalonFXConfiguration();
    // set controller gains
    config.Slot0 = kControllerGains;
    config.Slot0.GravityType = kGravityType;
    config.Slot0.kG = kG;
    // invert motors
    config.MotorOutput.Inverted = kMotorInverted;
    // set motor ratios
    config.Feedback.SensorToMechanismRatio = kRatio;
    // apply configuration
    m_motor.getConfigurator().apply(config);
    // set motor position to 0
    m_motor.setPosition(0);
  }

  /**
   * @brief whether the deployer is deployed or not
   * 
   * @return true if its deployed, false otherwise
   */
  public boolean isDeployed() {
    return Math.abs(kDownPosition - m_motor.getPosition().getValueAsDouble()) < kTolerance;
  }

  /**
   * @brief whether the deployer is retracted or not
   * 
   * @return true if its retracted, false otherwise
   */
  public boolean isRetracted() {
    return Math.abs(kUpPosition - m_motor.getPosition().getValueAsDouble()) < kTolerance;
  }

  /**
   * @brief deploy the deployer
   * 
   * @return Command
   */
  public Command down() {
    return this.runOnce(() -> {
      m_output.Output = -0.5;
      m_motor.setControl(m_output);
    });
  }

  /**
   * @brief retract the deployer
   * 
   * @return Command
   */
  public Command up() {
    return this.runOnce(() -> {
      m_output.Output = 0.5;
      m_motor.setControl(m_output);
    });
  }

  public Command stop() {
    return this.runOnce(() -> {
      m_output.Output = 0;
      m_motor.setControl(m_output);
    });
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
    // measured position
    builder.addDoubleProperty("Position", () -> m_motor.getPosition().getValueAsDouble(), null);
  }
}
