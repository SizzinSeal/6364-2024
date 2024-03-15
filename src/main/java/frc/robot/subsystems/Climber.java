package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import static frc.robot.Constants.Climber.*;

/**
 * @brief Intake Subsystem
 * 
 */
public class Climber extends SubsystemBase {
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
  public Climber() {
    super();
    // configure motors
    final TalonFXConfiguration config = new TalonFXConfiguration();
    // set controller gains
    config.Slot0 = new Slot0Configs().withKP(kP).withKI(kI).withKD(kD).withKS(kS).withKV(kV)
        .withKA(kA).withKG(kG).withGravityType(kGravityType);
    // invert motors
    config.MotorOutput.Inverted = kMotorInverted;
    // set motor ratios
    config.Feedback.SensorToMechanismRatio = kRatio;
    // set brake
    m_motor.setNeutralMode(NeutralModeValue.Brake);
    // apply configuration
    m_motor.getConfigurator().apply(config);
    // set motor position to 0
    m_motor.setPosition(0);
    // brake the motor
    m_motor.setControl(new StaticBrake());
    // commands
    SmartDashboard.putData("Climber Up", up());
    SmartDashboard.putData("Climber Down", down());
    SmartDashboard.putData("Climber Stop", stop());
  }

  /**
   * @brief whether the deployer is deployed or not
   * 
   * @return true if its deployed, false otherwise
   */
  public boolean isDeployed() {
    return m_motor.getPosition().getValueAsDouble() - kTolerance < kMinPosition;
  }

  /**
   * @brief whether the deployer is retracted or not
   * 
   * @return true if its retracted, false otherwise
   */
  public boolean isRetracted() {
    return m_motor.getPosition().getValueAsDouble() + kTolerance > kMaxPosition;
  }

  /**
   * @brief move the deployer down
   * 
   * @return Command
   */
  public Command down() {
    return this.runOnce(() -> {
      m_output.Output = kSpeed;
      m_motor.setControl(m_output);
    });
  }

  /**
   * @brief move the deployer up
   * 
   * @return Command
   */
  public Command up() {
    return this.runOnce(() -> {
      m_output.Output = -kSpeed;
      m_motor.setControl(m_output);
    });
  }

  /**
   * @brief stop the deployer from moving
   * 
   * @return Command
   */
  public Command stop() {
    return this.runOnce(() -> {
      m_output.Output = 0;
      m_motor.setControl(m_output);
      m_motor.setControl(new StaticBrake());
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
    builder.addDoubleProperty("Position", () -> m_motor.getPosition().getValueAsDouble(),
        (double position) -> m_motor.setPosition(position));
  }
}
