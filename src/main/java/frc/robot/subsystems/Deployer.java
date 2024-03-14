package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Deployer.*;

/**
 * @brief Intake Subsystem
 * 
 */
public class Deployer extends SubsystemBase {
  // init motors
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // init output
  private final PositionVoltage m_output = new PositionVoltage(kMaxPosition);

  private final VoltageOut m_sysIdOutput = new VoltageOut(0);
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(kRampRate).per(Second), Volts.of(kStepVoltage),
          Seconds.of(kTimeout)),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        m_motor.setControl(m_sysIdOutput.withOutput(volts.in(Volts)));
      }, log -> {
        log.motor("angler")
            .voltage(m_appliedVoltage
                .mut_replace(m_motor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(
                m_angle.mut_replace(m_motor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(m_velocity.mut_replace(m_motor.getVelocity().getValueAsDouble(),
                RotationsPerSecond));
      }, this));

  /**
   * @brief IntakeSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so
   *        that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix
   *        Tuner.
   */
  public Deployer() {
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
    // set current limit
    config.CurrentLimits.StatorCurrentLimit = kCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = kCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    // set brake
    m_motor.setNeutralMode(NeutralModeValue.Brake);

    // apply configuration
    m_motor.getConfigurator().apply(config);
    // set motor position to 0
    m_motor.setPosition(kMaxPosition);
    // set motor control mode
    m_motor.setControl(m_output);
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

  public Command goToAngle(double position) {
    return this.runOnce(() -> {
      m_output.Position = position;
      m_motor.setControl(m_output);
    });
  }

  public Command deploy() {
    return this.runOnce(() -> {
      m_output.Position = 0;
      m_motor.setControl(m_output);
    });
  }

  public Command retract() {
    return this.runOnce(() -> {
      m_output.Position = kMaxPosition;
      m_motor.setControl(m_output);
    });
  }

  /**
   * @brief move the deployer down
   * 
   * @return Command
   */
  public Command down() {
    return this.runOnce(() -> {
      // m_output.Position = -kSpeed;
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
      // m_output.Output = kSpeed;
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
      // m_output.Output = 0;
      m_motor.setControl(m_output);
      m_motor.setNeutralMode(NeutralModeValue.Brake);
    });
  }

  /**
   * @brief quasistatic sysid routine
   * 
   *        Quasistatic routines accelerate the motor slowly to measure static
   *        friction and other
   *        non-linear effects. Acceleration is kept low so its effect is
   *        negligible.
   * 
   * @param direction the direction of the sysid routine
   * @return Command
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  /**
   * @brief dynamic sysid routine
   * 
   *        Dynamic routines accelerate the motor quickly to measure dynamic
   *        friction and other
   *        non-linear effects.
   * 
   * @param direction the direction of the sysid routine
   * @return Command
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

  /**
   * @brief Send telemetry data to Shuffleboard
   * 
   *        The SendableBuilder object is used to send data to Shuffleboard. We
   *        use it to send the
   *        target velocity of the motors, as well as the measured velocity of the
   *        motors. This
   *        allows us to tune intake speed in real time, without having to
   *        re-deploy code.
   * 
   * @param builder the SendableBuilder object
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder); // call the superclass method
    // measured position
    builder.addDoubleProperty("Position", () -> m_motor.getPosition().getValueAsDouble(),
        (double position) -> m_motor.setPosition(position));
    builder.addDoubleProperty("Target Position", () -> m_output.Position,
        (double target) -> this.goToAngle(target).schedule());
  }
}
