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
  // control outputs
  private final MotionMagicVoltage m_output = new MotionMagicVoltage(0);

  // deployer sysid routine
  private final VoltageOut m_sysIdOutput = new VoltageOut(0);
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, // Default
      null, // Reduce dynamic voltage to 4 to prevent motor brownout
      null), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        m_motor.setControl(m_sysIdOutput.withOutput(volts.in(Volts)));
      }, log -> {
        log.motor("deployer")
            .voltage(m_appliedVoltage
                .mut_replace(m_motor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(
                m_angle.mut_replace(m_motor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(m_velocity.mut_replace(m_motor.getVelocity().getValueAsDouble(),
                RotationsPerSecond));
      }, this));

  /**
   * @brief quasistatic sysid routine
   * 
   *        Quasistatic routines accelerate the motor slowly to measure static friction and other
   *        non-linear effects. Acceleration is kept low so its effect is negligible.
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
   *        Dynamic routines accelerate the motor quickly to measure dynamic friction and other
   *        non-linear effects.
   * 
   * @param direction the direction of the sysid routine
   * @return Command
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

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
    // invert motors
    config.MotorOutput.Inverted = kMotorInverted;
    // set motor ratios
    config.Feedback.SensorToMechanismRatio = kRatio;
    // apply configuration
    m_motor.getConfigurator().apply(config);
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
    // target position
    builder.addDoubleProperty("Target Position", () -> m_output.Position, null);
  }
}
