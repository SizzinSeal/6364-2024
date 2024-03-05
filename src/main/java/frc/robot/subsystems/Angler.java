package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Angler.*;

public class Angler extends SubsystemBase {
  // init motors
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // init sensors
  private final DigitalInput m_limit = new DigitalInput(kLimitPort);
  // control outputs
  private final MotionMagicVoltage m_output = new MotionMagicVoltage(0);

  // sysid routine
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
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Angler() {
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
    // set Motion Magic settings
    final MotionMagicConfigs motionMagicConfig = config.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = kMaxSpeed; // rps
    motionMagicConfig.MotionMagicAcceleration = kAcceleration; // rps^2
    motionMagicConfig.MotionMagicJerk = kJerk; // rps^3
    // apply configuration
    m_motor.getConfigurator().apply(config);
    // set 0 position
    m_motor.setPosition(0);
    // post commands to smart dashboard
    SmartDashboard.putData("Calibrate Angler", this.calibrate());
  }

  /**
   * @brief whether the angler is at the target position or not
   * 
   * @return boolean
   */
  public boolean atTarget() {
    return Math.abs(m_output.Position - m_motor.getPosition().getValueAsDouble()) < kTolerance;
  }

  /**
   * @brief calibrate the angler
   * 
   * @return Command
   */
  public Command calibrate() {
    return this.runOnce(() -> {
      m_motor.setControl(new VelocityVoltage(-kProbeInitialSpeed));
    }).andThen(Commands.waitUntil(() -> m_limit.get()))
        .andThen(() -> m_motor.setControl(new VelocityVoltage(kProbeInitialSpeed))).withTimeout(0.5)
        .andThen(() -> m_motor.setControl(new VelocityVoltage(-kProbeFinalSpeed)))
        .andThen(Commands.waitUntil(() -> m_limit.get()))
        .andThen(() -> m_motor.setControl(new VoltageOut(0))).andThen(() -> m_motor.setPosition(0));
  }

  public Command setSpeed(double speed) {
    return this.runOnce(() -> {
      m_motor.setControl(new VoltageOut(speed));
    });
  }

  public Boolean getLimit() {
    return m_limit.get();
  }

  public Command zero() {
    return this.runOnce(() -> m_motor.setPosition(0));
  }

  /**
   * @brief Go the the specified angle
   * 
   * @param angle angle in rotations
   * @return Command
   */
  public Command goToAngle(double angle) {
    return this.runOnce(() -> {
      m_output.Position = angle;
      m_motor.setControl(m_output);
    });
  }

  /**
   * @brief Go to the shooting position
   * 
   * @return Command
   */
  public Command goToShoot() {
    return this.goToAngle(kShootingPosition);
  }

  /**
   * @brief Go to the loading position
   * 
   * @return Command
   */
  public Command goToLoad() {
    return this.goToAngle(kLoadingPosition);
  }

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
    // target position
    builder.addDoubleProperty("Target Position", () -> m_output.Position,
        (double target) -> this.goToAngle(target).schedule());
    // limit switch
    builder.addBooleanProperty("Limit Triggered", () -> m_limit.get(), null);
  }
}
