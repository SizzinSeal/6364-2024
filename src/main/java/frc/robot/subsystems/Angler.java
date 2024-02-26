package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.Angler.*;

public class Angler extends SubsystemBase {
  // init motors
  private final TalonFX m_motor = new TalonFX(kMotorId, kMotorBus);
  // init sensors
  private final DigitalInput m_limit = new DigitalInput(kLimitPort);
  // control outputs
  private final MotionMagicVoltage m_output = new MotionMagicVoltage(0);

  // deployer sysid routine
  private final VoltageOut m_sysIdOutput = new VoltageOut(0);
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(new SysIdRoutine.Config(kRampRate, kStepVoltage, kTimeout),
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
    config.Slot0 = kControllerGains;
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
    // post commands to smart dashboard
    SmartDashboard.putData("calibrate", this.calibrate());
    SmartDashboard.putData("Shooting Angle", this.goToShoot());
    SmartDashboard.putData("Loading Angle", this.goToLoad());
    SmartDashboard.putData("Quasistatic Routine Up",
        this.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Quasistatic Routine Down",
        this.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("Dynamic Routine Up",
        this.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("Dynamic Routine Down",
        this.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
    return this.runOnce(() -> m_motor.setControl(new VelocityVoltage(-kManualSpeed)))
        .until(() -> m_limit.get())
        .andThen(() -> m_motor.setControl(new VelocityVoltage(kManualSpeed))).withTimeout(0.5)
        .andThen(() -> m_motor.setControl(new VelocityVoltage(-kProbeSpeed)))
        .until(() -> m_limit.get()).andThen(() -> m_motor.setControl(new VoltageOut(0)))
        .andThen(() -> m_motor.setPosition(0));
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
   * @brief Go up manually
   * 
   * @return Command
   */
  public Command manualUp() {
    return this.startEnd(() -> m_motor.setControl(new VelocityVoltage(kManualSpeed)),
        () -> m_motor.setControl(new VelocityVoltage(0)));
  }

  /**
   * @brief Go up manually
   * 
   * @return Command
   */
  public Command manualDown() {
    return this.startEnd(() -> m_motor.setControl(new VelocityVoltage(-kManualSpeed)),
        () -> m_motor.setControl(new VelocityVoltage(0)));
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
    builder.addDoubleProperty("Position", () -> m_motor.getPosition().getValueAsDouble(), null);
    // target position
    builder.addDoubleProperty("Target Position", () -> m_output.Position,
        (double target) -> this.goToAngle(target));
  }
}
