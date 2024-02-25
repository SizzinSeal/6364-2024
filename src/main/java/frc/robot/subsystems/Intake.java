package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  // init motors
  private final TalonFX m_intakeMotor = new TalonFX(kIntakeId, kIntakeBus);
  private final TalonFX m_deployerMotor = new TalonFX(kDeployerId, kDeployerBus);
  // control output objects
  private final VoltageOut m_intakeVelocity = new VoltageOut(0);
  // simulation objects
  private final TalonFXSimState m_intakeMotorSimState = m_intakeMotor.getSimState();
  private final DCMotorSim m_intakeMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);

  // sysid
  private final VoltageOut m_deployerSysIdControl = new VoltageOut(0);
  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_deployerAppliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Angle> m_deployerAngle = mutable(Rotations.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutableMeasure<Velocity<Angle>> m_deployerVelocity =
      mutable(RotationsPerSecond.of(0));
  // deployer sysid routine
  private final SysIdRoutine m_deployerSysIdRoutine = new SysIdRoutine(new SysIdRoutine.Config(null, // Default
      null, // Reduce dynamic voltage to 4 to prevent motor brownout
      null), new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        m_deployerMotor.setControl(m_deployerSysIdControl.withOutput(volts.in(Volts)));
      }, log -> {
        log.motor("deployer")
            .voltage(m_deployerAppliedVoltage
                .mut_replace(m_deployerMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(m_deployerAngle
                .mut_replace(m_deployerMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(m_deployerVelocity
                .mut_replace(m_deployerMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
      }, this));

  /**
   * @brief deployer Quasistatic sysid routine
   * 
   *        Quasistatic routines accelerate the motor slowly to measure static friction and other
   *        non-linear effects. Acceleration is kept low so its effect is negligible.
   * 
   * @param direction the direction of the sysid routine
   * @return Command
   */
  public Command deployerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_deployerSysIdRoutine.quasistatic(direction);
  }

  /**
   * @brief deployer Dynamic sysid routine
   * 
   *        Dynamic routines accelerate the motor quickly to measure dynamic friction and other
   *        non-linear effects.
   * 
   * @param direction the direction of the sysid routine
   * @return Command
   */
  public Command deployerSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_deployerSysIdRoutine.dynamic(direction);
  }

  /**
   * @brief IntakeSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Intake() {
    super();
    // configure motors
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    TalonFXConfiguration deployerConfig = new TalonFXConfiguration();
    // set controller gains
    intakeConfig.Slot0 = kIntakeControllerConstants;
    deployerConfig.Slot0 = kDeployerControllerConstants;
    // invert motors
    intakeConfig.MotorOutput.Inverted = kIntakeInverted;
    deployerConfig.MotorOutput.Inverted = kDeployerInverted;
    // apply configuration
    m_intakeMotor.getConfigurator().apply(intakeConfig);
    m_deployerMotor.getConfigurator().apply(deployerConfig);
    // set deployer position to 0
    m_deployerMotor.setPosition(0);
  }

  /**
   * @brief Update motor speeds
   * 
   *        This is where we actually set the motor speeds. We do this in a seperate method to
   *        simplify the commands that change the target velocity.
   */
  private void updateMotorSpeeds() {
    m_intakeMotor.setControl(m_intakeVelocity);
  }

  /**
   * @brief Spin the intake motors to outake notes
   * 
   * @return Command
   */
  public Command outake() {
    return this.runOnce(() -> {
      m_intakeVelocity.Output = -kIntakeSpeed;
      this.updateMotorSpeeds();
    });
  }

  /**
   * @brief Spin the intake motors to intake notes
   * 
   * @return Command
   */
  public Command intake() {
    return this.runOnce(() -> {
      m_intakeVelocity.Output = -kIntakeSpeed;
      this.updateMotorSpeeds();
    });
  }

  /**
   * @brief Stop the intake motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.runOnce(() -> {
      m_intakeVelocity.Output = 0;
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
      m_intakeMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      // set motor sim input voltage
      m_intakeMotorSim.setInputVoltage(m_intakeMotorSimState.getMotorVoltage());
      // update motor sim
      m_intakeMotorSim.update(kSimLoopPeriod);
      // update motor sim state
      m_intakeMotorSimState.setRawRotorPosition(m_intakeMotorSim.getAngularPositionRotations());
      m_intakeMotorSimState.setRotorVelocity(m_intakeMotorSim.getAngularVelocityRPM() / 60.0);
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
    builder.addDoubleProperty("Upper Target Velocity", () -> m_intakeVelocity.Output,
        (double target) -> {
          m_intakeVelocity.Output = target;
          this.updateMotorSpeeds();
        });
    // add upper motor measured velocity property
    builder.addDoubleProperty("Upper Measured Velocity",
        () -> m_intakeMotor.getVelocity().getValueAsDouble(), null);
    // add deployer position property
    builder.addDoubleProperty("Deployer Position",
        () -> m_deployerMotor.getPosition().getValueAsDouble(), null);
  }
}
