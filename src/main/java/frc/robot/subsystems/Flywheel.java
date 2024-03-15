package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Flywheel.*;

/**
 * @brief Flywheel Subsystem
 * 
 */
public class Flywheel extends SubsystemBase {
  // init motors
  private final TalonFX m_upperMotor = new TalonFX(kUpperMotorId, kUpperMotorBus);
  private final TalonFX m_lowerMotor = new TalonFX(kLowerMotorId, kLowerMotorBus);
  // control output objects
  private final VelocityVoltage m_upperOutput = new VelocityVoltage(kUpperSpeed);
  private final VelocityVoltage m_lowerOutput = new VelocityVoltage(kLowerSpeed);
  // simulation objects
  private final TalonFXSimState m_upperMotorSimState = m_upperMotor.getSimState();
  private final TalonFXSimState m_lowerMotorSimState = m_lowerMotor.getSimState();
  private final DCMotorSim m_upperMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);
  private final DCMotorSim m_lowerMotorSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.001);
  // upper sysid routine
  private final VoltageOut m_upperSysIdOutput = new VoltageOut(0);
  private final MutableMeasure<Voltage> m_upperAppliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_upperAngle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_upperVelocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_upperSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(kRampRate).per(Second), Volts.of(kStepVoltage),
          Seconds.of(kTimeout)),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        m_upperMotor.setControl(m_upperSysIdOutput.withOutput(volts.in(Volts)));
      }, log -> {
        log.motor("Upper Flywheel")
            .voltage(m_upperAppliedVoltage
                .mut_replace(m_upperMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(
                m_upperAngle.mut_replace(m_upperMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(m_upperVelocity
                .mut_replace(m_upperMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
      }, this));
  // lower sysid routine
  private final VoltageOut m_lowerSysIdOutput = new VoltageOut(0);
  private final MutableMeasure<Voltage> m_lowerAppliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Angle> m_lowerAngle = mutable(Rotations.of(0));
  private final MutableMeasure<Velocity<Angle>> m_lowerVelocity = mutable(RotationsPerSecond.of(0));
  private final SysIdRoutine m_lowerSysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(Volts.of(kRampRate).per(Second), Volts.of(kStepVoltage),
          Seconds.of(kTimeout)),
      new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
        m_lowerMotor.setControl(m_lowerSysIdOutput.withOutput(volts.in(Volts)));
      }, log -> {
        log.motor("Lower Flywheel")
            .voltage(m_lowerAppliedVoltage
                .mut_replace(m_lowerMotor.get() * RobotController.getBatteryVoltage(), Volts))
            .angularPosition(
                m_lowerAngle.mut_replace(m_lowerMotor.getPosition().getValueAsDouble(), Rotations))
            .angularVelocity(m_lowerVelocity
                .mut_replace(m_lowerMotor.getVelocity().getValueAsDouble(), RotationsPerSecond));
      }, this));

  /**
   * @brief FlywheelSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public Flywheel() {
    super();
    // configure motors
    final TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    final TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
    // set controller gains
    upperConfig.Slot0 = new Slot0Configs().withKP(kUpperKP).withKI(kUpperKI).withKD(kUpperKD)
        .withKS(kUpperKS).withKV(kUpperKV).withKA(kUpperKA);
    lowerConfig.Slot0 = new Slot0Configs().withKP(kLowerKP).withKI(kLowerKI).withKD(kLowerKD)
        .withKS(kLowerKS).withKV(kLowerKV).withKA(kLowerKA);
    // invert motors
    upperConfig.MotorOutput.Inverted = kUpperMotorInverted;
    lowerConfig.MotorOutput.Inverted = kLowerMotorInverted;
    // set ratios
    upperConfig.Feedback.SensorToMechanismRatio = kUpperRatio;
    lowerConfig.Feedback.SensorToMechanismRatio = kLowerRatio;
    // set neutral modes
    m_upperMotor.setNeutralMode(kUpperNeutralMode);
    m_lowerMotor.setNeutralMode(kLowerNeutralMode);
    // apply configuration
    m_upperMotor.getConfigurator().apply((upperConfig));
    m_lowerMotor.getConfigurator().apply((lowerConfig));
    // commands
    SmartDashboard.putData("Flywheel Forwards", this.forwards());
    SmartDashboard.putData("Flywheel Reverse", this.reverse());
    SmartDashboard.putData("Flywheel Stop", this.stop());
  }

  /**
   * @brief Get the upper motor velocity
   * 
   * @return double
   */
  public double getUpperVelocity() {
    return m_upperMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @brief Get the lower motor velocity
   * 
   * @return double
   */
  public double getLowerVelocity() {
    return m_lowerMotor.getVelocity().getValueAsDouble();
  }

  /**
   * @brief Check if the flywheel is at speed
   * 
   * @return boolean
   */
  public Boolean isAtSpeed() {
    return Math.abs(kUpperSpeed - m_upperMotor.getVelocity().getValueAsDouble()) < kUpperTolerance
        && Math.abs(kLowerSpeed - m_lowerMotor.getVelocity().getValueAsDouble()) < kLowerTolerance;
  }

  /**
   * @brief set the speed of the upper motor
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  public void setUpperSpeed(double speed) {
    m_upperOutput.Velocity = speed;
    m_upperMotor.setControl(m_upperOutput);
    m_upperMotor.setNeutralMode(NeutralModeValue.Brake);
    if (speed == 0.0)
      m_lowerMotor.setControl(new StaticBrake());
  }

  /**
   * @brief set the speed of the lower motor
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  public void setLowerSpeed(double speed) {
    m_lowerOutput.Velocity = speed;
    m_lowerMotor.setControl(m_lowerOutput);
    m_lowerMotor.setNeutralMode(NeutralModeValue.Brake);
    if (speed == 0.0)
      m_lowerMotor.setControl(new StaticBrake());
  }

  /**
   * @brief set the speed of the flywheel
   * 
   * @param speed speed in revolutions per second
   * @return Command
   */
  public Command setSpeed(double speed) {
    return this.runOnce(() -> {
      this.setUpperSpeed(speed);
      this.setLowerSpeed(speed);
    });
  }

  /**
   * @brief Spin the intake motors to outake notes
   * 
   * @return Command
   */
  public Command reverse() {
    return this.runOnce(() -> {
      this.setUpperSpeed(-kUpperSpeed);
      this.setLowerSpeed(-kLowerSpeed);
    });
  }

  /**
   * @brief Spin up the flywheel motors
   * 
   * @return Command
   */
  public Command forwards() {
    return this.runOnce(() -> {
      this.setUpperSpeed(kUpperSpeed);
      this.setLowerSpeed(kLowerSpeed);
    });
  }

  /**
   * @brief Stop the flywheel motors
   * 
   * @return Command
   */
  public Command stop() {
    return this.setSpeed(0);
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
  public Command upperSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_upperSysIdRoutine.quasistatic(direction);
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
  public Command upperSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_upperSysIdRoutine.dynamic(direction);
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
  public Command lowerSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_lowerSysIdRoutine.quasistatic(direction);
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
  public Command lowerSysIdDynamic(SysIdRoutine.Direction direction) {
    return m_lowerSysIdRoutine.dynamic(direction);
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
      m_upperMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      m_lowerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
      // set motor sim input voltage
      m_upperMotorSim.setInputVoltage(m_upperMotorSimState.getMotorVoltage());
      m_lowerMotorSim.setInputVoltage(m_lowerMotorSimState.getMotorVoltage());
      // update motor sim
      m_upperMotorSim.update(kSimLoopPeriod);
      m_lowerMotorSim.update(kSimLoopPeriod);
      // update motor sim state
      m_upperMotorSimState.setRawRotorPosition(m_upperMotorSim.getAngularPositionRotations());
      m_upperMotorSimState.setRotorVelocity(m_upperMotorSim.getAngularVelocityRPM() / 60.0);
      m_lowerMotorSimState.setRawRotorPosition(m_lowerMotorSim.getAngularPositionRotations());
      m_lowerMotorSimState.setRotorVelocity(m_lowerMotorSim.getAngularVelocityRPM() / 60.0);
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
    builder.addDoubleProperty("Upper Target Velocity", () -> m_upperOutput.Velocity,
        (double target) -> this.setUpperSpeed(target));
    // add upper motor measured velocity property
    builder.addDoubleProperty("Upper Measured Velocity",
        () -> m_upperMotor.getVelocity().getValueAsDouble(), null);
    // add lower motor target velocity property
    builder.addDoubleProperty("Lower Target Velocity", () -> m_lowerOutput.Velocity,
        (double target) -> this.setLowerSpeed(target));
    // add lower motor measured velocity property
    builder.addDoubleProperty("Lower Measured Velocity",
        () -> m_lowerMotor.getVelocity().getValueAsDouble(), null);
  }
}
