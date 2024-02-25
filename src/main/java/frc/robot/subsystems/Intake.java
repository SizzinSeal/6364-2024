package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.math.system.plant.DCMotor;
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
  // sysid routines

  /**
   * @brief IntakeSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so
   *        that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix
   *        Tuner.
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
   *        This is where we actually set the motor speeds. We do this in a
   *        seperate method to
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
   *        This method is called periodically by the scheduler. We use it to
   *        update the simulated
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
    builder.addDoubleProperty("Deployer Position", () -> m_deployerMotor.getPosition().getValueAsDouble(), null);
  }
}
