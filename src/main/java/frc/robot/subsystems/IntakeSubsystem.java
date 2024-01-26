package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.util.sendable.SendableBuilder;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import static frc.robot.Constants.Intake.*;


/**
 * @brief Intake Subsystem
 * 
 */
public class IntakeSubsystem extends SubsystemBase {
  // init motors
  private final TalonFX m_upperMotor = new TalonFX(kUpperMotorId, kUpperBusName);
  private final TalonFX m_lowerMotor = new TalonFX(kLowerMotorId, kLowerBusName);
  private final VelocityDutyCycle m_upperMotorVelocity = new VelocityDutyCycle(0);
  private final VelocityDutyCycle m_lowerMotorVelocity = new VelocityDutyCycle(0);

  /**
   * @brief IntakeSubsystem constructor
   * 
   *        This is where the motors are configured. We configure them here so that we can swap
   *        motors without having to worry about reconfiguring them in Phoenix Tuner.
   */
  public IntakeSubsystem() {
    super();
    // configure motors
    TalonFXConfiguration upperConfig = new TalonFXConfiguration();
    TalonFXConfiguration lowerConfig = new TalonFXConfiguration();
    // set controller gains
    upperConfig.Slot0 = kUpperControllerConstants;
    lowerConfig.Slot0 = kLowerControllerConstants;
    // invert motors
    upperConfig.MotorOutput.Inverted = kUpperInverted;
    lowerConfig.MotorOutput.Inverted = kLowerInverted;
    // set ClosedLoopOutput type (either Velocity or TorqueCurrentFOC)

    // apply configuration
    m_upperMotor.getConfigurator().apply((upperConfig));
    m_lowerMotor.getConfigurator().apply((lowerConfig));
  }

  /**
   * @brief Update motor speeds
   * 
   *        This is where we actually set the motor speeds. We do this in a seperate method to
   *        simplify the commands that change the target velocity.
   */
  private void updateMotorSpeeds() {
    m_upperMotor.setControl(m_upperMotorVelocity);
    m_lowerMotor.setControl(m_lowerMotorVelocity);
  }

  /**
   * @brief Spin the intake motors to outake notes
   * 
   * @return Command
   */
  public Command outake() {
    return this.runOnce(() -> {
      m_upperMotorVelocity.Velocity = -kUpperSpeed;
      m_lowerMotorVelocity.Velocity = -kLowerSpeed;
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
      m_upperMotorVelocity.Velocity = kUpperSpeed;
      m_lowerMotorVelocity.Velocity = kLowerSpeed;
      this.updateMotorSpeeds();
    });
  }

  /**
   * @brief Spin the intake motors at a specific velocity
   * 
   * @param speed the speed of the motors, revolutions per second
   * 
   * @return Command
   */
  public Command intake(double speed) {
    return this.runOnce(() -> {
      m_upperMotorVelocity.Velocity = speed;
      m_lowerMotorVelocity.Velocity = speed;
      this.updateMotorSpeeds();
    });

  }

  /**
   * @brief Spin the intake motors at specific velocities
   * 
   * @param upperSpeed the speed of the upper motor, revolutions per second
   * @param lowerSpeed the speed of the lower motor, revolutions per second
   * 
   * @return Command
   */
  public Command intake(double upperSpeed, double lowerSpeed) {
    return this.runOnce(() -> {
      m_upperMotorVelocity.Velocity = upperSpeed;
      m_lowerMotorVelocity.Velocity = lowerSpeed;
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
      m_upperMotorVelocity.Velocity = 0;
      m_lowerMotorVelocity.Velocity = 0;
      this.updateMotorSpeeds();
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
    // add upper motor target velocity property
    builder.addDoubleProperty("Upper Target Velocity", () -> m_upperMotorVelocity.Velocity,
        (double target) -> {
          m_upperMotorVelocity.Velocity = target;
          this.updateMotorSpeeds();
        });
    // add upper motor measured velocity property
    builder.addDoubleProperty("Upper Measured Velocity",
        () -> m_upperMotor.getVelocity().getValueAsDouble(), null);
    // add lower motor target velocity property
    builder.addDoubleProperty("Lower Target Velocity", () -> m_lowerMotorVelocity.Velocity,
        (double target) -> {
          m_lowerMotorVelocity.Velocity = target;
          this.updateMotorSpeeds();
        });
    // add lower motor measured velocity property
    builder.addDoubleProperty("Lower Measured Velocity",
        () -> m_lowerMotor.getVelocity().getValueAsDouble(), null);
  }
}
