package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.util.sendable.SendableBuilder;
import static frc.robot.Constants.*;



public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_upperIntakeMotor = new TalonFX(LOWER_INTAKE_MOTOR_ID, CANBUS_NAME);
  private final TalonFX m_lowerIntakeMotor = new TalonFX(UPPER_INTAKE_MOTOR_ID, CANBUS_NAME);


  public IntakeSubsystem() {
    super();
  }


  public Command runIntake() {
    return this.runOnce(() -> {
      m_upperIntakeMotor.set(1.00);
    });
  }

  public Command stopIntake() {
    return this.runOnce(() -> {
      m_upperIntakeMotor.set(0.00);
    });
  }


  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("speed", () -> m_upperIntakeMotor.getVelocity().getValueAsDouble(),
        null);
  }
}
