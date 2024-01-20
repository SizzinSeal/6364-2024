package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX m_upperIntakeMotor = new TalonFX(LOWER_INTAKE_MOTOR_ID, CANBUS_NAME);
  private final TalonFX m_lowerIntakeMotor = new TalonFX(UPPER_INTAKE_MOTOR_ID, CANBUS_NAME);


  public IntakeSubsystem() {
    super();
  }


  public Command runIntake() {
    return this.runOnce(() -> m_upperIntakeMotor.set(1.00));
  }
}
