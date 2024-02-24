package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import static frc.robot.Constants.Angler.*;

public class Angler extends SubsystemBase {

  private final TalonFX m_motor = new TalonFX(99); // Replace 99 with the actual CAN ID

  public Angler() {
    super();
    TalonFXConfiguration config = new TalonFXConfiguration();
    // config.Slot0 = kAnglerControllerConstants;

    m_motor.getConfigurator().apply((config));
    m_motor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Command setAngle(double angle) {
    return this.runOnce(() -> {
      // Set the angle of the angler
    });
  }
}
