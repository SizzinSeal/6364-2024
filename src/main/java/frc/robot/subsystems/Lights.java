package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.led.CANdle;

public class Lights extends SubsystemBase {

  public CANdle lights;

  /**
   * Constructor.
   */
  public Lights() {
    lights = new CANdle(Constants.Lights.kDeviceId, "rio");
    lights.configBrightnessScalar(1);
    lights.setLEDs(0, 0, 0);
  }

  /**
   * Turn off lights.
   * 
   * @return Run once Command.
   */
  public Command turnOffLights() {
    return this.runOnce(() -> {
      lights.setLEDs(0, 0, 0);
    });
  }

  /**
   * Turn on lights. Constant -> No animation.
   * 
   * @return Run once Command.
   */
  public Command turnOnLights() {
    return this.runOnce(() -> {
      int R = Constants.Lights.kRGBvalues[0];
      int G = Constants.Lights.kRGBvalues[1];
      int B = Constants.Lights.kRGBvalues[2];
      lights.setLEDs(R, G, B);
    });
  }

  /**
   * Run animation set in Constants.
   * 
   * @return Run once Command.
   */
  public Command strobe() {
    return Commands.repeatingSequence(turnOnLights(), Commands.waitSeconds(0.3), turnOffLights());
  }

  /**
   * Disable animation.
   * 
   * @return Run once Command.
   */
  public Command disableAnimation() {
    return this.runOnce(() -> {
      lights.clearAnimation(0);
    });
  }
}
