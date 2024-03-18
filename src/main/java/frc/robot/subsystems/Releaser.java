package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Releaser.*;

public class Releaser extends SubsystemBase {
  // init servo
  private final Servo m_servo = new Servo(kServoPort);

  /*
   * @brief Releaser constructor
   */
  public Releaser() {
    super();
    m_servo.set(kStableAngle);
  }

  /**
   * @brief release the releaser
   * 
   * @return Command
   */
  public Command release() {
    return this.runOnce(() -> m_servo.set(kReleaseAngle));
  }

  /**
   * @bried ready the releaser
   * 
   * @return Command
   */
  public Command ready() {
    return this.runOnce(() -> m_servo.set(kStableAngle));
  }
}
