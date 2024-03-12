package frc.robot.commands;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.LimelightHandler;

public class MoveToNote extends PIDCommand {

  public MoveToNote(PIDController controller, DoubleSupplier measurementSource, double setpoint,
      DoubleConsumer useOutput, Subsystem[] requirements) {
    super(
        new PIDController(Constants.Drivetrain.kP, Constants.Drivetrain.kI,
            Constants.Drivetrain.kD),
        () -> LimelightHandler.getLatestLatencyAdjustedTimestamp(), setpoint, useOutput,
        requirements);
  }
}
