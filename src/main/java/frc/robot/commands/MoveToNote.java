package frc.robot.commands;

import frc.robot.LimelightHandler;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class MoveToNote extends MoveToPose {

  public MoveToNote(CommandSwerveDrivetrain drivetrain) {
    super(LimelightHandler.getNoteFieldPose2DEstimate(drivetrain), drivetrain);
  }

}
