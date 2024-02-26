package frc.robot.constants;

import com.ctre.phoenix6.signals.InvertedValue;

public class Intake {
  // sim loop period (seconds)
  public static final double kSimLoopPeriod = 0.005;
  // motor ids
  public static final int kMotorId = 20;
  // CAN bus name
  public static final String kMotorBus = "rio";
  // motor inversion TODO: find these values
  public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
  // speeds (voltage)
  public static final double kIntakeSpeed = 8;
  public static final double kOuttakeSpeed = 8;
  // ratios (driven/driver)
  public static final double kRatio = 1;
}
