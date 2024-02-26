package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Intake {
  // sim loop period (seconds)
  public static final double kSimLoopPeriod = 0.005;
  // motor ids
  public static final int kMotorId = 20;
  // CAN bus name
  public static final String kMotorBus = "rio";
  // motor inversion TODO: find these values
  public static final InvertedValue kMotorInverted = InvertedValue.Clockwise_Positive;
  // controller constants
  public static final Slot0Configs kControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  // speeds (rotations per second)
  public static final double kIntakeSpeed = 0; // TODO: find this value
  public static final double kOuttakeSpeed = 0; // TODO: find this value
  // ratios (driven/driver)
  public static final double kRatio = 1;
  // SysId constants
  public static final double kRampRate = 0.5; // volts per second
  public static final double kStepVoltage = 4; // volts
  public static final double kTimeout = 10; // seconds
}
