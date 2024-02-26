package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Deployer {
  // sim loop period (milliseconds)
  public static final double kSimLoopPeriod = 0.005;
  // motor ids
  public static final int kMotorId = 18;
  // CAN bus name
  public static final String kMotorBus = "rio";
  // motor inversion TODO: find these values
  public static final InvertedValue kMotorInverted = InvertedValue.CounterClockwise_Positive;
  // controller constants
  public static final Slot0Configs kControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  // positions (rotations)
  public static final double kUpPosition = 0;
  public static final double kDownPosition = 0.2; // TODO: find this value
  // speeds (rotations per second)
  public static final double kMaxSpeed = 0; // TODO: find this value
  // acceleration (rotations per second squared)
  public static final double kMaxAccel = 0; // TODO: find this value
  // ratios (driven/driver)
  public static final double kRatio = 90;
  // SysId constants
  public static final double kRampRate = 0.5; // volts per second
  public static final double kStepVoltage = 4; // volts
  public static final double kTimeout = 2; // seconds
}
