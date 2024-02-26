package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Flywheel {
  // sim loop period (seconds)
  public static final double kSimLoopPeriod = 0.005;
  // motor ids
  public static final int kUpperMotorId = 14;
  public static final int kLowerMotorId = 15;
  // motor CAN bus names
  public static final String kUpperMotorBus = "rio";
  public static final String kLowerMotorBus = "rio";
  // motor inversion TODO: find these values
  public static final InvertedValue kUpperMotorInverted = InvertedValue.Clockwise_Positive;
  public static final InvertedValue kLowerMotorInverted = InvertedValue.CounterClockwise_Positive;
  // motor neutral modes
  public static final NeutralModeValue kUpperNeutralMode = NeutralModeValue.Coast;
  public static final NeutralModeValue kLowerNeutralMode = NeutralModeValue.Coast;
  // controller constants
  public static final Slot0Configs kUpperControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  public static final Slot0Configs kLowerControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  // speeds (in rotations per second)
  public static final double kUpperSpeed = 0; // TODO: find this value
  public static final double kLowerSpeed = 0; // TODO: find this value
  // acceleration (in rotations per second squared)
  public static final double kUpperAccel = 0; // TODO: find this value
  public static final double kLowerAccel = 0; // TODO: find this value
  // tolerances (in rotations)
  public static final double kUpperTolerance = 5; // TODO: find this value
  public static final double kLowerTolerance = 5; // TODO: find this value
  // ratios (driven/driver)
  public static final double kUpperRatio = 1;
  public static final double kLowerRatio = 1;
}
