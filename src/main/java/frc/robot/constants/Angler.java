package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Angler {
  // motor ids
  public static final int kMotorId = 17;
  // motor CAN bus names
  public static final String kMotorBus = "rio";
  // sensor ports
  public static final int kLimitPort = 0; // TODO: find this value
  // motor inversion TODO: find these values
  public static final InvertedValue kMotorInverted = InvertedValue.Clockwise_Positive;
  // controller constants
  public static final Slot0Configs kControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  // positions (in rotations)
  public static final double kMaxPosition = 0; // TODO: find this value
  public static final double kMinPosition = 0;
  public static final double kShootingPosition = 0; // TODO: find this value
  public static final double kLoadingPosition = 0; // TODO: find this value
  // speeds (in rotations per second)
  public static final double kMaxSpeed = 0; // TODO: find this value
  public static final double kManualSpeed = 0; // TODO: find this value
  public static final double kProbeSpeed = 0; // TODO: find this value
  // acceleration (in rotations per second squared)
  public static final double kAcceleration = 0; // TODO: find this value
  public static final double kManualAcceleration = 0; // TODO: find this value
  public static final double kProbeAcceleration = 0; // TODO: find this value
  // tolerances (in rotations)
  public static final double kTolerance = 5; // TODO: find this value
  // ratios (driven/driver)
  public static final double kRatio = 1; // in reality this is a 1500 to 1 gear ratio, but
  // for some reason TalonFX doesn't support gear ratios higher than 1000 to 1
}
