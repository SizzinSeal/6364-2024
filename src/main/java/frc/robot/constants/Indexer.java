package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Indexer {
  // simulation loop period (seconds)
  public static final double kSimLoopPeriod = 0.005; // 5 ms
  // motor ids
  public static final int kMotorId = 19;
  // sensor ids
  public static final int kNoteDetectorPort = 0; // TODO: find this value
  // motor CAN bus names
  public static final String kMotorBus = "rio";
  // motor inversion
  public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive; // TODO: find
                                                                                  // this value
  // controller gains
  public static final Slot0Configs kMotorControllerConstants = // TODO: tune this
      new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
  // speeds (rotations per second)
  public static final double kEjectSpeed = 0; // TODO: find this value
  public static final double kLoadSpeed = 0; // TODO: find this value
  // ratios
  public static final double kRatio = 1;
}
