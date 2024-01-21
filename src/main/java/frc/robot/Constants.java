package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {

  // intake
  public static final int kUpperIntakeMotorId = 1;
  public static final int kLowerIntakeMotorId = 2;
  public static final InvertedValue kUpperIntakeInverted = InvertedValue.Clockwise_Positive;
  public static final InvertedValue kLowerIntakeInverted = InvertedValue.CounterClockwise_Positive;
  public static final String kUpperIntakeBusName = "";
  public static final String kLowerIntakeBusName = "";
  public static final double kUpperIntakeSpeed = 1.00; // rotations per second
  public static final double kLowerIntakeSpeed = 1.00; // rotations per second
  public static final Slot0Configs kUpperIntakeControllerConstats =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
  public static final Slot0Configs kLowerIntakeControllerConstats =
      new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);


}
