package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {
  public class Intake {
    // motor ids
    public static final int kUpperMotorId = 15;
    public static final int kLowerMotorId = 16;
    // motor CAN bus names
    public static final String kUpperBusName = "";
    public static final String kLowerBusName = "";
    // motor inversion
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    public static final Slot0Configs kLowerControllerConstants =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.2).withKS(0).withKV(1.5).withKA(0);
    // speeds
    public static final double kUpperSpeed = 1.00; // rotations per second
    public static final double kLowerSpeed = 1.00; // rotations per second
  }
}
