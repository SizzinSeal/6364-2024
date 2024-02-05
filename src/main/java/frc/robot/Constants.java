package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;

public class Constants {
  public class Intake {
    // motor ids
    public static final int kUpperMotorId = 14;
    public static final int kLowerMotorId = 15;
    // motor CAN bus names
    public static final String kUpperBusName = "";
    public static final String kLowerBusName = "";
    // motor inversion
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kLowerControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kUpperSpeed = .8 * 12; // rotations per second
    public static final double kLowerSpeed = .8 * 12; // rotations per second
  }
  public class Flywheel {
    // motor ids
    public static final int kUpperMotorId = 16;
    public static final int kLowerMotorId = 17;
    // motor CAN bus names
    public static final String kUpperBusName = "";
    public static final String kLowerBusName = "";
    // motor inversion
    public static final InvertedValue kUpperInverted = InvertedValue.Clockwise_Positive;
    public static final InvertedValue kLowerInverted = InvertedValue.CounterClockwise_Positive;
    // controller constants
    public static final Slot0Configs kUpperControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    public static final Slot0Configs kLowerControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kUpperSpeed = .9 * 12; // rotations per second
    public static final double kLowerSpeed = .9 * 12; // rotations per second
    // tolerances
    public static final double kUpperTolerance = 5; // revolutions per second
    public static final double kLowerTolerance = 5; // revolutions per second
  }
  public class Indexer {
    // ids
    public static final int kMotorId = 18;
    public static final int kNoteDetectorPort = 0;
    // motor CAN bus names
    public static final String kMotorBusName = "";
    // motor inversion
    public static final InvertedValue kInverted = InvertedValue.Clockwise_Positive;
    // controller constants
    public static final Slot0Configs kMotorControllerConstants =
        new Slot0Configs().withKP(0.01).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);
    // speeds
    public static final double kSpeed = .9 * 12; // rotations per second
  }


  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

}
