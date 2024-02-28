package frc.robot.constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

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
  public static final InvertedValue kUpperMotorInverted = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue kLowerMotorInverted = InvertedValue.CounterClockwise_Positive;
  // motor neutral modes
  public static final NeutralModeValue kUpperNeutralMode = NeutralModeValue.Brake;
  public static final NeutralModeValue kLowerNeutralMode = NeutralModeValue.Brake;
  // controller constants
  public static final Slot0Configs kUpperControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0.22748).withKV(0.11942)
          .withKA(0.015106);
  public static final Slot0Configs kLowerControllerGains = // TODO: tune this
      new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0.17641).withKV(0.10863)
          .withKA(0.018805);
  // speeds (in rotations per second)
  public static final double kUpperSpeed = 100; // TODO: find this value
  public static final double kLowerSpeed = 100; // TODO: find this value
  // acceleration (in rotations per second squared)
  public static final double kUpperAccel = 0; // TODO: find this value
  public static final double kLowerAccel = 0; // TODO: find this value
  // tolerances (in rotations)
  public static final double kUpperTolerance = 5; // TODO: find this value
  public static final double kLowerTolerance = 5; // TODO: find this value
  // ratios (driven/driver)
  public static final double kUpperRatio = 1;
  public static final double kLowerRatio = 1;
  // sysid constants
  public static final Measure<Velocity<Voltage>> kRampRate = Volts.of(0.2).per(Second);
  public static final Measure<Voltage> kStepVoltage = Volts.of(7.0);
  public static final Measure<Time> kTimeout = Seconds.of(15.0);
}
