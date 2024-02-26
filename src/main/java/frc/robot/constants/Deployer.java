package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;

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
  public static final Measure<Velocity<Voltage>> kRampRate = Volts.of(5.0).per(Second);
  public static final Measure<Voltage> kStepVoltage = Volts.of(2.0);
  public static final Measure<Time> kTimeout = Seconds.of(5.0);
  public static final double kG = 0; // TODO: find this value
}
