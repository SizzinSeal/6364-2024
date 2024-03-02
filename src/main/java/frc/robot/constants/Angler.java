package frc.robot.constants;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.units.Time;

public class Angler {
  // motor ids
  public static final int kMotorId = 17;
  // motor CAN bus names
  public static final String kMotorBus = "rio";
  // sensor ports
  public static final int kLimitPort = 5; // TODO: find this value
  // motor inversion TODO: find these values
  public static final InvertedValue kMotorInverted = InvertedValue.Clockwise_Positive;
  // controller constants
  public static final Slot0Configs kControllerGains = // TODO: tune this
      new Slot0Configs().withKP(1).withKI(0).withKD(0).withKS(0.2539).withKV(0.11015)
          .withKA(0.0010909);
  // positions (in rotations)
  public static final double kMaxPosition = 0; // TODO: find this value
  public static final double kMinPosition = 0;
  public static final double kShootingPosition = 130; // TODO: find this value
  public static final double kLoadingPosition = 20; // TODO: find this value
  // speeds (in rotations per second)
  public static final double kMaxSpeed = 120; // TODO: find this value
  public static final double kManualSpeed = 30; // TODO: find this value
  public static final double kProbeSpeed = 30; // TODO: find this value
  // acceleration (in rotations per second squared)
  public static final double kAcceleration = 200; // TODO: find this value
  public static final double kManualAcceleration = 0; // TODO: find this value
  public static final double kProbeAcceleration = 0; // TODO: find this value
  // jerk (in rotations per second cubed)
  public static final double kJerk = 800;
  // tolerances (in rotations)
  public static final double kTolerance = 0.013889; // 5 degrees
  // ratios (driven/driver)
  public static final double kRatio = 1; // in reality this is a 1500 to 1 gear ratio, but
  // for some reason TalonFX doesn't support gear ratios higher than 1000 to 1
  // sysid constants
  public static final Measure<Velocity<Voltage>> kRampRate = Volts.of(0.2).per(Second);
  public static final Measure<Voltage> kStepVoltage = Volts.of(8.0);
  public static final Measure<Time> kTimeout = Seconds.of(20.0);
}
