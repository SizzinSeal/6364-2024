package frc.robot;

import edu.wpi.first.hal.DIOJNI;

public class PhotoelectricSensor {
  public static boolean isInRange() {
    if (DIOJNI.getDIO(4)) {
      return true;
    } else {
      return false;
    }
  }
}
