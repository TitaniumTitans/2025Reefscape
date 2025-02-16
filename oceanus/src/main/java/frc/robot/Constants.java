package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static Mode getMode() {
    if (RobotBase.isReal()) {
      return Mode.REAL;
    } else {
      return Mode.SIM;
    }
  }
}
