package frc.robot;

public class Constants {
  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public static Mode getMode() {
    if (Robot.isReal()) {
      return Mode.REAL;
    } else {
      return Mode.REPLAY;
    }
  }
}
