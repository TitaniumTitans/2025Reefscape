package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.MaybeFlippedPose2d;

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

  public class AlignmentGoals {
    public static MaybeFlippedPose2d A = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d B = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d C = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d D = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d E = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d F = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d G = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d H = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d I = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d J = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d K = new MaybeFlippedPose2d(new Pose2d());
    public static MaybeFlippedPose2d L = new MaybeFlippedPose2d(new Pose2d());
  }
}
