package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import org.littletonrobotics.junction.Logger;

public class IntakeVisualizer {
  private final double xOffsetIntake = Units.inchesToMeters(10.75);
  private final double yOffsetIntake = Units.inchesToMeters(9.75);
  private final double xOffsetCoral = Units.inchesToMeters(-7.5);
  private final double yOffsetCoral = Units.inchesToMeters(18.8);

  private final String key;

  public IntakeVisualizer(String key) {
    this.key = key;
  }

  public void updateVisualization(double intakeAngle, double coralAngle) {
    Pose3d intakePose = new Pose3d(
        new Translation3d(
            xOffsetIntake,
            0.3302,
            yOffsetIntake
        ),
        new Rotation3d(0.0, Units.degreesToRadians(45 - intakeAngle), Units.degreesToRadians(180.0))
    ).transformBy(
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(2.3),
                0.0,
                Units.inchesToMeters(9.75)
            ).unaryMinus(),
            new Rotation3d()
        ));

    Pose3d coralPose = new Pose3d(
        new Translation3d(
            xOffsetCoral,
            0.3302,
            yOffsetCoral
        ),
        new Rotation3d(0.0, Units.degreesToRadians(coralAngle - 80), Units.degreesToRadians(180.0))
    ).transformBy(
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(20.5),
                0.0,
                Units.inchesToMeters(18.8)
            ).unaryMinus(),
            new Rotation3d()
        ));

    Logger.recordOutput(key + "3D Pose",
        intakePose,
        coralPose);
  }
}
