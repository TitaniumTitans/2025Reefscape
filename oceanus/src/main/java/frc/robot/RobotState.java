package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.FieldRelativeSpeeds;
import lombok.Getter;
import lombok.Setter;
import org.dyn4j.geometry.Polygon;
import org.dyn4j.geometry.Vector2;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.utils.mathutils.GeometryConvertor;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;

public class RobotState {

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public enum CoralLevel {
    L2,
    L3,
    L4
  }

  @Getter
  @Setter
  @AutoLogOutput(key = "RobotState/Coral Height")
  private CoralLevel coralLevel = CoralLevel.L2;

  public static final Polygon farKeepoutZoneBlue;
  public static final Polygon farKeepoutZoneRed;
  public static final Polygon closeKeepoutZoneBlue;
  public static final Polygon closeKeepoutZoneRed;
  public static final Rectangle2d keepoutZoneBarge;
  static {
    Translation2d[] farSafezonePoints = new Translation2d[6];
    Translation2d[] closeSafezonePoints = new Translation2d[6];
    Rotation2d angle = Rotation2d.fromDegrees(30);

    for (int i = 0; i < 6; i++) {
      farSafezonePoints[i] = FieldConstants.Reef.center.plus(
          new Translation2d(Units.inchesToMeters(80), angle)
      );

      closeSafezonePoints[i] = FieldConstants.Reef.center.plus(
          new Translation2d(Units.inchesToMeters(65), angle)
      );

      angle = angle.plus(Rotation2d.fromDegrees(60));
    }

    Vector2[] farBlueVecs = Arrays.stream(farSafezonePoints).map(GeometryConvertor::toDyn4jVector2).toArray(Vector2[]::new);
    Vector2[] farRedVecs = Arrays.stream(farSafezonePoints).map(AllianceFlipUtil::apply).map(GeometryConvertor::toDyn4jVector2).toArray(Vector2[]::new);
    Vector2[] closeBlueVecs = Arrays.stream(closeSafezonePoints).map(GeometryConvertor::toDyn4jVector2).toArray(Vector2[]::new);
    Vector2[] closeRedVecs = Arrays.stream(closeSafezonePoints).map(AllianceFlipUtil::apply).map(GeometryConvertor::toDyn4jVector2).toArray(Vector2[]::new);

    Logger.recordOutput("Far Safezone", farSafezonePoints);
    Logger.recordOutput("close Safezone", closeSafezonePoints);

    farKeepoutZoneBlue = new Polygon(farBlueVecs);
    farKeepoutZoneRed = new Polygon(farRedVecs);
    closeKeepoutZoneBlue = new Polygon(closeBlueVecs);
    closeKeepoutZoneRed = new Polygon(closeRedVecs);

    // barge is 3ft, 8in (112 cm) structure
    keepoutZoneBarge = new Rectangle2d(
        new Pose2d(FieldConstants.fieldLength / 2.0, FieldConstants.fieldWidth / 2.0, new Rotation2d()),
        Centimeters.of(112).plus(Inches.of(45.0)),
        Centimeters.of(889.0)
    );

    Logger.recordOutput("Barge Safezone", keepoutZoneBarge);
  }

  // Pose estimation
  @Getter
  @AutoLogOutput(key = "RobotState/OdometryPose")
  private Pose2d odometryPose = new Pose2d();

  private Rotation2d lastRawGyro = new Rotation2d();

  @Setter
  @Getter
  private FieldRelativeSpeeds lastFieldRelativeSpeeds = new FieldRelativeSpeeds();

  @Setter
  @Getter
  private boolean slowSpeed = false;

  // use for simulation
  @Setter
  private Optional<SwerveDriveSimulation> driveSimulation = Optional.empty();

  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS),
          new Rotation2d(),
          new SwerveModulePosition[]{
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition(),
              new SwerveModulePosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.01, 0.01, 0.02),
          VecBuilder.fill(0.08, 0.08, 0.03)
      );

  // used to filter vision measurements into odometry estimation
  // Odometry
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  private RobotState() {
    AutoLogOutputManager.addObject(this);
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(lastRawGyro, lastWheelPositions, pose);

    driveSimulation.ifPresent(swerveDriveSimulation -> swerveDriveSimulation.setSimulationWorldPose(pose));
  }

  public void addOdometryMeasurement(Rotation2d heading, SwerveModulePosition[] modulePositions, double timestamp) {
    lastRawGyro = heading;
    lastWheelPositions = modulePositions;
    poseEstimator.updateWithTime(timestamp, heading, modulePositions);
  }

  public void addVisionMeasurement(VisionObservation update) {
    poseEstimator.addVisionMeasurement(update.visionPose, update.timestamp, update.stdDevs);
  }


  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    if (driveSimulation.isPresent()) {
      return driveSimulation.get().getSimulatedDriveTrainPose();
    }
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  @AutoLogOutput(key = "RobotState/In Far Reef Zone")
  public boolean inFarReefZone() {
    return farKeepoutZoneBlue.contains(GeometryConvertor.toDyn4jTransform(getEstimatedPose()).getTranslation())
        || farKeepoutZoneRed.contains(GeometryConvertor.toDyn4jTransform(getEstimatedPose()).getTranslation());
  }

  @AutoLogOutput(key = "RobotState/In Close Reef Zone")
  public boolean inCloseReefZone() {
    return closeKeepoutZoneBlue.contains(GeometryConvertor.toDyn4jTransform(getEstimatedPose()).getTranslation())
        || closeKeepoutZoneRed.contains(GeometryConvertor.toDyn4jTransform(getEstimatedPose()).getTranslation());
  }

  @AutoLogOutput(key = "RobotState/In Barge Zone")
  public boolean inBargeZone() {
    return keepoutZoneBarge.contains(getEstimatedPose().getTranslation());
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
