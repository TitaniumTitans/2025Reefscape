package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.Setter;
import lombok.ToString;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;
import java.util.Optional;

public class RobotState {
  private static final double POSE_BUFFER_SIZE_SEC = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.0055, 0.0055, 0.002));

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose estimation
  private SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS),
      new Rotation2d(),
      new SwerveModulePosition[]{
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      }
  );

  private Rotation2d lastRawGyro = new Rotation2d();

  // use for simulation
  @Setter
  private Optional<SwerveDriveSimulation> driveSimulation = Optional.empty();

  private Field2d poseField = new Field2d();

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
          VecBuilder.fill(0.05, 0.05, 0.03) // 0.1, 0.1, 0.03
      );

  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    AutoLogOutputManager.addObject(this);
    SmartDashboard.putData("Robot Pose", poseField);
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(lastRawGyro, lastWheelPositions, pose);
    odometry.resetPose(pose);

    driveSimulation.ifPresent(swerveDriveSimulation -> swerveDriveSimulation.setSimulationWorldPose(pose));
  }

  public void addOdometryMeasurement(Rotation2d heading, SwerveModulePosition[] modulePositions, double timestamp) {
    lastRawGyro = heading;
    lastWheelPositions = modulePositions;
    poseEstimator.updateWithTime(timestamp, heading, modulePositions);
    odometry.update(heading, modulePositions);
  }

  public void addVisionMeasurement(VisionObservation update) {
    poseEstimator.addVisionMeasurement(update.visionPose, update.timestamp, update.stdDevs);
  }


  @AutoLogOutput(key = "RobotState/EstimatedPose")
  public Pose2d getEstimatedPose() {
    poseField.setRobotPose(poseEstimator.getEstimatedPosition());
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  @AutoLogOutput(key = "RobotState/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometry.getPoseMeters();
  }

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
