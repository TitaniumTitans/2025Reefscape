package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import lombok.EqualsAndHashCode;
import lombok.Getter;
import lombok.ToString;
import org.littletonrobotics.junction.AutoLogOutput;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.NoSuchElementException;
import java.util.Optional;

public class RobotState {
  private static final double POSE_BUFFER_SIZE_SEC = 2.0;
  private static final Matrix<N3, N1> odometryStateStdDevs =
      new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002));

  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  // Pose estimation
  @Getter
  @AutoLogOutput(key = "RobotState/OdometryPose")
  private Pose2d odometryPose = new Pose2d();

  @Getter
  @AutoLogOutput(key = "RobotState/EstimatedPose")
  private Pose2d estimatedPose = new Pose2d();

  // used to filter vision measurements into odometry estimation
  private final TimeInterpolatableBuffer<Pose2d> poseBuffer =
      TimeInterpolatableBuffer.createBuffer(POSE_BUFFER_SIZE_SEC);
  private final Matrix<N3, N1> qStdDevs = new Matrix<>(Nat.N3(), Nat.N1());
  // Odometry
  private final SwerveDriveKinematics kinematics;
  private SwerveModulePosition[] lastWheelPositions =
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };

  // Assume gyro starts at zero
  private Rotation2d gyroOffset = new Rotation2d();

  private RobotState() {
    for (int i = 0; i < 3; ++i) {
      qStdDevs.set(i, 0, Math.pow(odometryStateStdDevs.get(i, 0), 2));
    }
    kinematics = new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);
    AutoLogOutputManager.addObject(this);
    Logger.recordOutput("Robot State Created", true);
  }

  public void resetPose(Pose2d pose) {
    estimatedPose = pose;
    odometryPose = pose;
    gyroOffset = pose.getRotation().minus(gyroOffset);
    poseBuffer.clear();
  }

  public void addOdometryMeasurement(OdometryObservation update) {
    Twist2d twist = kinematics.toTwist2d(lastWheelPositions, update.wheelPositions());
    lastWheelPositions = update.wheelPositions();
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = odometryPose.exp(twist);
    // use gyro if connected
    update.gyroAngle.ifPresent(
        gyroAngle -> {
          Rotation2d angle = gyroAngle.plus(gyroOffset);
          odometryPose = new Pose2d(odometryPose.getTranslation(), angle);
        }
    );
    // add pose to buffer at timestamp
    poseBuffer.addSample(update.timestamp(), odometryPose);
    // calculate the final difference between the previous and current odometry pose
    Twist2d finalTwist = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(finalTwist);
  }

  public void addVisionMeasurement(VisionObservation update) {
    // If measurement is old enough to be outside the pose buffer's timespan, skip.
    try {
      if (poseBuffer.getInternalBuffer().lastKey() - POSE_BUFFER_SIZE_SEC > update.timestamp()) {
        return;
      }
    } catch (NoSuchElementException ex) {
      return;
    }
    // Get odometry based pose at timestamp
    var sample = poseBuffer.getSample(update.timestamp());
    if (sample.isEmpty()) {
      // exit if not there
      return;
    }

    // sample --> odometryPose transform and backwards of that
    var sampleToOdometryTransform = new Transform2d(sample.get(), odometryPose);
    var odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
    // get old estimate by applying odometryToSample Transform
    Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);

    // Calculate 3 x 3 vision matrix
    var r = new double[3];
    for (int i = 0; i < 3; ++i) {
      r[i] = update.stdDevs().get(i, 0) * update.stdDevs().get(i, 0);
    }
    // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
    // and C = I. See wpimath/algorithms.md.
    Matrix<N3, N3> visionK = new Matrix<>(Nat.N3(), Nat.N3());
    for (int row = 0; row < 3; ++row) {
      double stdDev = qStdDevs.get(row, 0);
      if (stdDev == 0.0) {
        visionK.set(row, row, 0.0);
      } else {
        visionK.set(row, row, stdDev / (stdDev + Math.sqrt(stdDev * r[row])));
      }
    }
    // difference between estimate and vision pose
    Transform2d transform = new Transform2d(estimateAtTime, update.visionPose());
    // scale transform by visionK
    var kTimesTransform =
        visionK.times(
            VecBuilder.fill(
                transform.getX(), transform.getY(), transform.getRotation().getRadians()));
    Transform2d scaledTransform =
        new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));

    // Recalculate current estimate by applying scaled transform to old estimate
    // then replaying odometry data
    estimatedPose = estimateAtTime.plus(scaledTransform).plus(sampleToOdometryTransform);
  }

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public record OdometryObservation(
      SwerveModulePosition[] wheelPositions, Optional<Rotation2d> gyroAngle, double timestamp) {}

  public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {}
}
