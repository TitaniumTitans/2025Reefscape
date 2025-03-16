package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import lombok.Builder;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;

public class DriveConstants {
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(24.0);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(25.0);
  public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
  };

  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
  public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(17.1 );
  public static final double MAX_ANGULAR_SPEED = MAX_LINEAR_SPEED_MPS / DRIVE_BASE_RADIUS;

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.967); //Units.inchesToMeters(1.976); // 2.0
  public static final double DRIVE_GEAR_RATIO = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static final double STEER_GEAR_RATIO = 18.75;

  // Pathplanner stuff
  public static final double WHEEL_COF = 1.2;

  public static final double XY_KP = 0.0;
  public static final double THETA_KP = 0.0;

  public static final TrapezoidProfile.Constraints XY_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1.0, 1.0); // m/s, m/s^2
  public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS =
      new TrapezoidProfile.Constraints(1.0, 1.0); // rad/s, rad/s^2

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
      .withCustomModuleTranslations(MODULE_TRANSLATIONS)
      .withRobotMass(Pounds.of(115))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60(1),
          DCMotor.getKrakenX60(1),
          DRIVE_GEAR_RATIO,
          STEER_GEAR_RATIO,
          Volts.of(0.1),
          Volts.of(0.1),
          Meters.of(WHEEL_RADIUS_METERS),
          KilogramSquareMeters.of(0.02),
          WHEEL_COF));

  public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
      Pounds.of(115),
      KilogramSquareMeters.of(6),
      new ModuleConfig(
          WHEEL_RADIUS_METERS,
          MAX_LINEAR_SPEED_MPS,
          1.2,
          DCMotor.getKrakenX60Foc(1)
              .withReduction(DRIVE_GEAR_RATIO),
          70,
          1
      ),
      MODULE_TRANSLATIONS
  );

  public static final ModuleConstants[] MODULE_CONSTANTS;
  static {
    switch (Constants.getMode()) {
      case REAL -> {
        MODULE_CONSTANTS = new ModuleConstants[]{
            // Front Left
            ModuleConstants.builder()
                .driveId(1)
                .steerId(2)
                .encoderId(3)
                .encoderOffset(Rotation2d.fromRotations(0.087891))
                .steerInverted(true)
                .turnInverted(false)
                .build(),
            // Front Right
            ModuleConstants.builder()
                .driveId(4)
                .steerId(5)
                .encoderId(6)
                .encoderOffset(Rotation2d.fromRotations(-0.429932))
                .steerInverted(true)
                .turnInverted(false)
                .build(),
            // Back Left
            ModuleConstants.builder()
                .driveId(7)
                .steerId(8)
                .encoderId(9)
                .encoderOffset(Rotation2d.fromRotations(0.131104))
                .steerInverted(true)
                .turnInverted(false)
                .build(),
            // Back Right
            ModuleConstants.builder()
                .driveId(10)
                .steerId(11)
                .encoderId(12)
                .encoderOffset(Rotation2d.fromRotations(0.377441))
                .steerInverted(true)
                .turnInverted(false)
                .build()
        };
      }
      default -> {
        MODULE_CONSTANTS = new ModuleConstants[] {};
      }
    }
  }

  // per module config object
  @Builder
  public record ModuleConstants(
      int driveId,
      int steerId,
      int encoderId,
      Rotation2d encoderOffset,
      boolean turnInverted,
      boolean steerInverted
  ) {}
}
