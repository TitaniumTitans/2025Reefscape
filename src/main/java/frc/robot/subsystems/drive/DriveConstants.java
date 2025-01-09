package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;

import static edu.wpi.first.units.Units.*;

public class DriveConstants {
  public static final double ODOMETRY_FREQUENCY = 250;
  public static final double TRACK_WIDTH_X = Units.inchesToMeters(20.75);
  public static final double TRACK_WIDTH_Y = Units.inchesToMeters(20.75);
  public static final Translation2d[] MODULE_TRANSLATIONS = {
      new Translation2d(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2),
      new Translation2d(-TRACK_WIDTH_X / 2, -TRACK_WIDTH_Y / 2)
  };

  public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH_X / 2, TRACK_WIDTH_Y / 2);
  public static final double MAX_LINEAR_SPEED_MPS = Units.feetToMeters(16.0);
  public static final double MAX_ANGULAR_SPEED = 4.69 / DRIVE_BASE_RADIUS;

  public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(2.0);

  // Pathplanner stuff
  public static final double WHEEL_COF = 1.2;

  public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
      .withCustomModuleTranslations(MODULE_TRANSLATIONS)
      .withRobotMass(Pounds.of(115))
      .withGyro(COTS.ofPigeon2())
      .withSwerveModule(new SwerveModuleSimulationConfig(
          DCMotor.getKrakenX60(1),
          DCMotor.getKrakenX60(1),
          6.122448979591837,
          21.428571428571427,
          Volts.of(0.1),
          Volts.of(0.1),
          Meters.of(WHEEL_RADIUS_METERS),
          KilogramSquareMeters.of(0.02),
          WHEEL_COF));
}
