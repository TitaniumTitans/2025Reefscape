package frc.robot.subsystems.drive;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.drive.DriveConstants.ROBOT_CONFIG;

public class DriveSubsystem extends SubsystemBase {
  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final Alert gyroDisconnectAlert =
      new Alert("Gyro disconnected, falling back to kinematics.", Alert.AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);

  private final SwerveDriveOdometry wpiOdom;
  private SwerveSetpoint prevSetpoint;
  private final SwerveSetpointGenerator setpointGenerator;
  private final SysIdRoutine sysId;

  public DriveSubsystem(GyroIO gyro,
                        ModuleIO flModuleIo,
                        ModuleIO frModuleIo,
                        ModuleIO blModuleIo,
                        ModuleIO brModuleIo) {
    AutoLogOutputManager.addObject(this);

    this.gyroIO = gyro;
    modules[0] = new Module(flModuleIo, 0);
    modules[1] = new Module(frModuleIo, 1);
    modules[2] = new Module(blModuleIo, 2);
    modules[3] = new Module(brModuleIo, 3);

    PhoenixOdometryThread.getInstance().start();

    wpiOdom = new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());

    RobotState.getInstance().resetPose(new Pose2d());
    setpointGenerator = new SwerveSetpointGenerator(ROBOT_CONFIG, Units.degreesToRadians(1080));

    prevSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates(), DriveFeedforwards.zeros(ROBOT_CONFIG.numModules));

    AutoBuilder.configure(
        RobotState.getInstance()::getEstimatedPose,
        this::resetPose,
        this::getChassisSpeeds,
        (ChassisSpeeds speeds, DriveFeedforwards feedforwards) ->  runVelocity(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.5, 0.0),
            new PIDConstants(5.0, 0.5, 0.0)
        ),
        ROBOT_CONFIG,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
    );

    PathPlannerLogging.setLogActivePathCallback(
        (List<Pose2d> path) -> Logger.recordOutput("PathPlanner/ActivePath", path.toArray(Pose2d[]::new))
    );
    PathPlannerLogging.setLogCurrentPoseCallback(
        (Pose2d pose) -> Logger.recordOutput("PathPlanner/CurrentPose", pose)
    );
    PathPlannerLogging.setLogTargetPoseCallback(
        (Pose2d pose) -> Logger.recordOutput("PathPlanner/TargetPose", pose)
    );

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroInputs);

    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();

    // update the module periodic
    for (var module : modules) {
      module.periodic();
    }

    // if disabled stop all output
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      // log empty setpoints when disabled
      Logger.recordOutput("SwerveStates/Setpoints", prevSetpoint.moduleStates());
      Logger.recordOutput("SwerveStates/Optimized", new SwerveModuleState[]{});

      prevSetpoint = new SwerveSetpoint(
          new ChassisSpeeds(),
          getModuleStates(),
          DriveFeedforwards.zeros(4)
      );
    }

    // update odometry measurements
    double[] timestamps =
        modules[0].getOdometryTimestamps();
    int timestampLength = Math.min(timestamps.length, gyroInputs.odometryYawTimestamps.length);
    for (int i = 0; i < timestampLength; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryMeasurement(gyroInputs.odometryYawPositions[i],
              wheelPositions,
              timestamps[i]);
    }

    wpiOdom.update(getGyroRotation(), getModulePositions());
    Logger.recordOutput("RobotState/WPIOdometry", wpiOdom.getPoseMeters());

    // Update gyro alert
    gyroDisconnectAlert.set(!gyroInputs.connected && Constants.getMode() != Constants.Mode.SIM);
  }

  // runs the drivetrainat a set chassis speed
  public void runVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] setpointStates;

    // calculate module setpoints
    if (!DriverStation.isTeleop()) {
      ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
      setpointStates = kinematics.toSwerveModuleStates(discretizedSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, DriveConstants.MAX_LINEAR_SPEED_MPS);

      Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
      Logger.recordOutput("SwerveSpeeds/Setpoints", speeds);
      Logger.recordOutput("SwerveSpeeds/Optimized", discretizedSpeeds);
      Logger.recordOutput("SwerveStates/UsingSetpoints", false);
    } else {
      prevSetpoint = setpointGenerator.generateSetpoint(
          prevSetpoint,
          speeds,
          0.02
      );

      Logger.recordOutput("SwerveStates/Setpoints", prevSetpoint.moduleStates());
      Logger.recordOutput("SwerveSpeeds/Setpoints", prevSetpoint.robotRelativeSpeeds());
      Logger.recordOutput("SwerveStates/UsingSetpoints", true);

      setpointStates = prevSetpoint.moduleStates();
    }

    Logger.recordOutput("SwerveStates/Actual Setpoints", setpointStates);

    // send setpoints to module
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // log optimal setpoints, runSetpoint mutates the state
    Logger.recordOutput("SwerveStates/Optimized", setpointStates);
  }

  public void resetGyro(Rotation2d angle) {
    gyroIO.reset(angle);
  }

  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  @AutoLogOutput(key = "SwerveSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  public Rotation2d getGyroRotation() {
    return gyroInputs.yawPosition;
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.MAX_LINEAR_SPEED_MPS;
  }

  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.MAX_ANGULAR_SPEED;
  }

//  public Command driveToPose(Pose2d pose) {
//
//  }

  public void resetPose(Pose2d pose) {
    Logger.recordOutput("Pose Reset To", pose);
    RobotState.getInstance().resetPose(pose);
  }

  public Command resetPoseFactory(Pose2d pose) {
    return runOnce(() -> resetPose(pose));
  }

  public Command resetPoseFactory(Supplier<Pose2d> pose) {
    return resetPoseFactory(pose.get());
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }
}

