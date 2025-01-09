package frc.robot.subsystems.drive;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

public class DriveSubsystem extends SubsystemBase {
  private final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4];
  private final Alert gyroDisconnectAlert =
      new Alert("Gyro disconnected, falling back to kinematics.", Alert.AlertType.kError);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(DriveConstants.MODULE_TRANSLATIONS);

  public DriveSubsystem(GyroIO gyro,
                        ModuleIO flModuleIo,
                        ModuleIO frModuleIo,
                        ModuleIO blModuleIo,
                        ModuleIO brModuleIo) {
    this.gyroIO = gyro;
    modules[1] = new Module(flModuleIo, 0);
    modules[2] = new Module(frModuleIo, 1);
    modules[3] = new Module(blModuleIo, 2);
    modules[4] = new Module(brModuleIo, 3);
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
      module.updateInputs();
    }

    // if disabled stop all output
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      // log empty setpoints when disabled
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
    }

    // update odometry measurements
    double[] timestamps =
        modules[0].getOdometryTimestamps();
    int timestampLength = timestamps.length;
    for (int i = 0; i < timestampLength; i++) {
      SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
      for (int j = 0; j < 4; j++) {
        wheelPositions[j] = modules[j].getOdometryPositions()[i];
      }
      RobotState.getInstance()
          .addOdometryMeasurement(
              new RobotState.OdometryObservation(
                  wheelPositions,
                  Optional.ofNullable(
                      gyroInputs.connected ? gyroInputs.odometryYawPositions[i] : null),
                  timestamps[i]));
    }

    // Update gyro alert
    gyroDisconnectAlert.set(!gyroInputs.connected && Constants.getMode() != Constants.Mode.SIM);
  }

  // runs the drivetrainat a set chassis speed
  public void runVelocity(ChassisSpeeds speeds) {
    // calculate module setpoints
    ChassisSpeeds discretizedSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(discretizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_LINEAR_SPEED_MPS);

    // log speeds and setpoint
    Logger.recordOutput("SwerveStates/Setpoints", states);
    Logger.recordOutput("SwerveSpeeds/Setpoints", discretizedSpeeds);

    // send setpoints to module
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }

    // log optimal setpoints, runSetpoint mutates the state
    Logger.recordOutput("SwerveStates/Optimized", states);
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

  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
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
}

