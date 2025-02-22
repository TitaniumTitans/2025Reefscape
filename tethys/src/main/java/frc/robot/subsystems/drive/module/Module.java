package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.drive.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private final Alert driveDisconnectedAlert;
  private final Alert steerDisconnectedAlert;
  private final Alert steerEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index
  ) {
    this.io = io;
    this.index = index;

    driveDisconnectedAlert = new Alert(
        "Disconnected drive motor on module " + index + ".",
        Alert.AlertType.kError
    );
    steerDisconnectedAlert = new Alert(
        "Disconnected steer motor on module " + index + ".",
        Alert.AlertType.kError
    );
    steerEncoderDisconnectedAlert = new Alert(
        "Disconnected CANCoder on module " + index + ".",
        Alert.AlertType.kError
    );
  }

  public void periodic() {
    // calculate samples for odometry
    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRads[i] * DriveConstants.WHEEL_RADIUS_METERS;
      Rotation2d angle = inputs.odometrySteerPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(inputs.driveConnected);
    steerDisconnectedAlert.set(inputs.steerConnected);
    steerEncoderDisconnectedAlert.set(inputs.steerEncoderConnected);
  }

  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);
  }

  // optimizes a module setpoint and runs it
  public void runSetpoint(SwerveModuleState state, LinearAcceleration torqueCurrent) {
    // optimize the state
    state.optimize(getAngle());
    state.cosineScale(getAngle());

    // apply the state
    double speedRadsPerSecond = state.speedMetersPerSecond / DriveConstants.WHEEL_RADIUS_METERS;
    io.setDriveVelocity(speedRadsPerSecond, torqueCurrent);
    Logger.recordOutput("Module" + index +"/Torque Feedforward", torqueCurrent);

//    if (state.speedMetersPerSecond < 0.1) {
    io.setSteerPosition(state.angle);
//    }
  }

  // use for determining ff gains
  public void runCharacterization(double voltage) {
    io.setDriveOpenLoop(voltage);
    io.setSteerPosition(Rotation2d.kZero);
  }

  // disables the all motor outputs
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setSteerOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.steerPosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRads * DriveConstants.WHEEL_RADIUS_METERS;
  }

  public double getVelocityMetersPerSecond() {
    return inputs.driveVelocityRadsPerSec * DriveConstants.WHEEL_RADIUS_METERS;
  }

  // gets the module state
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
  }

  // gets the module position
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  // gets the wheel position in radians
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRads;
  }

  // gets the wheel velocity in rotations/sec (phoenix default unit)
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadsPerSec);
  }
}
