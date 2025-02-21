package frc.robot.subsystems.drive.module;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Current;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  class ModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRads = 0.0;
    public double driveVelocityRadsPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;

    public boolean steerConnected = false;
    public Rotation2d steerPosition = new Rotation2d();
    public double steerVelocityRadsPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;

    public boolean steerEncoderConnected = false;
    public Rotation2d steerAbsolutePosition = new Rotation2d();

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRads = new double[] {};
    public Rotation2d[] odometrySteerPositions = new Rotation2d[] {};
  }

  // updates the set of logged inputs
  public default void updateInputs(ModuleIOInputsAutoLogged inputs) {};

  // sets the voltage of the drive motor
  public default void setDriveOpenLoop(double volts) {};

  // sets the voltage of the steer motor
  public default void setSteerOpenLoop(double volts) {};

  // sets the drive motor to a certain velocity using onboard PID controllers
  public default void setDriveVelocity(double radsPerSec) {};
  public default void setDriveVelocity(double radsPerSec, Current torqueCurrent) {};

  // sets the steer motor position using onboard PID controllers
  public default void setSteerPosition(Rotation2d rotation) {};
}
