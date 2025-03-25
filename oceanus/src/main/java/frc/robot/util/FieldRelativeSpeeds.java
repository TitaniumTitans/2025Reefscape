package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class FieldRelativeSpeeds {
  public double vx;
  public double vy;
  public double omega;

  public FieldRelativeSpeeds(double vx, double vy, double omega) {
    this.vx = vx;
    this.vy = vy;
    this.omega = omega;
  }

  public FieldRelativeSpeeds(ChassisSpeeds chassisSpeed, Rotation2d gyro) {
    this(chassisSpeed.vxMetersPerSecond * gyro.getCos() - chassisSpeed.vyMetersPerSecond * gyro.getSin(),
        chassisSpeed.vyMetersPerSecond * gyro.getCos() + chassisSpeed.vxMetersPerSecond * gyro.getSin(),
        chassisSpeed.omegaRadiansPerSecond);
  }

  public FieldRelativeSpeeds() {
    this.vx = 0.0;
    this.vy = 0.0;
    this.omega = 0.0;
  }
}