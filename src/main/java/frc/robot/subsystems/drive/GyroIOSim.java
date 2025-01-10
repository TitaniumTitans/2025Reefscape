package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputsAutoLogged inputs) {
    inputs.connected = true;
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadsPerSec =
        gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
  }
}