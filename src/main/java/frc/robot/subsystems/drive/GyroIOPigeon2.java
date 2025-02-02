package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOInputsAutoLogged;

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon = new Pigeon2(13, "canivore");
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(250);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();

    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw);
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
  }

  @Override
  public void updateInputs(GyroIOInputsAutoLogged inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadsPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawPositions = yawPositionQueue.stream()
        .map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
    inputs.odometryYawTimestamps = yawTimestampQueue.stream()
        .mapToDouble((Double value) -> value).toArray();

    yawPositionQueue.clear();
    yawTimestampQueue.clear();
  }

  @Override
  public void reset(Rotation2d angle) {
    pigeon.setYaw(angle.getMeasure());
  }
}