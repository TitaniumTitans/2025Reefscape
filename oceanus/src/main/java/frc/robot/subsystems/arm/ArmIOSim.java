package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim armSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      ArmConstants.PIVOT_GEAR_RATIO,
      1.0,
      1.5,
      -Math.PI * 2,
      Math.PI * 2,
      false,
      -Math.PI / 2.0,
      0.0,
      0.0
  );

  private final PIDController pid =
      new PIDController(20.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double appliedVoltage = 0.0;
  private double setpointRadians = 0.0;

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    if (closedLoop) {
      appliedVoltage = pid.calculate(armSim.getAngleRads(), setpointRadians);
    }

    armSim.setInputVoltage(appliedVoltage);
    armSim.update(0.02);

    inputs.armAngle = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.absoluteArmAngle = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.armVoltages = new double[] {appliedVoltage, 0.0};
  }

  @Override
  public void setArmPivotVoltage(double voltage) {
    appliedVoltage = voltage;
    closedLoop = false;
  }

  @Override
  public void setArmPivotAngle(Rotation2d angle) {
    closedLoop = true;
    setpointRadians = angle.getRadians();
  }
}
