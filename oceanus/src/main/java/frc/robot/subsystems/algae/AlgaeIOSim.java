package frc.robot.subsystems.algae;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AlgaeIOSim implements AlgaeIO {
  private SingleJointedArmSim pivotSim = new SingleJointedArmSim(
      DCMotor.getKrakenX60(1),
      (72.0 / 16.0) * (48.0 / 18.0),
      1.0,
      Units.inchesToMeters(17.5),
      0.0,
      2 * Math.PI,
      false,
      Units.degreesToRadians(90)
  );

  private double appliedPivotVolts = 0.0;
  private double appliedAlgaeVolts = 0.0;
  private boolean useClosedLoop = false;

  private PIDController pivotPID = new PIDController(1.0, 0.0, 0.0);

  @Override
  public void updateInputs(AlgaeIOInputsAutoLogged inputs) {
    // update simulation
    if (useClosedLoop) {
      appliedPivotVolts = pivotPID.calculate(Units.degreesToRotations(inputs.pivotAngle));
    }

    pivotSim.setInputVoltage(appliedPivotVolts);
    pivotSim.update(0.20);

    inputs.pivotAngle = Units.radiansToDegrees(pivotSim.getAngleRads());
    inputs.pivotCurrent = pivotSim.getCurrentDrawAmps();
    inputs.pivotVoltage = appliedPivotVolts;
    inputs.algaeCurret = 0.0;
    inputs.algaeVoltage = appliedAlgaeVolts;
    inputs.hasAlgae = false;
  }

  @Override
  public void setPivotVoltage(double voltage) {
    appliedPivotVolts = voltage;
    useClosedLoop = false;
  }

  @Override
  public void setPivotAngle(double degrees) {
    pivotPID.setSetpoint(Units.degreesToRotations(degrees));
    useClosedLoop = true;
  }
}
