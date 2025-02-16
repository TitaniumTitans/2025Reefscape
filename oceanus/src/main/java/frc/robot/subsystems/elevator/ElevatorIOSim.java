package frc.robot.subsystems.elevator;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim = new ElevatorSim(
      DCMotor.getKrakenX60Foc(2),
      1.0,
      5.0,
      Units.inchesToMeters(0.728),
      0.0,
      Units.inchesToMeters(35.617839),
      true,
      0.0,
      0.01,
      0.01
  );

  private final PIDController pidController;

  private double appliedVoltage = 0.0;
  private boolean useClosedLoop = false;

  public ElevatorIOSim() {
    pidController = new PIDController(1.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.elevatorPosition = Units.metersToInches(sim.getPositionMeters());

    sim.update(0.02);

    if (useClosedLoop) {
      appliedVoltage = pidController.calculate(inputs.elevatorPosition);
    }

    sim.setInputVoltage(appliedVoltage);

    inputs.elevatorAppliedVoltage = appliedVoltage;
    inputs.elevatorVelocity = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.elevatorCurrentDraw = new double[]{sim.getCurrentDrawAmps(), 0.0};
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    appliedVoltage = voltage;
  }

  @Override
  public void setElevatorPosition(double positionInches) {
    pidController.setSetpoint(positionInches);
  }

  @Override
  public void resetElevatorPosition(double positionInches) {
    sim.setState(Units.inchesToMeters(positionInches), 0.0);
  }
}
