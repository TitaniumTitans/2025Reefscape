package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim = new ElevatorSim(
      DCMotor.getKrakenX60Foc(2),
      1.0 / ElevatorConstants.GEAR_REDUCTION,
      0.1,
      ElevatorConstants.SPOOL_DIAMETER_METERS / 2.0,
      0.0,
      Units.inchesToMeters(70 / 2.0),
      false,
      0.0,
      0.0,
      0.0
  );

  private final PIDController pidController;

  private double appliedVoltage = 0.0;
  private boolean useClosedLoop = false;

  public ElevatorIOSim() {
    pidController = new PIDController(40.0, 0.0, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    inputs.elevatorPositionMeters = sim.getPositionMeters();

    Logger.recordOutput("ElevatorSim/PositionMeters", sim.getPositionMeters());

    sim.update(0.02);

    if (useClosedLoop) {
      appliedVoltage = pidController.calculate(inputs.elevatorPositionMeters);
    }

    sim.setInputVoltage(appliedVoltage);

    inputs.elevatorAppliedVoltage = new double[] {appliedVoltage, 0.0};
    inputs.elevatorVelocityMPS = sim.getVelocityMetersPerSecond();
    inputs.elevatorCurrentDraw = new double[] {sim.getCurrentDrawAmps(), 0.0};
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    appliedVoltage = voltage;
  }

  @Override
  public void setElevatorPosition(double positionMeters) {
    useClosedLoop = true;
    pidController.setSetpoint(positionMeters);
  }

  @Override
  public void resetElevatorPosition(double positionMeters) {
    sim.setState(Units.inchesToMeters(positionMeters), 0.0);
  }
}
