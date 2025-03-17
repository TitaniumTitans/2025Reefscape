package frc.robot.subsystems.coral;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralSubsystem extends SubsystemBase {
  private CoralIO io;
  private CoralIOInputsAutoLogged inputs;

  public CoralSubsystem(CoralIO io) {
    this.io = io;
    inputs = new CoralIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Coral", inputs);
  }

  public Command setPivotAngle(double angleDegrees) {
    return runOnce(() -> io.setPivotAngle(angleDegrees));
  }

  public Command setPivotVoltageFactory(double voltage) {
    return runEnd(
        () -> io.setPivotVoltage(voltage),
        this::stop);
  }

  public void stop() {
    io.setPivotVoltage(0.0);
  }

  public Command setScoringVoltages(double hopper, double outer, double inner) {
    return run(() -> {
      io.setHopperVoltage(hopper);
      io.setCoralVoltage(outer, inner);
    });
  }

  public Command resetPivotFactory() {
    return run(() -> io.setPivotVoltage(1.0))
        .until(() -> !inputs.limitHit)
        .andThen(
            run(() -> io.setPivotVoltage(-1.0))
                .until(() -> inputs.limitHit))
        .andThen(runOnce(() -> io.setPivotVoltage(0.0)))
        .andThen(runOnce(() -> io.resetPivot(90)));
  }
}

