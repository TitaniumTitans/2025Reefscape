package frc.robot.subsystems.coral;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    return runOnce(() -> {
      io.setHopperVoltage(hopper);
      io.setCoralVoltage(outer, inner);
    });
  }

  public Command resetPivotFactory() {
    return
        defer(() -> {
          Command command = Commands.none();
          if (!inputs.limitHit) {
            command = command.andThen(
                setPivotVoltageFactory(1.0)
                    .withTimeout(1.0)
                    .andThen(setPivotVoltageFactory(-1.0))
            ).until(() -> inputs.limitHit);
          }

          return command.andThen(
              setPivotVoltageFactory(1.0)
                  .until(() -> !inputs.limitHit)
          ).andThen(
              setPivotVoltageFactory(-1.0)
                  .until(() -> inputs.limitHit)
          ).andThen(
              setPivotVoltageFactory(0.0)
          ).andThen(
              runOnce(() -> setPivotAngle(90.0))
          );
        });
  }
}

