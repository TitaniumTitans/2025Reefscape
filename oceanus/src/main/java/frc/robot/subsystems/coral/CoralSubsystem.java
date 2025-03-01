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

  public Command setPivotAngle(double angle) {
    return run
  }
}

