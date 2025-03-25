package frc.robot.subsystems.algae;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismVisualizer;
import org.littletonrobotics.junction.Logger;

public class AlgaeSubsystem extends SubsystemBase {
  private final AlgaeIO io;
  private final AlgaeIOInputsAutoLogged inputs = new AlgaeIOInputsAutoLogged();

  public AlgaeSubsystem(AlgaeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Algae", inputs);

    MechanismVisualizer.getInstance().setAlgaeAngleDegrees(inputs.pivotAngle);
  }

  public Command setAlgaeAngle(double angle) {
    return runOnce(() -> io.setPivotAngle(angle));
  }
}

