package frc.robot.subsystems.elevator;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ElevatorSubsystem extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public enum ElevatorState {
    ZEROING,
    POSITION_CONTROL,
    DISABLED
  }

  private ElevatorState goalState = ElevatorState.ZEROING;

  public ElevatorSubsystem(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Goal State", goalState);
  }
}

