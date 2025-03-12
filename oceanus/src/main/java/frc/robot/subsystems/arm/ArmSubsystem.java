package frc.robot.subsystems.arm;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  public ArmSubsystem(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Arm", inputs);
  }

  public Command setArmVoltage(double voltage) {
    return runEnd(
        () -> io.setArmPivotVoltage(voltage),
        () -> io.setArmPivotVoltage(0.0)
    );
  }
}

