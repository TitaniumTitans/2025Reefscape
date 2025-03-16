package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ClimberSubsystem extends SubsystemBase {
  private final ClimberIO io;
  private final ClimberIOInputsAutoLogged inputs;
  public ClimberSubsystem(ClimberIO io) {
    this.io = io;
    inputs = new ClimberIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
  }
  public void setClimberPower(double power) {
    io.setMotorVoltage(power);
  }
  public Command setClimberPowerFactory(double power) {
    return runEnd(() -> {
          double climberPower = power;
          if (inputs.position.getDegrees() > 1600) {
            climberPower = MathUtil.clamp(climberPower, -12, 0);
          }
          setClimberPower(climberPower);
        },
        () -> setClimberPower(0.0));
  }
}
