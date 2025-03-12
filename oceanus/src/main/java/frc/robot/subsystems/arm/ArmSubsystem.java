package frc.robot.subsystems.arm;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
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

    if (DriverStation.isDisabled()) {
      io.setArmPivotVoltage(0.0);
    }
  }

  public Command setArmPosition(Rotation2d angle) {
    return run(() -> io.setArmPivotAngle(angle));
  }

  public Command setArmVoltageFactory(double voltage) {
    return runEnd(
        () -> io.setArmPivotVoltage(voltage),
        () -> io.setArmPivotVoltage(0.0)
    );
  }

  public Command setRollerVoltageFactory(double voltage) {
    return runEnd(
        () -> io.setRollerVoltage(voltage),
        () -> io.setRollerVoltage(0.0)
    );
  }
}

