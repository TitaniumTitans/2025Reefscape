package frc.robot.subsystems.arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.MechanismVisualizer;
import lombok.extern.java.Log;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private Rotation2d armSetpoint = new Rotation2d();
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

    MechanismVisualizer.getInstance().setArmAngleDegrees(inputs.armAngle.getDegrees());
  }

  public Command setArmPosition(Rotation2d angle) {
    armSetpoint = angle;
    Logger.recordOutput("Arm/Setpoint", armSetpoint);
    return run(() -> io.setArmPivotAngle(armSetpoint));
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

  public Command waitUntilAtSetpoint() {
    return Commands.none().until(
        () -> MathUtil.isNear(
            armSetpoint.getDegrees(),
            inputs.armAngle.getDegrees(),
            5
        )
    );
  }
}

