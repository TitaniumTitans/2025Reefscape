package frc.robot.subsystems.arm;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.util.MechanismVisualizer;
import lombok.extern.java.Log;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private Rotation2d armSetpoint = new Rotation2d();

  @AutoLogOutput(key = "Arm/Arm State")
  private ArmState goalState = ArmState.DISABLED;



  public enum ArmState {
    VOLTAGE_CONTROL,
    POSITION_CONTROL,
    DISABLED;
  }
  public ArmSubsystem(ArmIO io) {
    AutoLogOutputManager.addObject(this);
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Arm", inputs);
    Logger.recordOutput("Arm/Setpoint", armSetpoint);

    if (DriverStation.isDisabled() && goalState != ArmState.DISABLED) {
      goalState = ArmState.DISABLED;
      io.setArmPivotVoltage(0.0);
    }

    switch (goalState) {
      case POSITION_CONTROL -> io.setArmPivotAngle(armSetpoint);
      case DISABLED -> {
        armSetpoint = inputs.armAngle;
        io.setArmPivotAngle(armSetpoint);
      }
      default -> {
        // Do nothing
      }
    }

    MechanismVisualizer.getInstance().setArmAngleDegrees(inputs.armAngle.getDegrees());
  }

  public void setDisabled() {
    goalState = ArmState.DISABLED;
  }

  public void setArmPosition(Rotation2d angle) {
    goalState = ArmState.POSITION_CONTROL;
    armSetpoint = angle;
  }

  public void setArmRollerVoltage(double voltage) {
    io.setRollerVoltage(voltage);
  }

  public boolean atSetpoint() {
    return atSetpoint(armSetpoint.getDegrees());
  }

  public boolean hasCoral() {
    return inputs.hasCoral;
  }

  public boolean atSetpoint(double setpoint) {
    return MathUtil.isNear(
        setpoint,
        inputs.armAngle.getDegrees(),
        5
    );
  }

  public boolean atHome() {
    return MathUtil.isNear(
        ArmConstants.ARM_HOME_SETPOINT.getDegrees(),
        inputs.armAngle.getDegrees(),
        5
    );
  }

  public Command setArmPositionFactory(Rotation2d angle) {
    return runOnce(() -> {
      goalState = ArmState.POSITION_CONTROL;
      armSetpoint = angle;
    });
  }

  public Command setArmVoltageFactory(double voltage) {
    return runEnd(
        () -> {
          goalState = ArmState.VOLTAGE_CONTROL;
          io.setArmPivotVoltage(voltage);
          },
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
    return Commands.run(() -> {}).until(this::atSetpoint);
  }
}

