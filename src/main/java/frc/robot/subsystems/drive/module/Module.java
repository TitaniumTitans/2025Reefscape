package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Alert;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
      TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert steerDisconnectedAlert;
  private final Alert steerEncoderDisconnectedAlert;
  private final SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
        constants
  ) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    

    driveDisconnectedAlert = new Alert(
        "Disconnected drive motor on module " + index + ".",
        Alert.AlertType.kError
    );
    steerDisconnectedAlert = new Alert(
        "Disconnected steer motor on module " + index + ".",
        Alert.AlertType.kError
    );
    steerEncoderDisconnectedAlert = new Alert(
        "Disconnected CANCoder on module " + index + ".",
        Alert.AlertType.kError
    );
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + index, inputs);

    // Update alerts
    driveDisconnectedAlert.set(inputs.driveConnected);
    steerDisconnectedAlert.set(inputs.steerConnected);
    steerEncoderDisconnectedAlert.set(inputs.steerEncoderConnected);
  }
}
