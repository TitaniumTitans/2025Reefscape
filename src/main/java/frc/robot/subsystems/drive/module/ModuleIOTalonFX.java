package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final SwerveModuleConstants<
      TalonFXConfiguration,
      TalonFXConfiguration,
      CANcoderConfiguration> config;

  public ModuleIOTalonFX( config) {
    this.config = config;

  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {

  }


}
