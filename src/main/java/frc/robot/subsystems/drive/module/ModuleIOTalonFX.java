package frc.robot.subsystems.drive.module;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.subsystems.drive.DriveConstants;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder encoder;
  private final DriveConstants.ModuleConstants config;

  public ModuleIOTalonFX(DriveConstants.ModuleConstants config) {
    this.config = config;

    driveMotor = new TalonFX(config.driveId());
    steerMotor = new TalonFX(config.steerId());
    encoder = new CANcoder(config.encoderId());
  }

  @Override
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {

  }


}
