package frc.robot.subsystems.drive;


import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.module.Module;
import frc.robot.subsystems.drive.module.ModuleIO;

public class DriveSubsystem extends SubsystemBase {
  private final GyroIO gyroIO;
  private final Module[] modules = new Module[4];
  private final Alert gyroDisconnectAlert =
      new Alert("Gyro disconnected, falling back to kinematics.", Alert.AlertType.kError);

  public DriveSubsystem(GyroIO gyro,
                        ModuleIO flModuleIo,
                        ModuleIO frModuleIo,
                        ModuleIO blModuleIo,
                        ModuleIO brModuleIo) {
    this.gyroIO = gyro;
    modules[1] = new Module(flModuleIo, 0);
    modules[2] = new Module(frModuleIo, 1);
    modules[3] = new Module(blModuleIo, 2);
    modules[4] = new Module(brModuleIo, 3);
  }

  @Override
  public void periodic() {

  }
}

