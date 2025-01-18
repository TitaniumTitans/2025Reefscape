package frc.robot.subsystems.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        //position, voltage, current draw, and any other important logging data
        public Rotation2d pivotPosititon = new Rotation2d();
        public double intakeVelocity = 0.0;
        public double pivotVelocity = 0.0;
        public double pivotVoltage = 0.0;
        public double intakeVoltage = 0.0;
        public double pivotCurrentDraw = 0.0;
        public double intakeCurrentDraw = 0.0;
        public double pivotTemperature = 0.0;
        public double intakeTemperature = 0.0;
    }
    default void setMotorVoltageIntake(double voltage) {}
    default void setMotorVoltagePivot(double voltage) {}
    default void updateInputs(IntakeIOInputsAutoLogged inputs) {}
    default void stop() {}
}
