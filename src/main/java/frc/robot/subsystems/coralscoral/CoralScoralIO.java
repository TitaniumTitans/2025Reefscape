package frc.robot.subsystems.coralscoral;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface CoralScoralIO {
    @AutoLog
    class CoralScoralIOInputs {
        public boolean hasPiece = false;
        public Rotation2d pivotPosition = new Rotation2d();
        public double pivotVelocity = 0.0;
        public double scorerVelocity = 0.0;
        public double scorerVoltage = 0.0;
        public double masterPivotVoltage = 0.0;
        public double followerPivotVoltage = 0.0;
        public double scorerCurrentDraw = 0.0;
        public double masterPivotCurrentDraw = 0.0;
        public double followerPivotCurrentDraw = 0.0;
        public double scorerTemperature = 0.0;
        public double masterPivotTemperature = 0.0;
        public double followerPivotTemperature = 0.0;
        public double[] coralRanges = {0.0, 0.0, 0.0, 0.0};
    }
    default void setMotorVoltageScorer(double voltage) {}
    default void setMotorVoltagePivot(double voltage) {}
    default void setPivotAngle(double angleDegrees) {}
    default void resetPosition() {}
    default void updateInputs(CoralScoralIOInputsAutoLogged inputs) {}
    default void stop() {}
}
