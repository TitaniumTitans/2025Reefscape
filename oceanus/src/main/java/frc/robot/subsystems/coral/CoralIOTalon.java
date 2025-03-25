package frc.robot.subsystems.coral;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;

public class CoralIOTalon implements CoralIO {
  private final TalonFX pivot;
  private final TalonFX hopper;
  private final TalonFX outerCoral;
  private final TalonFX innerCoral;

  private final LaserCan laserCan;
  private final DigitalInput limit;

  private final MotionMagicVoltage mmControl;

  public CoralIOTalon() {
    pivot = new TalonFX(CoralConstants.PIVOT_ID);
    hopper = new TalonFX(CoralConstants.HOPPER_ID);
    outerCoral = new TalonFX(CoralConstants.OUTER_ID);
    innerCoral = new TalonFX(CoralConstants.INNER_ID);
    laserCan = new LaserCan(CoralConstants.LASER_CAN_ID);
    limit = new DigitalInput(8);

    mmControl = new MotionMagicVoltage(0.0);
    pivot.setPosition(75.5);

    configureDevices();
  }

  @Override
  public void updateInputs(CoralIOInputsAutoLogged inputs) {
    inputs.coralAppliedVoltage = new double[] {
        pivot.getMotorVoltage().getValueAsDouble(),
        hopper.getMotorVoltage().getValueAsDouble(),
        outerCoral.getMotorVoltage().getValueAsDouble(),
        innerCoral.getMotorVoltage().getValueAsDouble()
    };
    inputs.coralCurrentDraw = new double[] {
        pivot.getSupplyCurrent().getValueAsDouble(),
        hopper.getSupplyCurrent().getValueAsDouble(),
        outerCoral.getSupplyCurrent().getValueAsDouble(),
        innerCoral.getSupplyCurrent().getValueAsDouble()
    };
    inputs.coralPivotAngle = Rotation2d.fromRotations(pivot.getPosition().getValueAsDouble());

    inputs.limitHit = limit.get();

//    if (inputs.limitHit && inputs.coralPivotAngle.getDegrees() != 90.0) {
//      pivot.setPosition(Units.degreesToRotations(90.0));
//    }

    var measurement = laserCan.getMeasurement();
    if (measurement != null) {
      inputs.hasCoral = measurement.distance_mm < 70;
    } else {
      inputs.hasCoral = false;
    }
  }

  @Override
  public void setPivotAngle(double angleDegrees) {
    pivot.setControl(mmControl.withPosition(Units.degreesToRotations(angleDegrees)));
  }

  @Override
  public void setPivotVoltage(double appliedVolts) {
    pivot.setVoltage(appliedVolts);
  }

  @Override
  public void setHopperVoltage(double appliedVolts) {
    hopper.setVoltage(appliedVolts);
  }

  @Override
  public void setCoralVoltage(double outerVolt, double innerVolt) {
    outerCoral.setVoltage(outerVolt);
    innerCoral.setVoltage(innerVolt);
  }

  @Override
  public void resetPivot(double angleDegrees) {
    pivot.setPosition(Units.degreesToRotations(angleDegrees));
  }

  public void configureDevices() {
    var motorConfig = new TalonFXConfiguration();

    motorConfig.Feedback.SensorToMechanismRatio = CoralConstants.PIVOT_GEAR_RATIO;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motorConfig.Slot0.kP = CoralConstants.KP;
    motorConfig.Slot0.kI = CoralConstants.KI;
    motorConfig.Slot0.kD = CoralConstants.KD;
    motorConfig.Slot0.kS = CoralConstants.KS;
    motorConfig.Slot0.kG = CoralConstants.KG;
    motorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    motorConfig.MotionMagic.MotionMagicAcceleration = 5;

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
    motorConfig.CurrentLimits.StatorCurrentLimit = 60;

    PhoenixUtil.tryUntilOk(5, () -> pivot.getConfigurator().apply(motorConfig));
  }
}
