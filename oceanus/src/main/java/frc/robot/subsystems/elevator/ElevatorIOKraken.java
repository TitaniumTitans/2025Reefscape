package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.gos.lib.phoenix6.properties.pid.Phoenix6TalonPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOKraken implements ElevatorIO {
  private final TalonFX master;
  private final TalonFX follower;
  private final DigitalInput lowerLimit;
  private final DigitalInput upperLimit;

  private final MotionMagicVoltage mmControl;
  private final Follower followerControl;

  private final StatusSignal<Angle> masterPositionSignal;
  private final StatusSignal<AngularVelocity> masterVelocitySignal;
  private final StatusSignal<Current> masterCurrentSignal;
  private final StatusSignal<Current> followerCurrentSignal;
  private final StatusSignal<Voltage> masterVoltageSignal;
  private final StatusSignal<Voltage> followerVoltageSignal;

  public ElevatorIOKraken() {
    master = new TalonFX(ElevatorConstants.MASTER_MOTOR_ID);
    follower = new TalonFX(ElevatorConstants.FOLLOWER_MOTOR_ID);

    configureDevices();

    mmControl = new MotionMagicVoltage(0.0)
        .withSlot(0)
        .withEnableFOC(false)
        .withUpdateFreqHz(100);
    followerControl = new Follower(master.getDeviceID(), true)
        .withUpdateFreqHz(100);
    follower.setControl(followerControl);

    lowerLimit = new DigitalInput(6);
    upperLimit = new DigitalInput(7);

    masterPositionSignal = master.getPosition();
    masterVelocitySignal = master.getVelocity();
    masterVoltageSignal = master.getMotorVoltage();
    masterCurrentSignal = master.getSupplyCurrent();

    followerVoltageSignal = follower.getMotorVoltage();
    followerCurrentSignal = follower.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0,
        masterPositionSignal,
        masterVelocitySignal,
        masterVoltageSignal,
        masterCurrentSignal,
        followerVoltageSignal,
        followerCurrentSignal
        );
  }

  @Override
  public void updateInputs(ElevatorIOInputsAutoLogged inputs) {
    BaseStatusSignal.refreshAll(
        masterPositionSignal,
        masterVelocitySignal,
        masterVoltageSignal,
        masterCurrentSignal,
        followerVoltageSignal,
        followerCurrentSignal
    );

    inputs.upperLimitSwitch = upperLimit.get();
    inputs.bottomLimitSwitch = lowerLimit.get();

    inputs.elevatorPositionMeters =
        masterPositionSignal.refresh().getValueAsDouble() * ElevatorConstants.SPOOL_DIAMETER_METERS * Math.PI;
    inputs.elevatorVelocityMPS =
        masterVelocitySignal.getValueAsDouble() * (ElevatorConstants.SPOOL_DIAMETER_METERS / 2.0);

    inputs.elevatorAppliedVoltage = new double[] {
        masterVoltageSignal.getValueAsDouble(),
        followerVoltageSignal.getValueAsDouble()
    };
    inputs.elevatorCurrentDraw = new double[] {
        masterCurrentSignal.getValueAsDouble(),
        followerCurrentSignal.getValueAsDouble()
    };

    Logger.recordOutput("Elevator/Raw Rotations", masterPositionSignal.getValueAsDouble());
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    master.setVoltage(voltage);
  }

  @Override
  public void setElevatorPosition(double positionMeters) {
    double rotations = positionMeters / (ElevatorConstants.SPOOL_DIAMETER_METERS  * Math.PI);
    Logger.recordOutput("Elevator/Raw Rotation Setpoint", rotations);
    master.setControl(mmControl.withPosition(rotations));
  }

  @Override
  public void resetElevatorPosition(double positionMeters) {
    master.setPosition(positionMeters);
    follower.setPosition(positionMeters);
  }

  private void configureDevices() {
    var config = new TalonFXConfiguration();

    // motion magic
//    config.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(120);
//    config.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(120);
    config.MotionMagic.MotionMagicCruiseVelocity =
        2 / (ElevatorConstants.SPOOL_DIAMETER_METERS  * Math.PI);
    config.MotionMagic.MotionMagicAcceleration =
        5 / (ElevatorConstants.SPOOL_DIAMETER_METERS  * Math.PI); // 100
    // 140

    config.Slot0.kP = ElevatorConstants.ELEVATOR_KP;
    config.Slot0.kI = ElevatorConstants.ELEVATOR_KI;
    config.Slot0.kD = ElevatorConstants.ELEVATOR_KD;
    config.Slot0.kS = ElevatorConstants.ELEVATOR_KS;
    config.Slot0.kG = ElevatorConstants.ELEVATOR_KG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

    // current limits at default
    config.CurrentLimits.SupplyCurrentLimit = 60;
    config.CurrentLimits.StatorCurrentLimit = 80;

    // PID control stuff
    config.Feedback.SensorToMechanismRatio = 1.0 / ElevatorConstants.GEAR_REDUCTION;

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> master.getConfigurator().apply(config));

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));

    master.setPosition(0.0);
    follower.setPosition(0.0);
  }
}
