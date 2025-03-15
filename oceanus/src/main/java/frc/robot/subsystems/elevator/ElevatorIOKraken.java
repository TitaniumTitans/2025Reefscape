package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import com.gos.lib.phoenix6.properties.pid.Phoenix6TalonPidPropertyBuilder;
import com.gos.lib.properties.pid.PidProperty;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;

public class ElevatorIOKraken implements ElevatorIO {
  private final TalonFX master;
  private final TalonFX follower;
  private final DigitalInput lowerLimit;
  private final DigitalInput upperLimit;

  private final MotionMagicVoltage mmControl;
  private final Follower followerControl;

  private final PidProperty masterProperty;

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

    masterProperty = new Phoenix6TalonPidPropertyBuilder(
        "Elevator/MasterPID", false, master, 0
    )
        .addP(ElevatorConstants.ELEVATOR_KP)
        .addI(ElevatorConstants.ELEVATOR_KI)
        .addD(ElevatorConstants.ELEVATOR_KD)
        .addKG(0.0, GravityTypeValue.Elevator_Static)
        .build();

    mmControl = new MotionMagicVoltage(0.0);
    followerControl = new Follower(master.getDeviceID(), true);

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
    masterProperty.updateIfChanged();

    inputs.upperLimitSwitch = upperLimit.get();
    inputs.bottomLimitSwitch = lowerLimit.get();

    inputs.elevatorPositionMeters =
        Units.metersToInches(masterPositionSignal.refresh().getValueAsDouble() * ElevatorConstants.SPOOL_DIAMETER_METERS * Math.PI);
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
  }

  @Override
  public void setElevatorVoltage(double voltage) {
    master.setVoltage(voltage);
    follower.setControl(followerControl);
  }

  @Override
  public void setElevatorPosition(double positionMeters) {
    double rotations = positionMeters / (ElevatorConstants.SPOOL_DIAMETER_METERS / 2.0);
    master.setControl(mmControl.withPosition(rotations));
    follower.setControl(followerControl);
  }

  @Override
  public void resetElevatorPosition(double positionMeters) {
    master.setPosition(0.0);
    follower.setPosition(0.0);
  }

  private void configureDevices() {
    var config = new TalonFXConfiguration();

    // motion magic
    config.MotionMagic.MotionMagicCruiseVelocity = Units.degreesToRotations(90);
    config.MotionMagic.MotionMagicAcceleration = Units.degreesToRotations(90);

    // current limits at default
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.StatorCurrentLimit = 80;

    // PID control stuff
    config.Feedback.SensorToMechanismRatio = 1.0 / ElevatorConstants.GEAR_REDUCTION;

    // Motor output
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    master.getConfigurator().apply(config);

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    follower.getConfigurator().apply(config);

    master.setPosition(0.0);
    follower.setPosition(0.0);
  }
}
