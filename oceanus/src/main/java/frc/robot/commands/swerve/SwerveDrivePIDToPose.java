package frc.robot.commands.swerve;

import java.util.function.Supplier;

import com.stuypulse.stuylib.control.angle.feedback.AnglePIDController;
import com.stuypulse.stuylib.control.feedback.PIDController;
import com.stuypulse.stuylib.control.feedforward.MotorFeedforward;
import com.stuypulse.stuylib.math.Vector2D;
import com.stuypulse.stuylib.streams.angles.filters.AMotionProfile;
import com.stuypulse.stuylib.streams.booleans.BStream;
import com.stuypulse.stuylib.streams.booleans.filters.BDebounceRC;
import com.stuypulse.stuylib.streams.numbers.IStream;
import com.stuypulse.stuylib.streams.numbers.filters.LowPassFilter;
import com.stuypulse.stuylib.streams.vectors.VStream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.util.HolonomicController;
import frc.robot.util.TranslationMotionProfileIan;
import org.littletonrobotics.junction.Logger;

public class SwerveDrivePIDToPose extends Command {

  private final DriveSubsystem swerve;

  private final HolonomicController controller;
  private final Supplier<Pose2d> pose2dSupplier;
  private Pose2d targetPose;

  private double maxVelocity;
  private double maxAcceleration;

  private final BStream isAligned;
  private final IStream velocityError;

  private Number xTolerance;
  private Number yTolerance;
  private Number thetaTolerance;
  private Number maxVelocityWhenAligned;

  private VStream translationSetpoint;

  public SwerveDrivePIDToPose(DriveSubsystem swerve, Pose2d pose2dSupplier) {
    this(swerve, () -> pose2dSupplier);
  }

  public SwerveDrivePIDToPose(DriveSubsystem swerve, Supplier<Pose2d> pose2dSupplier) {
    this.swerve = swerve;

    controller = new HolonomicController(
        new PIDController(DriveConstants.XY_KP, DriveConstants.XY_KI, DriveConstants.XY_KD).add(new MotorFeedforward(0, 0, 0).position()),
        new PIDController(DriveConstants.XY_KP, DriveConstants.XY_KI, DriveConstants.XY_KD).add(new MotorFeedforward(0, 0, 0).position()),
        new AnglePIDController(DriveConstants.THETA_KP, DriveConstants.THETA_KI, DriveConstants.THETA_KD)
            .setSetpointFilter(new AMotionProfile(DriveConstants.MAX_ALIGNMENT_ANGULAR_VELOCITY, DriveConstants.MAX_ALIGNMENT_ANGULAR_ACCELERATION)));

    maxVelocity = DriveConstants.MAX_ALIGNMENT_LINEAR_VELOCITY;
    maxAcceleration = DriveConstants.MAX_ALIGNMENT_LINEAR_ACCELERATION;

    translationSetpoint = getNewTranslationSetpointGenerator();

    this.pose2dSupplier = pose2dSupplier;

    isAligned = BStream.create(this::isAligned)
        .filtered(new BDebounceRC.Both(0.15));

    velocityError = IStream.create(() -> new Translation2d(controller.getError().vxMetersPerSecond, controller.getError().vyMetersPerSecond).getNorm())
        .filtered(new LowPassFilter(0.05))
        .filtered(x -> Math.abs(x));

    xTolerance = Units.inchesToMeters(2.0);
    yTolerance = Units.inchesToMeters(2.0);
    thetaTolerance = Units.degreesToRadians(5.0);
    maxVelocityWhenAligned = 0.15;

    addRequirements(swerve);
  }

  public SwerveDrivePIDToPose withTolerance(Number x, Number y, Number theta) {
    xTolerance = x;
    yTolerance = y;
    thetaTolerance = theta;
    return this;
  }

  public SwerveDrivePIDToPose withTranslationalConstraints(double maxVelocity, double maxAcceleration) {
    this.maxVelocity = maxVelocity;
    this.maxAcceleration = maxAcceleration;
    return this;
  }

  public SwerveDrivePIDToPose withoutMotionProfile() {
    this.translationSetpoint = VStream.create(() -> new Vector2D(pose2dSupplier.get().getTranslation()));
    return this;
  }

  // the VStream needs to be recreated everytime the command is scheduled to allow the target tranlation to jump to the start of the path
  private VStream getNewTranslationSetpointGenerator() {
    return VStream.create(() -> new Vector2D(targetPose.getTranslation()))
        .filtered(new TranslationMotionProfileIan(
            this.maxVelocity,
            this.maxAcceleration,
            new Vector2D(RobotState.getInstance().getEstimatedPose().getTranslation()),
            Vector2D.kOrigin));
  }

  @Override
  public void initialize() {
    targetPose = pose2dSupplier.get();
    translationSetpoint = getNewTranslationSetpointGenerator();
  }

  private boolean isAlignedX() {
    return Math.abs(targetPose.getX() - RobotState.getInstance().getEstimatedPose().getX()) < xTolerance.doubleValue();
  }

  private boolean isAlignedY() {
    return Math.abs(targetPose.getY() - RobotState.getInstance().getEstimatedPose().getY()) < yTolerance.doubleValue();
  }

  private boolean isAlignedTheta() {
    return Math.abs(targetPose.getRotation().minus(RobotState.getInstance().getEstimatedPose().getRotation()).getRadians()) < thetaTolerance.doubleValue();
  }

  private boolean isAligned() {
    return isAlignedX() && isAlignedY() && isAlignedTheta() && velocityError.get() < maxVelocityWhenAligned.doubleValue();
  }

  @Override
  public void execute() {
    controller.update(new Pose2d(translationSetpoint.get().getTranslation2d(), targetPose.getRotation()), RobotState.getInstance().getEstimatedPose());

    swerve.runVelocity(new ChassisSpeeds(
        controller.getOutput().vxMetersPerSecond,
        controller.getOutput().vyMetersPerSecond,
        controller.getOutput().omegaRadiansPerSecond));

    Logger.recordOutput("Alignment/Target", targetPose);
    Logger.recordOutput("Alignment/Target X", targetPose.getX());
    Logger.recordOutput("Alignment/Target Y", targetPose.getY());
    Logger.recordOutput("Alignment/Target Theta", targetPose.getRotation());

    Logger.recordOutput("Alignment/Output X", controller.getOutput().vxMetersPerSecond);
    Logger.recordOutput("Alignment/Output Y", controller.getOutput().vyMetersPerSecond);
    Logger.recordOutput("Alignment/Output Theta", controller.getOutput().omegaRadiansPerSecond);
  }

  @Override
  public boolean isFinished() {
    return isAligned.get();
  }

  @Override
  public void end(boolean interrupted) {
    swerve.runVelocity(new ChassisSpeeds());
  }

}