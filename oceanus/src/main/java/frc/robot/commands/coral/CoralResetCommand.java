package frc.robot.commands.coral;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.coral.CoralSubsystem;
import org.littletonrobotics.junction.Logger;


public class CoralResetCommand extends Command {
  private final CoralSubsystem coralSubsystem;
  private boolean limitTriggered = false;
  private boolean shouldEnd = false;
  private final Timer downTimer = new Timer();
  public CoralResetCommand(CoralSubsystem coralSubsystem) {
    this.coralSubsystem = coralSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.coralSubsystem);
  }

  @Override
  public void initialize() {
    downTimer.restart();

    limitTriggered = false;
    shouldEnd = false;
  }

  @Override
  public void execute() {
    // check initial conditions
    if (!coralSubsystem.limitHit() && !limitTriggered) {
      // watch for timer (coral above limit)
      if (downTimer.hasElapsed(0.1)) {
        Logger.recordOutput("Coral/ResetState", "Clearing Above Limit");
        coralSubsystem.setPivotVoltage(-2.0);
      } else {
        Logger.recordOutput("Coral/ResetState", "Moving up to limit");
        coralSubsystem.setPivotVoltage(1.0);
      }
    } else {
      // we hit the limit switch the first time
      if (!limitTriggered) {
        limitTriggered = true;
      }

      // are we still seeing limit switch?
      if (coralSubsystem.limitHit()) {
        Logger.recordOutput("Coral/ResetState", "Moving Past Limit");
        coralSubsystem.setPivotVoltage(1.0);
      } else {
        Logger.recordOutput("Coral/ResetState", "Finished");
        coralSubsystem.resetPivotAngle(92.0);
        shouldEnd = true;
      }
    }
  }

  @Override
  public boolean isFinished() {
    return shouldEnd;
  }

  @Override
  public void end(boolean interrupted) {
    coralSubsystem.setPivotVoltage(0.0);
  }
}
