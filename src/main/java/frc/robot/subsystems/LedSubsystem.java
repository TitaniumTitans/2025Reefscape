package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class LedSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final int LED_ID = 0;
  //Not sure how many LEDS so arbitrary number, same thing with the ID
  private final int NUM_LED = 400;
  private boolean prevHasCoral;
  private final Animation empty = new RainbowAnimation(0.0, 1.0, NUM_LED);
  private final Animation full = new RainbowAnimation(1.0, 0.75, NUM_LED);

  public LedSubsystem(Boolean hasCoral) {
    candle = new CANdle(LED_ID);
    RobotState.getInstance().setHasCoral(hasCoral);
    prevHasCoral = hasCoral;
  }
  @Override
  public void periodic() {
    if (RobotState.getInstance().isHasCoral() && prevHasCoral) {
      candle.animate(full);
    } else if (!RobotState.getInstance().isHasCoral() && !prevHasCoral) {
      candle.animate(empty);
    }

    if (RobotState.getInstance().isHasCoral() != prevHasCoral) {
      prevHasCoral = RobotState.getInstance().isHasCoral();
    }
  }
}
