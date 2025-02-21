package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.BooleanSupplier;

public class LedSubsystem extends SubsystemBase {
  private final CANdle candle;
  private final int LED_ID = 0;
  //Not sure how many LEDS so arbitrary number, same thing with the ID
  private final int NUM_LED = 400;
  private final BooleanSupplier hasCoral;
  private boolean prevHasCoral;
  private final Animation empty = new RainbowAnimation(0.0, 1.0, NUM_LED);
  private final Animation full = new RainbowAnimation(1.0, 0.75, NUM_LED);

  public LedSubsystem(BooleanSupplier hasCoral) {
    candle = new CANdle(LED_ID);
    this.hasCoral = hasCoral;
    prevHasCoral = hasCoral.getAsBoolean();
  }
  @Override
  public void periodic() {
    if (hasCoral.getAsBoolean() && prevHasCoral) {
      candle.animate(full);
    } else if (!hasCoral.getAsBoolean() && !prevHasCoral) {
      candle.animate(empty);
    }

    if (hasCoral.getAsBoolean() != prevHasCoral) {
      prevHasCoral = hasCoral.getAsBoolean();
    }
  }
}
