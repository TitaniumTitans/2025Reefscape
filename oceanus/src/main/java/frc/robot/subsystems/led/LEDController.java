package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import lombok.Setter;

public class LEDController {

  private static LEDController instance;

  public static LEDController getInstance() {
    if (instance == null) {
      instance = new LEDController(9, 61);
    }
    return instance;
  }

  private AddressableLED leds;
  private AddressableLEDBuffer ledsBuffer;

  @Setter
  private boolean hasCoralGround = false;
  @Setter
  private boolean hasCoralArm = false;
  @Setter
  private boolean algaeAlignment = false;
  @Setter
  private boolean coralAlignment = false;


  private final LEDPattern defaultPattern = Patterns.RAINBOW;

  protected LEDController(int port, int length) {
    leds = new AddressableLED(port);
    ledsBuffer = new AddressableLEDBuffer(length);

    leds.setLength(length);
    leds.setData(ledsBuffer);
    leds.start();

    applyPattern(defaultPattern);
  }

  public void applyPattern(LEDPattern pattern) {
    pattern.applyTo(ledsBuffer);
  }

  public void periodic() {
    if (coralAlignment) {
      applyPattern(Patterns.ALIGNMENT_CORAL);
    } else if (algaeAlignment) {
      applyPattern(Patterns.ALIGNMENT_ALGAE);
    } else if (hasCoralArm) {
      applyPattern(Patterns.HAS_CORAL_ARM);
    } else if (hasCoralGround) {
      applyPattern(Patterns.HAS_CORAL_GROUND);
    } else {
      applyPattern(Patterns.RAINBOW);
    }

    leds.setData(ledsBuffer);
    leds.start();
  }
}