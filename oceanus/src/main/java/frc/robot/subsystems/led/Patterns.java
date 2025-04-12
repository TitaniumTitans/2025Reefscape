package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;

public class Patterns {
  public static final LEDPattern ALIGNMENT_CORAL = LEDPattern.solid(Color.kYellow);
  public static final LEDPattern ALIGNMENT_ALGAE = LEDPattern.solid(Color.kGreen);
  public static final LEDPattern HAS_CORAL_GROUND = LEDPattern.solid(Color.kRed);
  public static final LEDPattern HAS_CORAL_ARM = LEDPattern.solid(Color.kBlue);
  public static final LEDPattern RAINBOW = LEDPattern.rainbow(255, 255);
}
