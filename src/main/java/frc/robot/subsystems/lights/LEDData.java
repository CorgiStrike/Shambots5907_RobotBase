package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.Animation;
import frc.robot.ShamLib.Candle.MultipleColorSegments;
import frc.robot.ShamLib.Candle.RGB;

public class LEDData {
  private RGB values;
  private Animation animation;
  private MultipleColorSegments segs;

  public LEDData(RGB values) {
    this.values = values;
  }

  public LEDData(Animation animation) {
    this.animation = animation;
  }

  public LEDData(MultipleColorSegments segs) {
    this.segs = segs;
  }

  /**
   * Use this constructor for light states that will manage the lights themselves (i.e. in a
   * separate command)
   */
  public LEDData() {}

  public void applyToCANdle(LightsIO io) {
    if (values != null) {
      io.setLEDs(values);
    } else if (animation != null) {
      io.animate(animation);
    } else if (segs != null) {
      io.setMultipleSegs(segs);
    }
  }
}
