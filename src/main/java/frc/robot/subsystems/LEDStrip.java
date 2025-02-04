package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDStrip extends SubsystemBase {
  private static final CANdle candle = new CANdle(7);

  /*
  // Team colors
  public static final Color ORANGE = new Color(255, 25, 0);
  public static final Color BLUE = new Color(8, 32, 255);

  // Game piece colors
  public static final Color YELLOW = new Color(242, 60, 0);
  public static final Color PURPLE = new Color(184, 0, 185);

  // Indicator colors
  public static final Color WHITE = new Color(255, 230, 220);
  public static final Color GREEN = new Color(56, 209, 0);
  public static final Color BLACK = new Color(0, 0, 0);
  public static final Color RED = new Color(255, 0, 0);
  */

  public LEDSegment BatteryIndicator;
  public LEDSegment PressureIndicator;
  public LEDSegment MastEncoderIndicator;
  public LEDSegment BoomEncoderIndicator;
  public LEDSegment WristEncoderIndicator;
  public LEDSegment DriverStationIndicator;
  public LEDSegment MainStrip;

  public static final LEDSegment FirstBulb = new LEDSegment(0, 1, 0);
  public static final LEDSegment SecondBulb = new LEDSegment(1, 1, 0);

  public LEDStrip() {
    CANdleConfiguration candleConfiguration = new CANdleConfiguration();
    candleConfiguration.statusLedOffWhenActive = true;
    candleConfiguration.disableWhenLOS = false;
    candleConfiguration.stripType = LEDStripType.RGB;
    candleConfiguration.brightnessScalar = 0.5;
    candleConfiguration.vBatOutputMode = VBatOutputMode.Modulated;
    candle.configAllSettings(candleConfiguration, 100);

    BatteryIndicator = new LEDSegment(0, 2, 0);
    PressureIndicator = new LEDSegment(2, 2, 1);
    MastEncoderIndicator = new LEDSegment(4, 1, -1);
    BoomEncoderIndicator = new LEDSegment(5, 1, -1);
    WristEncoderIndicator = new LEDSegment(6, 1, -1);
    DriverStationIndicator = new LEDSegment(7, 1, -1);
    MainStrip = new LEDSegment(8, 300, 2);
  }

  public static LEDSegment getSegment(int startBulb, int length) {
    return new LEDSegment(startBulb, length, -1);
  }

  public static LEDSegment getBulb(int bulb) {
    return getSegment(bulb, 1);
  }

  public void setBrightness(double percent) {
    candle.configBrightnessScalar(percent, 100);
  }

  public Command makeClearSegmentCommand(LEDSegment segment) {
    return runOnce(
        () -> {
          segment.clearAnimation();
          segment.disableLEDs();
        });
  }

  public Command makeWholeColorCommand(Color color) {
    return runOnce(
        () -> {
          setLEDs(color);
        });
  }

  public static int colorDoubleToInt(double value) {
    return (int) (value * 255.0);
  }

  public static void setLEDs(Color color) {
    candle.setLEDs(
        colorDoubleToInt(color.red), colorDoubleToInt(color.green), colorDoubleToInt(color.blue));
  }

  public static void setLEDs(Color color, LEDSegment segment) {
    candle.setLEDs(
        colorDoubleToInt(color.red),
        colorDoubleToInt(color.green),
        colorDoubleToInt(color.blue),
        0,
        segment.startIndex,
        segment.segmentSize);
  }

  public Command makeSegmentColorCommand(Color color, LEDSegment segment) {
    return runOnce(
        () -> {
          setLEDs(color, segment);
        });
  }

  public static class LEDSegment {

    public final int startIndex;
    public final int segmentSize;
    public final int animationSlot;

    private LEDSegment(int startIndex, int segmentSize, int animationSlot) {
      this.startIndex = startIndex;
      this.segmentSize = segmentSize;
      this.animationSlot = animationSlot;
    }

    public void setColor(Color color) {
      clearAnimation();
      setLEDs(color, this);
    }

    private void setAnimation(Animation animation) {
      candle.animate(animation, animationSlot);
    }

    public void fullClear() {
      clearAnimation();
      disableLEDs();
    }

    public void clearAnimation() {
      candle.clearAnimation(animationSlot);
    }

    public void disableLEDs() {
      setColor(Color.kBlack);
    }

    public void setFlowAnimation(Color color, double speed) {
      setAnimation(
          new ColorFlowAnimation(
              colorDoubleToInt(color.red),
              colorDoubleToInt(color.green),
              colorDoubleToInt(color.blue),
              0,
              speed,
              segmentSize,
              Direction.Forward,
              startIndex));
    }

    public void setFadeAnimation(Color color, double speed) {
      setAnimation(
          new SingleFadeAnimation(
              colorDoubleToInt(color.red),
              colorDoubleToInt(color.green),
              colorDoubleToInt(color.blue),
              0,
              speed,
              segmentSize,
              startIndex));
    }

    public void setBandAnimation(Color color, double speed) {
      setAnimation(
          new LarsonAnimation(
              colorDoubleToInt(color.red),
              colorDoubleToInt(color.green),
              colorDoubleToInt(color.blue),
              0,
              speed,
              segmentSize,
              BounceMode.Front,
              3,
              startIndex));
    }

    public void setStrobeAnimation(Color color, double speed) {
      setAnimation(
          new StrobeAnimation(
              colorDoubleToInt(color.red),
              colorDoubleToInt(color.green),
              colorDoubleToInt(color.blue),
              0,
              speed,
              segmentSize,
              startIndex));
    }

    public void setRainbowAnimation(double speed) {
      setAnimation(new RainbowAnimation(1, speed, segmentSize, false, startIndex));
    }
  }

  // public static class Color {
  //   public int red;
  //   public int green;
  //   public int blue;

  //   public Color(int red, int green, int blue) {
  //     this.red = red;
  //     this.green = green;
  //     this.blue = blue;
  //   }

  //   //     /**
  //   //      * Highly imperfect way of dimming the LEDs. It does not maintain color or
  //   //      * accurately adjust perceived brightness.
  //   //      *
  //   //      * @param dimFactor
  //   //      * @return The dimmed color
  //   //      */
  //   //     // public Color dim(double dimFactor) {
  //   //     //     int newRed = (int) (MathUtils.ensureRange(red * dimFactor, 0, 200));
  //   //     //     int newGreen = (int) (MathUtils.ensureRange(green * dimFactor, 0, 200));
  //   //     //     int newBlue = (int) (MathUtils.ensureRange(blue * dimFactor, 0, 200));

  //   //     //     return new Color(newRed, newGreen, newBlue);
  // }

  public Object setColorRGB(int r, int g, int b) {
    throw new UnsupportedOperationException("Unimplemented method 'setColorRGB'");
  }

  public void setFlowAnimation(int i, int j, int k, int l, int m, int n) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'setFlowAnimation'");
  }
}
