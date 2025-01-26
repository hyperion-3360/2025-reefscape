package frc.robot.subsystems.leds;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color.RGBChannel;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  public enum LEDState {
    Orange,
    Rainbow,
    Green,
    Off
  }

  AddressableLED ledStrip = new AddressableLED(Constants.LEDConstants.kLEDPWMPort);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
  private double multiplier = 0;
  private int pixelIndex = 0;
  private int m_rainbowFirstPixelHue = 0;
  double zeroPos = 0;
  private int firstPos = 0;
  private LEDState m_currentState = LEDState.Off;

  public LEDs() {
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
  }

  @Override
  public void periodic() {
    // ExecLedState(m_currentState);
    // Rainbow();
    // Orange();
    // Move();
    // OrangeOnOff();
    ledStrip.setData(ledBuffer);
  }

  private void ExecLedState(LEDState ledState) {
    switch (ledState) {
      case Orange:
        OrangeOnOff();
        break;

      case Green:
        greenOnOff();
        break;

      case Rainbow:
        Rainbow();
        break;

      case Off:
        TurnOff();
        break;

      default:
        break;
    }
  }

  public Command SetLedState(LEDState state) {
    return this.run(() -> m_currentState = state);
  }

  private void TurnOff() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, 0, 0, 0);
    }
  }

  private void Rainbow() {

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      final int hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 128);
    }
    m_rainbowFirstPixelHue += 5;
    m_rainbowFirstPixelHue %= 180;
  }

  private void Orange() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 5 == 0) {
        i += 5;
      }
      ledBuffer.setRGB(i, 255, 35, 0);
    }

    ledStrip.setData(ledBuffer);
  }

  private void Move() {
    int prevPos = firstPos;

    ledBuffer.setRGB(prevPos, 0, 0, 0);
    firstPos += 1;

    if (firstPos >= ledBuffer.getLength()) {
      firstPos = 0;
    }

    ledBuffer.setRGB(firstPos, 255, 0, 0);
  }

  private void greenOnOff() {

    zeroPos += 0.03;
    for (int i = 0; i < ledBuffer.getLength(); i++) {

      // Create a "bumpy" waveform that shifts down the strip over time
      // Output range shoudl be [0,1]
      double pctDownStrip = (double) i / ledBuffer.getLength();
      double numCyclesOnStrip =
          (double) ledBuffer.getLength() / (double) ledBuffer.getLength() / 2.0;
      double colorBump =
          Math.sin(2 * Math.PI * numCyclesOnStrip * (pctDownStrip - zeroPos)) * 0.5 + 0.5;

      // Square the value so that the edge is sharper.
      colorBump *= colorBump;

      // Scale to LED units
      colorBump *= 30;

      // Set the pixel color
      ledBuffer.setRGB(
          i,
          (int) colorBump, // Red
          (int) colorBump, // Green
          0); // Blue
    }
  }

  private void OrangeOnOff() {

    zeroPos += 0.03;
    for (int i = 0; i < ledBuffer.getLength(); i++) {

      // Create a "bumpy" waveform that shifts down the strip over time
      // Output range shoudl be [0,1]
      double pctDownStrip = (double) i / ledBuffer.getLength() * 4;
      double numCyclesOnStrip =
          (double) ledBuffer.getLength() / (double) ledBuffer.getLength() / 2.0 * 4;
      double colorBump =
          Math.sin(Math.PI * numCyclesOnStrip * (pctDownStrip - zeroPos)) * 0.5 + 0.5;

      // Square the value so that the edge is sharper.
      colorBump *= colorBump;

      // Scale to LED units
      colorBump *= 30;

      double colorBumpR = colorBump * 8;

      // Set the pixel color
      ledBuffer.setRGB(
          i,
          (int) colorBumpR, // Red
          (int) colorBump, // Green
          0); // Blue
    }
  }

  /**
   * creates a sine wave and lerp between the two to mix the colors
   *
   * @param color1 the first color to blend with you can make a new color using new Color8Bit
   * @param color2 the second color to blend with you can make a new color using new Color8Bit
   * @param brightness in percentage [0, 1]
   * @param frequency the number of waves the function does per cycle
   * @param sharpness the sharpness of the sine wave
   */
  private void LEDSinWave(
      Color8Bit color1, Color8Bit color2, double brightness, double frequency, int sharpness) {
    zeroPos += 0.03;
    double multiplier = MathUtil.clamp(brightness, 0, 1);
    for (int i = 0; i < ledBuffer.getLength(); i++) {

      double pctDownStrip = (double) i / ledBuffer.getLength() * 4;
      double numCyclesOnStrip =
          (double) ledBuffer.getLength() / (double) ledBuffer.getLength() / 2.0 * 4;
      // this was graphed using desmos so it should be accurate
      double colorBump =
          Math.pow(
              Math.pow(
                  Math.sin((frequency * 2) * Math.PI * numCyclesOnStrip * (pctDownStrip - zeroPos)),
                  2),
              sharpness);

      // mixes the two colors using the sine wave value and sets their values
      int lerpedColor =
          Color.lerpRGB(
              MathUtil.clamp(color1.red * multiplier, 0, 255),
              MathUtil.clamp(color1.green * multiplier, 0, 255),
              MathUtil.clamp(color1.blue * multiplier, 0, 255),
              MathUtil.clamp(color2.red * multiplier, 0, 255),
              MathUtil.clamp(color2.green * multiplier, 0, 255),
              MathUtil.clamp(color2.blue * multiplier, 0, 255),
              colorBump);

      int redColor = Color.unpackRGB(lerpedColor, RGBChannel.kRed);

      int greenColor = Color.unpackRGB(lerpedColor, RGBChannel.kGreen);

      int blueColor = Color.unpackRGB(lerpedColor, RGBChannel.kBlue);

      ledBuffer.setRGB(i, redColor, greenColor, blueColor);
    }
  }

  /**
   * creates a sine wave and lerp between the two to mix the colors
   *
   * @param color1 the first color to blend with you can make a new color using new Color8Bit
   * @param color2 the second color to blend with you can make a new color using new Color8Bit
   * @param brightness in percentage [0, 1]
   * @param frequency the number of waves the function does per cycle
   * @param sharpness the sharpness of the sine wave
   */
  public Command setGradientPattern(
      Color8Bit color1, Color8Bit color2, double brightness, double frequency, int sharpness) {
    return this.run(() -> LEDSinWave(color1, color2, brightness, frequency, sharpness));
  }

  /**
   * @param color1
   */
  private void LEDPulsePattern(Color8Bit color1) {

    for (int i = 0; i < ledBuffer.getLength() / 2; i++) {
      // makes the "head" of the pulse effect
      if (pixelIndex - i == 0) {
        multiplier = 1;
      }
      // makes everything above the "head" be a constant brightness
      if (pixelIndex - i < 0) {
        multiplier = 0.2;
      }
      // makes a trailing tail effect behind the head of the LED
      if (pixelIndex - i > 0) {
        MathUtil.clamp(multiplier += 1 / pixelIndex, 0.2, 1);
      }

      ledBuffer.setRGB(
          i,
          (int) MathUtil.clamp(color1.red * multiplier, 0, 255),
          (int) MathUtil.clamp(color1.green * multiplier, 0, 255),
          (int) MathUtil.clamp(color1.blue * multiplier, 0, 255));

      ledBuffer.setRGB(
          (ledBuffer.getLength() - i) - 1,
          (int) MathUtil.clamp(color1.red * multiplier, 0, 255),
          (int) MathUtil.clamp(color1.green * multiplier, 0, 255),
          (int) MathUtil.clamp(color1.blue * multiplier, 0, 255));
    }
    pixelIndex++;
    // if the pixelIndex is at the middle of the ledBuffer: restart it
    if (pixelIndex % 1 / ((double) ledBuffer.getLength()) / 2 == 0) {
      pixelIndex = 0;
    }
  }
/**
 * 
 * @param color the color of the pulse
 * @param speed speed in ms (note, we subtract 20ms to the speed to account for periodic's 20ms loop)
 * @return
 */
  public Command setPulsePattern(Color8Bit color, double speed) {
    LEDPulsePattern(color);
    return this.runOnce(() -> LEDPulsePattern(color))
        .andThen(new WaitUntilCommand((speed - 20) / 1000).repeatedly());
  }
}
