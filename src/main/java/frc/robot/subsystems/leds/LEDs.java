// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  public enum Pattern {
    INTAKE,
    READY,
    CLIMBER,
    SHOOTER,
    ELEVATOR,
    IDLE
  }

  private AddressableLED m_led = new AddressableLED(Constants.LEDConstants.kLEDPWMPort);
  private AddressableLEDBuffer m_ledBuffer =
      new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
  private AddressableLEDBuffer m_stageLedBuffer =
      new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);

  /** This variable should be able to be changed in smart dashboard */
  // double brightnessPercent = 0.0;
  private boolean m_isRainbow = false;

  private LEDPattern m_currentPattern = LEDPattern.rainbow(255, 255);
  private Distance LED_SPACING = Meters.of(1.0 / 27);
  private final LEDPattern m_scrollingRainbow =
      m_currentPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);

  private double multiplier = 0;
  private int pixelIndex = 0;
  double zeroPos = 0;

  // shuffleboard values
  private String tabName = "LEDs";
  private int r = 0;
  private int g = 0;
  private int b = 0;
  private int pulseSpeed = 0;
  private double pulseDelay = 0;

  private Color kTrueOrange = new Color(255, 10, 0);

  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());

    m_led.start();
    // SmartDashboard.putNumber("LED Brightness (%)", brightnessPercent);
    // LEDPattern.solid(Color.kGreen).applyTo(m_stageLedBuffer);
    m_isRainbow = false;
    // m_scrollingRainbow.applyTo(m_stageLedBuffer);
    stageLEDs();
  }

  // Our LED strip is WS2811, red and green are inverted. Swap red and green on all leds.
  private void stageLEDs() {
    for (int i = 0; i < m_stageLedBuffer.getLength(); i++) {
      Color color = m_stageLedBuffer.getLED(i);
      Color swappedRedGreen = new Color(color.green, color.red, color.blue);
      m_ledBuffer.setLED(i, swappedRedGreen);
    }
  }

  public void SetPattern(Pattern ledPattern) {
    switch (ledPattern) {
      case IDLE:
        {
          m_currentPattern = LEDPattern.solid(kTrueOrange);
          break;
        }
      case INTAKE:
        m_currentPattern =
            LEDPattern.solid(Color.kWhite)
                .blink(Time.ofBaseUnits(0.1, Second), Time.ofBaseUnits(0.1, Second));
        break;

      case ELEVATOR:
        m_currentPattern =
            LEDPattern.solid(Color.kWhite)
                .blink(Time.ofBaseUnits(0.1, Second), Time.ofBaseUnits(0.1, Second));
        break;

      case READY:
        m_currentPattern = LEDPattern.solid(Color.kGreen);
        break;

      case SHOOTER:
        m_currentPattern =
            LEDPattern.solid(Color.kWhite)
                .blink(Time.ofBaseUnits(0.05, Second), Time.ofBaseUnits(0.05, Second));
        break;

      case CLIMBER:
        m_currentPattern =
            LEDPattern.solid(Color.kWhite)
                .blink(Time.ofBaseUnits(0.05, Second), Time.ofBaseUnits(0.05, Second));
        break;
    }
  }

  // spotless:off
  /** Sets a non-moving pattern to the LEDs 
   * (could potentially be remade to work with moving patterns) 
   */
  // spotless:on
  public void setStillPattern(LEDPattern pattern) {
    pattern
        // .atBrightness(Percent.of(SmartDashboard.getNumber(getName(), brightnessPercent)))
        .applyTo(m_stageLedBuffer);
    m_isRainbow = false;
    stageLEDs();
  }

  @Override
  public void periodic() {
    m_currentPattern.applyTo(m_stageLedBuffer);
    stageLEDs();
    m_led.setData(m_ledBuffer);
  }

  /**
   * @param color
   */
  private void LEDPulsePattern(Color8Bit color, boolean fade) {

    for (int i = 0; i < (m_ledBuffer.getLength()) / 2; i++) {
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
        multiplier = 0.2;
        if (fade) {
          MathUtil.clamp(multiplier = (double) i / (double) pixelIndex, 0.2, 1);
        }
      }

      m_ledBuffer.setRGB(
          i,
          (int) MathUtil.clamp(color.red * multiplier, 0, 255),
          (int) MathUtil.clamp(color.green * multiplier, 0, 255),
          (int) MathUtil.clamp(color.blue * multiplier, 0, 255));

      m_ledBuffer.setRGB(
          (m_ledBuffer.getLength() - i) - 1,
          (int) MathUtil.clamp(color.red * multiplier, 0, 255),
          (int) MathUtil.clamp(color.green * multiplier, 0, 255),
          (int) MathUtil.clamp(color.blue * multiplier, 0, 255));
    }
    pixelIndex++;

    // if the pixelIndex is at the middle of the ledBuffer: restart it
    if (pixelIndex % (double) (m_ledBuffer.getLength()) / 2 == 0) {
      pixelIndex = 0;
    }
  }

  /**
   * @param color the color of the pulse
   * @param speed
   * @return
   */
  public Command setPulsePattern(Color8Bit color, double speed, double delay, boolean fade) {
    r = (int) SmartDashboard.getNumber("red", color.red);
    g = (int) SmartDashboard.getNumber("green", color.green);
    b = (int) SmartDashboard.getNumber("blue", color.blue);
    pulseSpeed = (int) SmartDashboard.getNumber("pulse speed", speed);
    pulseDelay = SmartDashboard.getNumber("pulse delay", delay);

    return Commands.repeatingSequence(
            this.runOnce(() -> LEDPulsePattern(new Color8Bit(r, g, b), fade)),
            new WaitCommand(pulseSpeed))
        .until(() -> pixelIndex % (double) (m_ledBuffer.getLength()) / 2 == 0)
        .andThen(new WaitCommand(pulseDelay))
        .repeatedly();
  }

  private void scrollingPattern() {
    LEDPattern base = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kOrange, Color.kBlack);
    LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Second).of(25));

    // Apply the LED pattern to the data buffer
    pattern.applyTo(m_ledBuffer);
  }

  public Command setScrollingPattern() {
    return this.runOnce(() -> scrollingPattern());
  }
}
