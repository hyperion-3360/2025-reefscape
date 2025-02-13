// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  public enum Pattern {
    INTAKE,
    READY,
    CLIMBER,
    SHOOTER,
    ELEVATOR,
    PULSE,
    IDLE
  }

  private AddressableLED m_led = new AddressableLED(Constants.LEDConstants.kLEDPWMPort);
  private AddressableLEDBuffer m_ledBuffer =
      new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);

  /** This variable should be able to be changed in smart dashboard */
  // double brightnessPercent = 0.0;
  private boolean m_isMovingPattern = true;

  private Distance LED_SPACING = Meters.of(1.0 / Constants.LEDConstants.kLEDLength);
  private LEDPattern m_currentPattern =
      LEDPattern.rainbow(255, 255).scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);

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

  // Our LED strip has dim red LEDS. This creates a better orange.
  private Color kTrueOrange = new Color(255, 10, 0);

  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.start();
    m_currentPattern.applyTo(m_ledBuffer);
    swapRedGreenLEDs();
  }

  // Our LED strip is WS2811, red and green are inverted. Swap red and green on all leds.
  private void swapRedGreenLEDs() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      Color color = m_ledBuffer.getLED(i);
      Color swappedRedGreen = new Color(color.green, color.red, color.blue);
      m_ledBuffer.setLED(i, swappedRedGreen);
    }
  }

  public void SetPattern(Pattern ledPattern) {
    switch (ledPattern) {
      case IDLE:
        m_currentPattern = LEDPattern.solid(kTrueOrange);
        m_isMovingPattern = false;
        break;

      case INTAKE:
        m_currentPattern = LEDPattern.solid(Color.kWhite).blink(Second.of(0.1));
        m_isMovingPattern = true;
        break;

      case ELEVATOR:
        m_currentPattern = LEDPattern.solid(Color.kWhite).blink(Second.of(0.1));
        m_isMovingPattern = true;
        break;

      case READY:
        m_currentPattern = LEDPattern.solid(Color.kGreen);
        m_isMovingPattern = false;
        break;

      case SHOOTER:
        m_currentPattern = LEDPattern.solid(Color.kWhite).blink(Second.of(0.05));
        m_isMovingPattern = true;
        break;

      case CLIMBER:
        m_currentPattern = LEDPattern.solid(Color.kWhite).blink(Second.of(0.05));
        m_isMovingPattern = true;
        break;

      case PULSE:
        m_currentPattern =
            LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kBlack, kTrueOrange)
                .scrollAtAbsoluteSpeed(MetersPerSecond.of(1), LED_SPACING);
        m_isMovingPattern = true;
        break;
    }
  }

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      SetPattern(Pattern.IDLE);
      m_currentPattern.applyTo(m_ledBuffer);
      swapRedGreenLEDs();
      m_led.setData(m_ledBuffer);
    }
    if (m_isMovingPattern) {
      m_currentPattern.applyTo(m_ledBuffer);
      swapRedGreenLEDs();
      m_led.setData(m_ledBuffer);
    }
  }
}
