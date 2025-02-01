// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {

  AddressableLED m_led = new AddressableLED(Constants.LEDConstants.kLEDPWMPort);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);
  AddressableLEDBuffer m_stageLedBuffer =
      new AddressableLEDBuffer(Constants.LEDConstants.kLEDLength);

  /** This variable should be able to be changed in smart dashboard */
  double brightnessPercent = 0.0;

  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());

    m_led.start();
    SmartDashboard.putNumber("LED Brightness (%)", brightnessPercent);
    // LEDPattern.solid(Color.kGreen).applyTo(m_stageLedBuffer);
    LEDPattern.rainbow(255, 255).applyTo(m_stageLedBuffer);
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

  // spotless:off
  /** Sets a non-moving pattern to the LEDs 
   * (could potentially be remade to work with moving patterns) 
   */
  // spotless:on
  public void setStillPattern(LEDPattern pattern) {
    pattern
        // .atBrightness(Percent.of(SmartDashboard.getNumber(getName(), brightnessPercent)))
        .applyTo(m_stageLedBuffer);
    stageLEDs();
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }
}
