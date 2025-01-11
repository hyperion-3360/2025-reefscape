// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {

  AddressableLED m_led = new AddressableLED(6);
  AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(60);

  public LEDs() {
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  // spotless:off
  /** Sets a non-moving pattern to the LEDs 
   * (could potentially be remade to work with moving patterns) 
   */
  // spotless:on
  public Command setStillPattern(LEDPattern pattern) {
    return runOnce(() -> pattern.applyTo(m_ledBuffer));
  }

  @Override
  public void periodic() {
    m_led.setData(m_ledBuffer);
  }
}
